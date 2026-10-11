#include "config.h"
#include "globals.h"
#include "crypto.h"
#include "radio.h"
#include "mesh.h"
#include "mc_platform.h"   // mcOnOwnFrameEcho: эхо своего кадра
#include <esp_random.h>  // esp_fill_random: запасная личность, если NVS не отвечает

uint8_t* findPeerPub(uint8_t hash) {
    for (int i = 0; i < PEER_CACHE_MAX; i++) {
        // Занятость слота — по явному признаку, а не по первому байту ключа: он бывает нулём.
        if (peerCache[i].used && peerCache[i].hash == hash) return peerCache[i].pub;
    }
    return NULL;
}

void rememberPeerPub(uint8_t hash, const uint8_t* pub) {
    int slot = -1;
    uint32_t oldest = 0xFFFFFFFF;
    for (int i = 0; i < PEER_CACHE_MAX; i++) {
        // Своя запись — обновляем её; иначе вытесняем самую старую. Свободный слот выигрывает
        // сам: last_seen у него нулевой, старше не бывает.
        if (peerCache[i].used && peerCache[i].hash == hash) { slot = i; break; }
        if (!peerCache[i].used) { slot = i; break; }
        if (peerCache[i].last_seen < oldest) { oldest = peerCache[i].last_seen; slot = i; }
    }
    peerCache[slot].used = true;
    peerCache[slot].hash = hash;
    memcpy(peerCache[slot].pub, pub, 32);
    peerCache[slot].last_seen = millis();
}

void initAdvertIdentity() {
    // Seed — случайные 32 байта, которые лежат в NVS (cfgIdentitySeed). Раньше здесь
    // стоял SHA256(cfg.name): имя узла открытым текстом уходит в каждом адверте, поэтому
    // приватный ключ мог вычислить любой, кто узел слышал, — и подделать его адверты, и
    // прочитать переписку в личке (общий секрет X25519 считается из этого же seed).
    if (!cfgIdentitySeed(bot_priv)) {
        // NVS недоступна. Случайная личность на один запуск лучше предсказуемой: сеть
        // увидит узел как новый, но ключ хотя бы не выводится из публичного имени.
        esp_fill_random(bot_priv, 32);
        Serial.println("[ADV] ВНИМАНИЕ: seed не из NVS — личность узла только до перезагрузки");
    }
    Ed25519::derivePublicKey(bot_pub, bot_priv);

    // Ed25519 private key (64 Б) для X25519-обмена при ответе в личку.
    // seed = тот же bot_priv, pub должен совпасть с bot_pub (RFC8032).
    uint8_t pub_check[32];
    ed25519_create_keypair(pub_check, bot_prv64, bot_priv);
    if (memcmp(pub_check, bot_pub, 32) != 0) {
        Serial.println("[ADV] WARNING: ed25519 pub mismatch (!)");
    }

    #ifndef BOT_ID_HASH
    ownShortHash = bot_pub[0];   // авто: hash ноды = первый байт pubkey
    #endif
    Serial.printf("[ADV] identity pub: ");
    for (int i = 0; i < 32; i++) Serial.printf("%02X", bot_pub[i]);
    Serial.printf(", own short hash: 0x%02X\n", ownShortHash);
}

// Дедуп-хэш кадра: [тип пакета 1B][тело после пути]. Путь в хэш не входит — от этого и
// зависят дедуп и защита от петель ретрансляции: одна и та же копия кадра, пришедшая
// другой дорогой (с другим набором ретрансляторов в пути), хэшируется так же и
// отбрасывается, а значит и не переиздаётся повторно. Возвращает false для битых кадров.
static bool meshFrameHash(const uint8_t* data, int len, uint8_t* payload_type, uint8_t out[32]) {
    if (len < 2) return false;
    uint8_t header = data[0];
    uint8_t pt = (header >> 2) & 0x0F;
    int offset = 1;
    if (((header & 0x03) == 0x00) || ((header & 0x03) == 0x03)) offset += 4;
    if (offset >= len) return false;
    uint8_t path_len = data[offset++];
    uint8_t hash_size = (path_len >> 6) + 1;
    uint8_t hop_count = path_len & 0x3F;
    offset += hop_count * hash_size;
    if (offset >= len) return false;

    // Контекст общий и настроен один раз: setup делает calloc, а эта функция зовётся на
    // КАЖДЫЙ принятый и каждый отправленный кадр (см. sha256Ctx в crypto.cpp).
    mbedtls_md_context_t* ctx = sha256Ctx();
    if (!ctx) return false;
    if (mbedtls_md_starts(ctx) != 0) return false;
    mbedtls_md_update(ctx, &pt, 1);
    mbedtls_md_update(ctx, &data[offset], len - offset);
    if (mbedtls_md_finish(ctx, out) != 0) return false;

    if (payload_type) *payload_type = pt;
    return true;
}

bool meshFrameHashOf(const uint8_t* data, int len, uint8_t out[32]) {
    return meshFrameHash(data, len, NULL, out);
}

// ===== Эхо своего кадра =====
//
// Свой ушедший кадр помечается как виденный, чтобы эхо от ретранслятора не прошло дедуп
// заново. Но само эхо — ценный сигнал: оно означает, что кадр ПРИНЯЛ и ПЕРЕИЗДАЛ кто-то
// ещё, то есть сообщение ушло в сеть, а не в пустоту. У лички это видно по
// подтверждению от получателя, а у канального сообщения подтверждать некому — и до сих
// пор о его судьбе не было известно ничего.
//
// Поэтому запоминаем хэши своих кадров отдельно и ждём их обратно. Таблица короткая:
// эхо приходит за секунды, и помнить дольше незачем.
#define OWN_ECHO_COUNT 4
#define OWN_ECHO_TTL_MS 60000

static uint8_t  ownEchoHash[OWN_ECHO_COUNT][SEEN_HASH_SIZE];
static uint32_t ownEchoMs[OWN_ECHO_COUNT] = {0};
static int      ownEchoNext = 0;

static void ownEchoRemember(const uint8_t* hash) {
    memcpy(ownEchoHash[ownEchoNext], hash, SEEN_HASH_SIZE);
    ownEchoMs[ownEchoNext] = millis();
    ownEchoNext = (ownEchoNext + 1) % OWN_ECHO_COUNT;
}

// Принятый кадр — наше ли это эхо. Возвращает true и гасит запись, чтобы вторая и третья
// копии флуда не считались вторым подтверждением.
static bool ownEchoMatch(const uint8_t* hash) {
    const uint32_t now = millis();
    for (int i = 0; i < OWN_ECHO_COUNT; i++) {
        if (!ownEchoMs[i]) continue;
        if ((uint32_t)(now - ownEchoMs[i]) > OWN_ECHO_TTL_MS) { ownEchoMs[i] = 0; continue; }
        if (memcmp(ownEchoHash[i], hash, SEEN_HASH_SIZE) != 0) continue;
        ownEchoMs[i] = 0;
        return true;
    }
    return false;
}

bool checkAndMarkSeen(uint8_t* data, int len) {
    uint8_t payload_type;
    uint8_t hash_ctx[32];
    if (!meshFrameHash(data, len, &payload_type, hash_ctx)) return true;  // битый пакет

    // выбираем кольцевой буфер по типу пакета
    int count  = (payload_type == 0x04) ? SEEN_ADVERT_HASH_COUNT : SEEN_HASH_COUNT;
    uint8_t* hashes = (payload_type == 0x04) ? seen_advert_hashes : seen_hashes;
    int* nextIdx    = (payload_type == 0x04) ? &seen_advert_next_idx : &seen_next_idx;

    // Наше собственное эхо: кадр переиздал кто-то ещё, значит он ушёл в сеть. Проверяем
    // ДО дедупа — дедуп про такой кадр скажет только «уже видел», и этим всё кончится.
    if (ownEchoMatch(hash_ctx)) mcOnOwnFrameEcho(hash_ctx);

    // ищем в кольцевом буфере
    for (int i = 0; i < count; i++) {
        if (memcmp(hash_ctx, &hashes[i * SEEN_HASH_SIZE], SEEN_HASH_SIZE) == 0) {
            return true;
        }
    }
    // помечаем
    memcpy(&hashes[*nextIdx * SEEN_HASH_SIZE], hash_ctx, SEEN_HASH_SIZE);
    *nextIdx = (*nextIdx + 1) % count;
    return false;
}

// Свой же кадр, ушедший в эфир, помечаем как «уже виденный»: его эхо, вернувшееся через
// ретранслятор, иначе прошло бы dedup как свежее, и узел переиздал бы собственное
// сообщение. Хэш не замечает пути, поэтому копия с достроенным путём матчится так же.
void markOwnFrameSeen(const uint8_t* data, int len) {
    uint8_t payload_type;
    uint8_t hash_ctx[32];
    if (!meshFrameHash(data, len, &payload_type, hash_ctx)) return;
    ownEchoRemember(hash_ctx);

    int count  = (payload_type == 0x04) ? SEEN_ADVERT_HASH_COUNT : SEEN_HASH_COUNT;
    uint8_t* hashes = (payload_type == 0x04) ? seen_advert_hashes : seen_hashes;
    int* nextIdx    = (payload_type == 0x04) ? &seen_advert_next_idx : &seen_next_idx;

    // занимаем слот как при приёме — перезаписываем самый старый
    memcpy(&hashes[*nextIdx * SEEN_HASH_SIZE], hash_ctx, SEEN_HASH_SIZE);
    *nextIdx = (*nextIdx + 1) % count;
}

