#include "config.h"
#include "globals.h"
#include "radio.h"
#include "mesh.h"

// ===== Ретрансляция чужих флуд-кадров (ВЫКЛЮЧЕНА ПО УМОЛЧАНИЮ) =====
// Ретранслятором не должен быть никто: FEATURE_RELAY по умолчанию 0 (значение задаёт
// config.h ядра), и весь этот файл в обычную сборку не попадает — ниже остаются только две
// пустышки, чтобы линковке было что звать. Возможность при этом не потеряна: сборка с
// -DFEATURE_RELAY=1 возвращает всё, что описано дальше.
//
// Чем оно было, когда работало: услышавший кадр добавляет в путь свой хэш и переиздаёт его,
// чтобы сообщение уходило дальше одного хопа. Раньше в кадре всегда был пустой путь — узел
// умел читать маршруты, но никто их не строил, и сообщения умирали на первом же хопе.
//
// Две ошибки, из-за которых включать это было нельзя, закрыты: meshRelayTick передаёт один
// кадр за тик (раньше — всю очередь одним проходом, до шестнадцати секунд в главном цикле), и
// кадр с чужим размером хэша хопа не переиздаётся вовсе (раньше путь ему портился). На железе
// ретрансляция всё равно не прогонялась ни разу: признак выключен у всех.
//
// Переиздание отделено от разбора и не блокирует главный цикл: кадр кладётся в маленькую
// очередь со случайной паузой (отправитель ещё повторяет свой флуд, а ретрансляторы,
// услышавшие тот же кадр, должны разойтись по времени, иначе нагрузят друг друга), а
// переиздаёт его главный цикл, когда пауза истечёт. Дедуп делает остальное: копия кадра,
// пришедшая с уже достроенным путём, хэшируется так же и отбрасывается (путь в хэш не
// входит), поэтому петли не возникает. Своё эхо узел не переиздаёт потому, что любой
// собственный кадр помечается «уже виденным» на передаче (markOwnFrameSeen).

#if FEATURE_RELAY

// Отложенные переиздания: пауза — чтобы не попасть в повторы отправителя и не столкнуться
// с другими ретрансляторами, услышавшими тот же кадр.
#ifndef RELAY_QUEUE_MAX
#define RELAY_QUEUE_MAX 8
#endif
#ifndef RELAY_DELAY_MIN_MS
#define RELAY_DELAY_MIN_MS 1400
#endif
#ifndef RELAY_DELAY_MAX_MS
#define RELAY_DELAY_MAX_MS 2500
#endif
#ifndef MAX_RELAY_HOPS
#define MAX_RELAY_HOPS 32
#endif
// Сколько кадров помним как «уже переиздали». Память маленькая намеренно: она живёт не для
// защиты от петель (для этого общий дедуп в radio_rx), а чтобы не поставить в очередь одну и
// ту же копию дважды. Стоит дешевле, чем лишняя передача в эфир.
#ifndef RELAY_SEEN_COUNT
#define RELAY_SEEN_COUNT 24
#endif

static struct {
    unsigned long dueMs;         // 0 — слот свободен
    uint8_t frame[256];
    int len;
} relayQueue[RELAY_QUEUE_MAX];

static uint8_t relaySeen[RELAY_SEEN_COUNT * SEEN_HASH_SIZE];
static int relaySeenNext = 0;

static bool relayWasQueued(const uint8_t hash[32]) {
    for (int i = 0; i < RELAY_SEEN_COUNT; i++) {
        if (memcmp(hash, &relaySeen[i * SEEN_HASH_SIZE], SEEN_HASH_SIZE) == 0) return true;
    }
    return false;
}

static void relayMarkQueued(const uint8_t hash[32]) {
    memcpy(&relaySeen[relaySeenNext * SEEN_HASH_SIZE], hash, SEEN_HASH_SIZE);
    relaySeenNext = (relaySeenNext + 1) % RELAY_SEEN_COUNT;
}

// Решение и постановка в очередь. data/len — принятый кадр ДО переиздания (с исходным путём).
// Возвращает, что стало с кадром: см. RELAY_* в mesh.h.
int maybeQueueRelay(const uint8_t* data, int len) {
    // Быстрый канал mesh OTA — односкачный (сырые кадры ретрансляторы не переносят).
    if (otaFastMode) return RELAY_SKIPPED;
    if (len < 2) return RELAY_SKIPPED;
    uint8_t header = data[0];
    uint8_t payload_type = (header >> 2) & 0x0F;
    uint8_t route_type = header & 0x03;

    // Транспортные коды (0x00/0x03) — служебный обмен между соседями, и direct (0x02)
    // адресован одному хопу; переиздаются только flood-кадры (0x01).
    if (payload_type == 0x00 || payload_type == 0x03) return RELAY_SKIPPED;
    if (route_type != 0x01) return RELAY_SKIPPED;

    // Уже переиздавали этот кадр — в очередь второй раз его незачем. Проверка идёт ДО всего
    // остального, и поэтому работает даже для копии, которую общий дедуп уже отбросил: такая
    // копия — это шанс поставить кадр в очередь после того, как очередь освободилась.
    uint8_t hash[32];
    if (!meshFrameHashOf(data, len, hash)) return RELAY_SKIPPED;
    if (relayWasQueued(hash)) return RELAY_SKIPPED;

    int offset = 1;              // у flood-кадров байт пути стоит сразу после заголовка
    if (offset >= len) return RELAY_SKIPPED;
    uint8_t path_len = data[offset++];
    uint8_t path_hash_size = (path_len >> 6) + 1;
    uint8_t hop_count = path_len & 0x3F;

    // Размер хэша хопа кадр объявляет сам (1…4 байта), а свой путь мы строим константой
    // PATH_HASH_SIZE. Пока они совпадают, всё сходится; когда нет — переиздание портит кадр
    // непоправимо. Раньше именно так и было: шаг по чужому пути считался размером ИЗ КАДРА,
    // сравнение «нет ли меня в пути» читало PATH_HASH_SIZE байт своей константой, а
    // пересобранный кадр писал в path_len снова свой размер поверх пути, набранного чужим
    // шагом. Для кадра с однобайтовым хэшем выходил путь, который не разберёт никто —
    // а однобайтовый хэш как раз у оригинального MeshCore, и встретить его в общем канале
    // штатно.
    //
    // Поэтому такой кадр не переиздаётся вовсе. Достроить чужой путь мы не можем: наши два
    // байта на хоп в их однобайтовую разметку не уложить, а переписать разметку — значит
    // соврать про уже пройденные хопы. Следствие, которое надо знать: для сети с другим
    // размером хэша мы не ретранслятор ни на одном кадре, включая кадры без пути. Это
    // осознанный отказ, а не недоделка.
    if (path_hash_size != PATH_HASH_SIZE) {
        Serial.printf("[RLY] хэш хопа %u Б, у нас %u Б — чужая разметка пути, не переиздаём\n",
                      (unsigned)path_hash_size, (unsigned)PATH_HASH_SIZE);
        return RELAY_SKIPPED;
    }
    if (hop_count >= MAX_RELAY_HOPS) return RELAY_SKIPPED;   // путь упёрся в потолок
    int pathBytes = hop_count * path_hash_size;
    if (offset + pathBytes >= len) return RELAY_SKIPPED;     // битый кадр: тело пустое

    // Наш хэш уже есть в пути — кадр когда-то прошёл через нас (петля или эхо) —
    // повторно в него не вписываемся.
    for (int h = 0; h < hop_count; h++) {
        if (memcmp(&data[offset + h * path_hash_size], bot_pub, PATH_HASH_SIZE) == 0) {
            return RELAY_SKIPPED;
        }
    }

    // Личку НЕ для нас не разносим (у каждого свои ключи, читать её кроме адресата никто
    // не сможет), а личку для нас дальше передавать незачем — мы и есть получатель.
    if (payload_type == 0x02) {
        if (offset + pathBytes + 2 > len) return RELAY_SKIPPED;
        if (data[offset + pathBytes] == ownShortHash) return RELAY_SKIPPED;
    }

    if (len + PATH_HASH_SIZE > 255) return RELAY_SKIPPED;   // кадр не влезает в SX1262

    // Строим переиздаваемый кадр: тот же заголовок и тело, путь — исходный плюс наш хэш
    // в конце. Узел добавляет в путь первые два байта своего публичного ключа: тот же
    // источник, что и ownShortHash, только длиной PATH_HASH_SIZE.
    uint8_t fwd[256];
    fwd[0] = data[0];
    fwd[1] = ((PATH_HASH_SIZE - 1) << 6) | (hop_count + 1);
    memcpy(fwd + 2, data + 2, pathBytes);                                    // исходный путь
    memcpy(fwd + 2 + pathBytes, bot_pub, PATH_HASH_SIZE);                    // наш хэш
    memcpy(fwd + 2 + pathBytes + PATH_HASH_SIZE, data + 2 + pathBytes,       // тело
           len - (2 + pathBytes));
    int fwdLen = len + PATH_HASH_SIZE;

    // Свободный слот очереди; полная — кадр не переиздаём, но НЕ запоминаем его хэш:
    // следующая копия от отправителя получит шанс, и кадр дойдёт, как только очередь
    // разгрузится. Раньше кадр в такой ситуации помечался «увиденным» общим дедупом и
    // терялся целиком — вместе со всеми копиями, потому что дальше они считались
    // дубликатами.
    int slot = -1;
    for (int i = 0; i < RELAY_QUEUE_MAX; i++) {
        if (relayQueue[i].dueMs == 0) { slot = i; break; }
    }
    if (slot < 0) {
        relayQueueDrops++;
        Serial.printf("[RLY] очередь переполнена (%u ждут) — ждём следующую копию (всего %lu)\n",
                      (unsigned)RELAY_QUEUE_MAX, (unsigned long)relayQueueDrops);
        return RELAY_NOROOM;
    }

    relayQueue[slot].dueMs = millis() + random(RELAY_DELAY_MIN_MS, RELAY_DELAY_MAX_MS);
    if (relayQueue[slot].dueMs == 0) relayQueue[slot].dueMs = 1;   // 0 занято признаком «свободен»
    memcpy(relayQueue[slot].frame, fwd, fwdLen);
    relayQueue[slot].len = fwdLen;
    relayMarkQueued(hash);

    Serial.printf("[RLY] type=%u hops=%u->%u, переиздание через %lu мс\n",
                  payload_type, hop_count, hop_count + 1,
                  (unsigned long)(relayQueue[slot].dueMs - millis()));
    return RELAY_QUEUED;
}

void meshRelayTick() {
    // ОДИН кадр за тик. Раньше цикл шёл по всем восьми слотам и для каждого просроченного
    // звал txFrame, а тот ждёт канал до CAD_WAIT_BUDGET_MS (1.5 с) и держит кадр в эфире
    // ещё около полусекунды: полная очередь останавливала главный цикл почти на шестнадцать
    // секунд. И это не редкий случай — паузы берутся из одного диапазона
    // RELAY_DELAY_MIN_MS…MAX_MS, поэтому кадры, принятые подряд, просрочиваются вместе.
    // Остановленный главный цикл — это не только молчащий ретранслятор: в нём же тикают
    // heartbeat, сторожа сессий прошивки и опрос кнопки.
    //
    // Берём самый просроченный, а не первый попавшийся: слоты обходятся по индексу, и при
    // выборе «первого» кадр в старшем слоте мог бы ждать, пока младшие занимают каждый тик.
    // Сравнение через (long)(now - dueMs) — оно переживает переполнение millis().
    unsigned long now = millis();
    int best = -1;
    long bestLate = -1;
    for (int i = 0; i < RELAY_QUEUE_MAX; i++) {
        if (relayQueue[i].dueMs == 0) continue;
        long late = (long)(now - relayQueue[i].dueMs);
        if (late < 0) continue;                 // срок ещё не подошёл
        if (late > bestLate) { bestLate = late; best = i; }
    }
    if (best < 0) return;
    relayQueue[best].dueMs = 0;

    // Никакого hex-лога кадра: на 245-байтном кадре он стоит ~40 мс UART, и вся эта
    // задержка встаёт перед передачей — то есть ретранслятор выходит в эфир позже
    // соседей и глуше вероятность. В журнал уходит строка о постановке в очередь.
    relayForwardedCount++;
    txFrame(relayQueue[best].frame, relayQueue[best].len);
}

#else   // !FEATURE_RELAY — обычная сборка: функции пустые, линковке есть что звать.
        // Это путь по умолчанию, а не аварийный: ретранслятором не должен быть никто.
        // Заглушки нужны затем, чтобы radio_rx.cpp и главный цикл прошивки звали их без
        // #if вокруг каждого вызова — условие живёт в одном месте, здесь.
int maybeQueueRelay(const uint8_t*, int) { return RELAY_SKIPPED; }
void meshRelayTick() {}
#endif