#include "config.h"
#include "crypto.h"
#include <mbedtls/md.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>

float cpuTempC() {
    return temperatureRead() - (float)TEMP_SENSOR_OFFSET;
}

// ===== CRC по полубайтам =====
// Таблицы на ШЕСТНАДЦАТЬ записей, а не на 256: 64 и 32 байта флеша против полутора
// килобайт, а работы вместо восьми сдвигов на байт — две выборки. Разница заметна там,
// где CRC считается по многу: crc16 на каждый кадр OTA, crc32 — на весь образ целиком,
// то есть на мегабайт с лишним за сессию.
//
// Полиномы и начальные значения прежние (0xEDB88320 отражённый и 0x1021 с 0xFFFF),
// поэтому в эфире и в файлах не меняется ничего: .otaz, собранные старой прошивкой,
// читаются новой и наоборот. Совпадение с побитовым вариантом проверено на всех длинах
// от нуля до 64 КБ и на счёте по кускам (так его и считает OTA).
static const uint32_t CRC32_NIB[16] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

static const uint16_t CRC16_NIB[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

uint32_t crc32_upd(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ CRC32_NIB[crc & 0x0F];
        crc = (crc >> 4) ^ CRC32_NIB[crc & 0x0F];
    }
    return crc;
}

uint16_t crc16buf(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)((crc << 4) ^ CRC16_NIB[((crc >> 12) ^ (data[i] >> 4)) & 0x0F]);
        crc = (uint16_t)((crc << 4) ^ CRC16_NIB[((crc >> 12) ^ (data[i] & 0x0F)) & 0x0F]);
    }
    return crc;
}

int encryptGroupText(const uint8_t* secret32, uint8_t* dest, const uint8_t* src, int src_len) {
    if (src_len <= 0) return 0;
    int padded = (src_len + 15) & ~15;
    uint8_t* cipher = dest + 2;  // MAC спереди

    // AES-128-ECB по блокам, хвост добиваем нулями
    uint8_t block[16];
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, secret32, 128);
    for (int i = 0; i < padded; i += 16) {
        memset(block, 0, 16);
        memcpy(block, src + i, min(16, src_len - i));
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, block, cipher + i);
    }
    mbedtls_aes_free(&aes);

    // HMAC-SHA256(cipher) с полным 32-байтным секретом
    uint8_t hmac_out[32];
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    secret32, 32, cipher, padded, hmac_out);
    dest[0] = hmac_out[0];
    dest[1] = hmac_out[1];
    return 2 + padded;
}

// Расшифровка в буфер вызывающего. Всё, что раньше делала decryptGroupText, живёт здесь:
// одна проверка HMAC, одна расшифровка. Текстовая обёртка ниже просто вырезает из результата
// строку. Две копии расшифровки разъехались бы молча — проверка HMAC в одной и отсутствие её
// в другой выглядят одинаково, пока кто-нибудь не подделает кадр.
int decryptGroupRaw(const uint8_t* secret32, uint8_t* mac, uint8_t* ciphertext, int len,
                    uint8_t* out, int outCap) {
    if (len <= 0 || len % 16 != 0) return 0;
    if (len > GROUP_TEXT_MAX || len > outCap) return 0;

    // Проверяем HMAC-SHA256(ciphertext) с полным 32-байтным секретом
    uint8_t hmac_out[32];
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    secret32, 32,
                    ciphertext, len, hmac_out);
    if (hmac_out[0] != mac[0] || hmac_out[1] != mac[1]) return 0;

    // AES-128-ECB расшифровка
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, secret32, 128);
    for (int i = 0; i < len; i += 16) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, ciphertext + i, out + i);
    }
    mbedtls_aes_free(&aes);
    return len;
}

// ===== Один контекст SHA-256 на всех =====
// mbedtls_md_setup внутри делает calloc, а md_info_from_type — линейный поиск по списку
// алгоритмов. И то и другое стояло на самом горячем пути: дедуп-хэш считается на КАЖДЫЙ
// принятый и каждый отправленный кадр (meshFrameHash), подтверждения доставки — на
// каждую личку. Это malloc/free в эфирном темпе, то есть и такт процессора, и
// фрагментация хипа. Возвраты при этом не проверялись: не хватило памяти — контекст
// остался ненастроенным, а код пошёл дальше.
//
// Настраиваем один раз. Цикл однопоточный, приём и передача идут из него же; задача
// проверки подписи на нулевом ядре сюда не заходит — у Ed25519 свой SHA-512.
static mbedtls_md_context_t shaCtx;
static bool shaReady = false;

mbedtls_md_context_t* sha256Ctx() {
    if (shaReady) return &shaCtx;
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return nullptr;
    mbedtls_md_init(&shaCtx);
    if (mbedtls_md_setup(&shaCtx, info, 0) != 0) {
        mbedtls_md_free(&shaCtx);
        return nullptr;
    }
    shaReady = true;
    return &shaCtx;
}

void sha256Trunc(uint8_t* out, int outLen, const uint8_t* a, int aLen,
                 const uint8_t* b, int bLen) {
    if (outLen <= 0 || outLen > 32) return;
    mbedtls_md_context_t* ctx = sha256Ctx();
    if (!ctx) { memset(out, 0, outLen); return; }
    uint8_t full[32];
    if (mbedtls_md_starts(ctx) != 0) { memset(out, 0, outLen); return; }
    if (a && aLen > 0) mbedtls_md_update(ctx, a, aLen);
    if (b && bLen > 0) mbedtls_md_update(ctx, b, bLen);
    if (mbedtls_md_finish(ctx, full) != 0) { memset(out, 0, outLen); return; }
    memcpy(out, full, outLen);
}

String decryptGroupText(const uint8_t* secret32, uint8_t* mac, uint8_t* ciphertext, int len) {
    if (len <= 0 || len % 16 != 0) return "";
    // Предел стоит ЗДЕСЬ, а не держится на вызывающем. Ниже расшифровка пишет в буфер ровно
    // len байт; сегодня len не может превысить 240 (кадр в эфире короче), но это свойство
    // единственного вызывающего, а не этой функции. Появится второй — по сети, из приложения,
    // из тестов, — и переполнение стека станет тихим.
    if (len > GROUP_TEXT_MAX) return "";

    uint8_t plaintext[GROUP_TEXT_MAX];
    if (decryptGroupRaw(secret32, mac, ciphertext, len, plaintext, sizeof(plaintext)) <= 0) {
        return "";
    }

    // plaintext: [timestamp 4B][txt_type 1B][text...]
    String message = "";
    for (int i = 5; i < len; i++) {
        if (plaintext[i] == 0) break;
        message += (char)plaintext[i];
    }
    return message;
}

int decryptRaw(const uint8_t* secret32, const uint8_t* mac, const uint8_t* ciphertext, int len, uint8_t* out, int out_size) {
    if (len <= 0 || len % 16 != 0) return 0;

    // Проверяем HMAC-SHA256(ciphertext)
    uint8_t hmac_out[32];
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    secret32, 32, ciphertext, len, hmac_out);
    if (hmac_out[0] != mac[0] || hmac_out[1] != mac[1]) return 0;

    // AES-128-ECB расшифровка
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, secret32, 128);
    // Расшифровка идёт блоками по 16 байт, поэтому в out влезает только целое число
    // блоков: при out_size, не кратном 16, последний блок вышел бы за буфер на до 15 байт.
    int n = min(len, out_size & ~15);
    if (n <= 0) return 0;
    for (int i = 0; i < n; i += 16) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, ciphertext + i, out + i);
    }
    mbedtls_aes_free(&aes);
    return n;
}

int privateKeyTo16(const String& keyb64, uint8_t key16[16]) {
    if (keyb64.length() == 0) return -1;   // автоключ
    size_t olen = 0;
    uint8_t tmp[32];
    int rc = mbedtls_base64_decode(tmp, sizeof(tmp), &olen,
                                   (const uint8_t*)keyb64.c_str(), keyb64.length());
    if (rc != 0 || olen != 16) {
        Serial.printf("[PRV] bad PSK '%s' (need 16 raw bytes in base64), using auto key\n", keyb64.c_str());
        return -1;
    }
    memcpy(key16, tmp, 16);
    return 1;                              // PSK-ключ
}

char* fmtFix(float v, uint8_t dec, char* buf, size_t n) {
    if (n == 0) return buf;
    long scale = 1;
    for (uint8_t i = 0; i < dec; i++) scale *= 10;
    long x = (long)(v * scale + (v >= 0 ? 0.5f : -0.5f));
    const char* sign = "";
    if (x < 0) { x = -x; sign = "-"; }
    if (dec == 0) snprintf(buf, n, "%s%ld", sign, x);
    else          snprintf(buf, n, "%s%ld.%0*ld", sign, x / scale, (int)dec, x % scale);
    return buf;
}

float parseFixed(const char* s) {
    while (*s == ' ') s++;
    bool neg = (*s == '-');
    if (neg || *s == '+') s++;
    long ip = 0;
    while (*s >= '0' && *s <= '9') ip = ip * 10 + (*s++ - '0');
    float frac = 0.0f, scale = 0.1f;
    if (*s == '.' || *s == ',') {
        s++;
        while (*s >= '0' && *s <= '9') { frac += (*s++ - '0') * scale; scale *= 0.1f; }
    }
    float v = (float)ip + frac;
    return neg ? -v : v;
}

void jsonEscape(const char* in, char* out, size_t outlen) {
    size_t n = 0;
    for (size_t i = 0; in[i] != 0 && n + 3 < outlen; i++) {
        char c = in[i];
        if (c == '"' || c == '\\') { out[n++] = '\\'; out[n++] = c; }
        else if (c == '\n') { out[n++] = '\\'; out[n++] = 'n'; }
        else if (c == '\r') { out[n++] = '\\'; out[n++] = 'r'; }
        else if (c == '\t') { out[n++] = '\\'; out[n++] = 't'; }
        else if ((uint8_t)c < 0x20) { /* остальные управляющие пропускаем */ }
        else out[n++] = c;
    }
    out[n] = 0;
}

// Микроградусы в строку "55.755826" (пришло из tdeck: координаты в hello сенсора)
char* fmtUdeg(int32_t udeg, char* buf, size_t n) {
    if (n == 0) return buf;
    const char* sign = "";
    // Знак снимается в 32-битном БЕЗ знака, а не через -udeg: при INT32_MIN у знакового
    // отрицания нет результата в типе (неопределённое поведение), и компилятор вправе
    // выкинуть проверку целиком. Значение приходит из NMEA и из настроек, то есть извне.
    uint32_t mag = (uint32_t)udeg;
    if (udeg < 0) { mag = (uint32_t)0 - mag; sign = "-"; }
    snprintf(buf, n, "%s%lu.%06lu", sign, (unsigned long)(mag / 1000000),
             (unsigned long)(mag % 1000000));
    return buf;
}
