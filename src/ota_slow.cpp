#include "config.h"
#include "globals.h"
#include "crypto.h"
#include "mesh.h"
#include "ota.h"
#include <esp_random.h>   // разброс паузы перед подтверждением

// ===== МЕДЛЕННЫЙ РЕЖИМ: ПРОШИВКА ПО САМОМУ MESHCORE =====
//
// Быстрый режим идёт сырыми FSK-кадрами на отдельном канале, и ретрансляторы их не
// переносят — дальше OTA_RADIO_MAX_HOPS он бесполезен. Здесь всё наоборот: образ едет
// обычными групповыми сообщениями сенсорного канала. Они шифруются тем же ключом, что и
// остальной трафик, ретранслируются флудом и доходят туда же, куда доходит любое сообщение
// сети. Цена — скорость: чанк 128 байт вместо 240, сообщение вместо сырого кадра, пауза на
// эфир вместо восьми миллисекунд. Образ едет часами, а не минуту.
//
// Роли разнесены намеренно: ОТПРАВЛЯЕТ прошивальщик (он стоит ближе к дальним узлам),
// СЛУШАЮТ подтверждения оба — и он, и координатор. Подтверждение идёт в тот же канал,
// поэтому его слышит каждый, кто в канале: отправитель двигает по нему окно, координатор
// показывает ход на странице. Если отправитель подтверждения не услышал, а узел его послал,
// повтор окна разбудит узел, и он подтвердит снова — потери здесь стоят времени, а не сессии.
//
// Подтверждение кумулятивное, как в TCP: «жду чанк N». Отправителю не нужна маска, узлу —
// не нужно помнить дыры, а потеря середины окна просто откатывает его на N.

#if FEATURE_MESH_OTA_SENDER

static File otaSlowFile;

// Экран ведущего. Хук тот же, что у быстрого режима: человеку не нужно различать режимы по
// виду экрана. Не чаще OTA_DRAW_MS — перерисовка OLED по I2C стоит около 25 мс.
static void otaSlowDraw() {
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw < OTA_DRAW_MS) return;
    lastDraw = millis();
    uint32_t pct = otaSlowChunks ? (otaSlowAcked * 100 / otaSlowChunks) : 0;
    if (pct > 100) pct = 100;
    mcUiOtaProgress(otaSlowTarget, pct, otaSlowAcked, lastRSSI, lastSNR);
}
bool otaSlowFinishedOk = false;   // итог последней медленной сессии — только для страницы

// Чанк по номеру: читаем из /ota.bin (это .otaz — заголовок плюс сжатый поток) и шлём как
// есть. Узел разворачивает поток тем же распаковщиком, что и в быстром режиме, поэтому
// здесь важно лишь резать ровно по OTA_SLOW_CHUNK_BYTES и не терять хвост.
static int otaSlowReadChunk(uint32_t seq, uint8_t* out) {
    if (!otaSlowFile) return -1;
    const uint32_t off = OTA_Z_HDR + seq * (uint32_t)OTA_SLOW_CHUNK_BYTES;
    if (!otaSlowFile.seek(off)) return -1;
    return otaSlowFile.read(out, OTA_SLOW_CHUNK_BYTES);
}

void otaSlowAbort(const char* why) {
    if (!otaSlowOn) return;
    otaSlowOn = false;
    if (otaSlowFile) { otaSlowFile.close(); otaSlowFile = File(); }
    snprintf(otaLastErr, sizeof(otaLastErr), "медленный режим: %s", why);
    mcUiOtaAbort(why);
    slog("[SLOW] прервано: %s\n", why);
    char msg[48];
    snprintf(msg, sizeof(msg), "%s%s", OTA_SLOW_MSG_FAIL, why);
    sensorSendMsg(msg);
}

bool otaSlowStart(const String& target) {
    if (otaSlowOn || otaSessionActive()) return false;
    if (sensorChannelIdx < 0) {
        strlcpy(otaLastErr, "канал сенсоров не настроен", sizeof(otaLastErr));
        return false;
    }
    otaSlowFile = LittleFS.open("/ota.bin", "r");
    if (!otaSlowFile) {
        strlcpy(otaLastErr, "нет образа для медленной прошивки", sizeof(otaLastErr));
        return false;
    }
    // В файле лежит .otaz: заголовок OTA_Z_HDR, дальше сжатый поток. Едет поток, размер и
    // CRC распакованного образа берутся из заголовка — узел проверит ими то, что собрал.
    const uint32_t zsize = (uint32_t)otaSlowFile.size() - OTA_Z_HDR;
    otaSlowTarget  = target;
    otaSlowTotal   = otaFwSize;
    otaSlowCrc     = otaFwCrc;
    otaSlowChunks  = (zsize + OTA_SLOW_CHUNK_BYTES - 1) / OTA_SLOW_CHUNK_BYTES;
    otaSlowSeq     = 0;
    otaSlowAcked   = 0;
    otaSlowRetries = 0;
    otaSlowOn      = true;
    otaSlowFinishedOk = false;   // итог прошлой сессии больше не показываем
    otaSlowNextMs  = millis();
    otaSlowAckMs   = 0;
    otaLastErr[0]  = 0;
    // otaPhase НЕ трогаем. По нему работает машина быстрого режима: увидев OTA_PHASE_DATA,
    // otaBotTick решает, что идёт его сессия, и начинает опрашивать узел — с otaSeq, который
    // остался от прошлой прошивки. В журнале это выглядело как «[SLOW] старт», а следом
    // «[OTA] poll seq=1158» и «abort (no response)»: медленная сессия будила быструю, и та
    // её же и убивала. Ход медленного режима страница берёт из otaSlow* напрямую.

    char msg[96];
    snprintf(msg, sizeof(msg), "%s%s:%u:%08X:%u", OTA_SLOW_MSG_START, target.c_str(),
             (unsigned)otaSlowTotal, (unsigned)otaSlowCrc, (unsigned)otaSlowChunks);
    // Одной посылкой, как чанки и подтверждения. Старт — тоже рукопожатие: узел отвечает
    // через OTA_SLOW_ACK_DELAY_MS, то есть пока флуд шлёт вторую и третью копию (с паузой
    // флуда это 1–2.3 с и до 4.6 с на весь флуд), его подтверждение падает в занятый эфир.
    // Маятник «в канале говорит только один» для этого нарушался ровно на старте сессии.
    // Потерянное подтверждение не страшно: окно повторяется целиком, попыток много.
    sensorSendMsg(msg, 0, 1);
    slog("[SLOW] старт -> '%s': %u байт, %u чанков по %d\n", target.c_str(),
         (unsigned)otaSlowTotal, (unsigned)otaSlowChunks, (int)OTA_SLOW_CHUNK_BYTES);
    return true;
}

// Подтверждение от узла. Слышат его и отправитель, и координатор — здесь его разбирает
// отправитель, чтобы двинуть окно.
void otaSlowOnAck(uint32_t next) {
    if (!otaSlowOn) return;
    if (next > otaSlowChunks) next = otaSlowChunks;
    if (next > otaSlowSeq) {
        otaSlowSeq = next;
        otaSlowAcked = next;
        otaSlowRetries = 0;
        otaSlowNextMs = millis();     // окно подтверждено — шлём следующее без паузы
        otaSlowAckMs = 0;
        otaSlowDraw();
        slog("[SLOW] подтверждено %u из %u\n", (unsigned)next, (unsigned)otaSlowChunks);
    } else if (next == otaSlowSeq) {
        // Узел ждёт то же, что мы и шлём: значит окно до него не дошло. Повторяем сразу, не
        // досиживая таймаут.
        otaSlowNextMs = millis();
        otaSlowAckMs = 0;
    }
}

void otaSlowDone(bool ok, const char* why) {
    if (!otaSlowOn) return;
    otaSlowOn = false;
    otaSlowFinishedOk = ok;   // страница покажет итог, не трогая фазу быстрого режима
    if (otaSlowFile) { otaSlowFile.close(); otaSlowFile = File(); }
    if (ok) {
        otaSlowAcked = otaSlowChunks;
        mcUiOtaDone(otaSlowTarget);
        snprintf(otaNote, sizeof(otaNote), "%s прошит по каналу", otaSlowTarget.c_str());
        slog("[SLOW] ГОТОВО: %s применил образ\n", otaSlowTarget.c_str());
    } else {
        snprintf(otaLastErr, sizeof(otaLastErr), "медленный режим: %s", why ? why : "отказ");
        slog("[SLOW] отказ узла: %s\n", why ? why : "?");
    }
}

// Шаг отправителя. Зовётся из главного цикла: ничего не ждёт, только смотрит на часы.
void otaSlowTick() {
    if (!otaSlowOn) return;

    // Фаза ответа: МОЛЧИМ до конца окна. Это и есть маятник — пока идёт очередь узла, мы не
    // отправляем ничего, даже если следующий чанк давно готов.
    if (otaSlowAckMs != 0) {
        if ((long)(millis() - otaSlowAckMs) < 0) return;
        if (++otaSlowRetries > OTA_SLOW_MAX_RETRIES) {
            otaSlowAbort("узел не подтверждает");
            return;
        }
        // Пауза перед повтором растёт с числом неудач: сеть могла замолчать надолго, и
        // долбить прежним темпом — мешать и себе, и остальным. Потолок не даёт паузе уйти
        // в бесконечность: связь возвращается, и мы должны это заметить.
        unsigned long wait = (unsigned long)otaSlowRetries * OTA_SLOW_RETRY_STEP_MS;
        if (wait > OTA_SLOW_RETRY_MAX_MS) wait = OTA_SLOW_RETRY_MAX_MS;
        slog("[SLOW] нет подтверждения, повтор окна с %u через %lu с (попытка %u из %d)\n",
             (unsigned)otaSlowSeq, wait / 1000, (unsigned)otaSlowRetries,
             (int)OTA_SLOW_MAX_RETRIES);
        otaSlowAckMs = 0;
        otaSlowNextMs = millis() + wait;
        return;
    }

    if ((long)(millis() - otaSlowNextMs) < 0) return;

    // Всё отправлено и подтверждено — ждём, пока узел применит образ и скажет об этом.
    if (otaSlowSeq >= otaSlowChunks) {
        otaSlowAckMs = millis() + OTA_SLOW_ACK_TIMEOUT_MS;
        return;
    }

    // Окно: сколько чанков ушло с последнего подтверждения.
    const uint32_t inWindow = otaSlowSeq + OTA_SLOW_WINDOW;
    static uint32_t sent = 0;
    if (sent < otaSlowSeq || sent >= inWindow) sent = otaSlowSeq;

    uint8_t raw[OTA_SLOW_CHUNK_BYTES];
    const int n = otaSlowReadChunk(sent, raw);
    if (n <= 0) { otaSlowAbort("образ не читается"); return; }

    // base64: hex удвоил бы объём, а образ и без того едет часами.
    unsigned char b64[OTA_SLOW_CHUNK_BYTES * 2];
    size_t b64len = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64), &b64len, raw, (size_t)n) != 0) {
        otaSlowAbort("чанк не кодируется");
        return;
    }
    char msg[OTA_SLOW_CHUNK_BYTES * 2 + 24];
    snprintf(msg, sizeof(msg), "%s%u:%.*s", OTA_SLOW_MSG_DATA, (unsigned)sent,
             (int)b64len, (const char*)b64);
    // Одной посылкой, без повторов: окно и так повторяется целиком, а лишняя копия каждого
    // чанка удвоила бы и без того долгую сессию.
    sensorSendMsg(msg, 0, 1);

    sent++;
    otaSlowDraw();
    if (sent >= inWindow || sent >= otaSlowChunks) {
        // Окно кончилось — ждём подтверждения.
        otaSlowAckMs = millis() + OTA_SLOW_ACK_TIMEOUT_MS;
    } else {
        otaSlowNextMs = millis() + OTA_SLOW_GAP_MS;
    }
}

#endif // FEATURE_MESH_OTA_SENDER

// ===== СТОРОНА УЗЛА =====
// Принимает чанки из канала, разворачивает поток тем же распаковщиком, что и быстрый режим
// (otaFeedStream), и подтверждает кумулятивно. Приём не требует переключения радио: это
// обычные сообщения, поэтому узел всё время остаётся на связи — в отличие от быстрого
// режима, где он сорок секунд глух ко всему, кроме чанков.
#if FEATURE_MESH_OTA_RECEIVER

static uint32_t slowRxExpect = 0;      // какой чанк ждём
static unsigned long slowRxLastMs = 0; // когда пришёл последний
static bool slowRxOn = false;
static unsigned long slowRxAckDueMs = 0;   // когда отправить отложенное подтверждение

// Экран принимающего узла: otaGot/otaTotal ведёт поток (обёртки в ota_receiver.cpp), поэтому
// и OLED, и раздел T-Deck показывают ход одинаково в обоих режимах.
static void slowRxDraw() {
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw < OTA_DRAW_MS) return;
    lastDraw = millis();
    mcUiOtaSensorProgress(otaGot, otaTotal, (uint32_t)packetCount, lastRSSI, lastSNR);
}

// Подтверждение уходит НЕ сразу, а через паузу: эфир после чанка ещё занят второй копией
// флуда и работой ретрансляторов, и мгновенный ответ ведущий просто не слышит. Заявка
// ставится здесь, отправляет её otaSlowRxTick.
static void slowRxAckLater() {
    if (slowRxAckDueMs != 0) return;    // заявка уже стоит — не двигаем её вперёд
    slowRxAckDueMs = millis() + OTA_SLOW_ACK_DELAY_MS + (esp_random() % OTA_SLOW_ACK_JITTER_MS);
    if (slowRxAckDueMs == 0) slowRxAckDueMs = 1;   // 0 занято признаком «заявки нет»
}

static void slowRxAckNow() {
    slowRxAckDueMs = 0;
    char msg[32];
    snprintf(msg, sizeof(msg), "%s%u", OTA_SLOW_MSG_ACK, (unsigned)slowRxExpect);
    sensorSendMsg(msg, 0, 1);
}

void otaSlowRxAbort(const char* why) {
    if (!slowRxOn) return;
    slowRxOn = false;
    otaSlowOn = false;
    otaSlowStreamEnd(false);
    char msg[48];
    snprintf(msg, sizeof(msg), "%s%s", OTA_SLOW_MSG_FAIL, why);
    sensorSendMsg(msg);
    mcUiOtaSensorAbort(why, 0, 0);
    Serial.printf("[SLOW] приём прерван: %s\n", why);
}

// "ota:slow:<target>:<байт>:<crc32>:<чанков>"
void otaSlowRxStart(const String& args) {
    int p1 = args.indexOf(':');
    int p2 = (p1 < 0) ? -1 : args.indexOf(':', p1 + 1);
    int p3 = (p2 < 0) ? -1 : args.indexOf(':', p2 + 1);
    if (p3 < 0) return;
    if (args.substring(0, p1) != cfg.name) return;    // не нам

    otaSlowTotal  = (uint32_t)strtoul(args.substring(p1 + 1, p2).c_str(), NULL, 10);
    otaSlowCrc    = (uint32_t)strtoul(args.substring(p2 + 1, p3).c_str(), NULL, 16);
    otaSlowChunks = (uint32_t)strtoul(args.substring(p3 + 1).c_str(), NULL, 10);
    if (otaSlowTotal == 0 || otaSlowChunks == 0) return;

    if (!otaSlowStreamBegin(otaSlowTotal, otaSlowCrc)) {
        otaSlowRxAbort("no RAM");
        return;
    }
    slowRxExpect = 0;
    slowRxAckDueMs = 0;
    slowRxOn = true;
    otaSlowOn = true;
    slowRxLastMs = millis();
    #if HAS_OLED
    screenWake();
    #endif
    Serial.printf("[SLOW] приём: %u байт, %u чанков\n", (unsigned)otaSlowTotal, (unsigned)otaSlowChunks);
    slowRxAckLater();   // говорим, с чего начинать — но не в занятый эфир
}

// "osd:<seq>:<base64>"
void otaSlowRxData(const String& args) {
    if (!slowRxOn) return;
    const int c = args.indexOf(':');
    if (c <= 0) return;
    const uint32_t seq = (uint32_t)strtoul(args.substring(0, c).c_str(), NULL, 10);
    slowRxLastMs = millis();

    // Не тот чанк, которого ждём: либо повтор уже записанного, либо кусок будущего окна.
    // Записать его некуда — поток разворачивается строго по порядку, — поэтому просто
    // напоминаем отправителю, с чего продолжать.
    if (seq != slowRxExpect) {
        if (seq > slowRxExpect) slowRxAckLater();
        return;
    }

    const String b64 = args.substring(c + 1);
    uint8_t raw[OTA_SLOW_CHUNK_BYTES + 8];
    size_t rawLen = 0;
    if (mbedtls_base64_decode(raw, sizeof(raw), &rawLen,
                              (const unsigned char*)b64.c_str(), b64.length()) != 0) {
        return;   // битая кодировка — просто ждём повтора окна
    }
    const bool last = (seq + 1 >= otaSlowChunks);
    if (!otaSlowStreamFeed(raw, rawLen, last)) {
        otaSlowRxAbort("write fail");
        return;
    }
    slowRxExpect = seq + 1;
    slowRxDraw();

    // Подтверждаем на границе окна и на последнем чанке: подтверждать каждый — значит
    // занять эфир ответами вдвое плотнее, чем данными.
    if (slowRxExpect >= otaSlowChunks || (slowRxExpect % OTA_SLOW_WINDOW) == 0) slowRxAckLater();

    if (slowRxExpect >= otaSlowChunks) {
        slowRxAckNow();       // последнее подтверждение шлём без паузы: ждать уже нечего
        slowRxOn = false;
        otaSlowOn = false;
        const bool ok = otaSlowStreamEnd(true);
        char msg[64];
        if (ok) {
            snprintf(msg, sizeof(msg), "%s%s", OTA_SLOW_MSG_DONE, cfg.name.c_str());
            sensorSendMsg(msg);
            Serial.printf("[SLOW] образ принят и проверен — применяю\n");
            delay(400);           // дать сообщению уйти в эфир до перезагрузки
            ESP.restart();
        } else {
            otaSlowRxAbort("crc mismatch");
        }
    }
}

// Сторож: отправитель мог перезагрузиться, и держать раздел открытым вечно незачем.
void otaSlowRxTick() {
    if (!slowRxOn) return;
    // Отложенное подтверждение: пауза дана, чтобы эфир освободился от чанка и его повторов.
    if (slowRxAckDueMs != 0 && (long)(millis() - slowRxAckDueMs) >= 0) slowRxAckNow();
    if (millis() - slowRxLastMs < OTA_SLOW_STALL_MS) return;
    otaSlowRxAbort("тишина в канале");
}

#endif // FEATURE_MESH_OTA_RECEIVER
