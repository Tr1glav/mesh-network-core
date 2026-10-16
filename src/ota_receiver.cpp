#include "config.h"
#include "globals.h"
#include "crypto.h"
#include "radio.h"
#include "mesh.h"
#include "ota.h"

// ===== Приём прошивки по радио: сторона узла =====
// Выделено из ota.cpp. Этот код не делит ни одной переменной с раздающей стороной и с
// веб-страницей, поэтому отделяется начисто и собирается только там, где нужен:
// распаковка потока на лету, запись в раздел прошивки, окно подтверждений и проверка
// образа перед применением.

#if FEATURE_MESH_OTA_RECEIVER

extern "C" {
#include "esp32s3/rom/miniz.h"
}

static uint32_t otaStreamLen = 0; // байт в сжатом потоке
static tinfl_decompressor *otaInfl = NULL;
static uint8_t *otaDict = NULL; // окно распаковки, оно же буфер вывода
static size_t otaDictOfs = 0;
static uint8_t otaWin[OTA_WINDOW][OTA_RAW_CHUNK_BYTES]; // слот = seq % OTA_WINDOW
static uint8_t otaWinLen[OTA_WINDOW];
static uint16_t otaWinMask = 0; // бит i: чанк otaSeqExp+i уже в окне
static FwScan otaImgScan;       // маркер платы в распакованном образе

static void otaZFree() {
    free(otaInfl);
    otaInfl = NULL;
    free(otaDict);
    otaDict = NULL;
}

// ===== КУДА УХОДИТ ПРИНЯТЫЙ ПОТОК =====
// Mesh OTA доставляет N байт с CRC; смысла этих байт протокол не касается — окна,
// подтверждения, повторы, разжатие на лету и бюджеты эфира одинаковы для любого груза.
// Различается только место назначения, и оно выбирается здесь, по виду груза из
// объявления сессии:
//
//   прошивка — раздел через Update, маркер платы, в конце перезагрузка;
//   прочее   — прошивке платы через хуки mcOtaData* (ядро про файловые системы не знает).
//
// Вид приходит из эфира, а место назначения — нет: отправитель говорит «это пакет», куда
// его класть, решает приёмник. Иначе вышел бы примитив удалённой записи в файловую систему.
static OtaKind otaKind = OTA_KIND_FIRMWARE;

static bool otaSinkBegin(uint32_t total) {
    if (otaKind == OTA_KIND_FIRMWARE) {
        fwScanReset(&otaImgScan); // маркер платы ищется по ходу записи, см. otaSinkWrite
        if (!Update.begin(total)) {
            Update.printError(Serial);
            return false;
        }
        return true;
    }
    return mcOtaDataBegin(total);
}

static bool otaSinkWrite(const uint8_t *data, size_t n) {
    if (otaKind == OTA_KIND_FIRMWARE) {
        if (Update.write((uint8_t *)data, n) != n) {
            Update.printError(Serial);
            return false;
        }
        // Маркер платы — свойство прошивки: у пакета его нет и быть не должно, поэтому
        // сканер живёт здесь, а не в общем пути приёма.
        fwScanFeed(&otaImgScan, data, n);
        return true;
    }
    return mcOtaDataWrite(data, n);
}

// Поток принят целиком и сошёлся по CRC — применить. why заполняется причиной отказа:
// её узел называет ведущему, иначе тот видит только таймаут.
static bool otaSinkApply(const char **why) {
    if (otaKind == OTA_KIND_FIRMWARE) {
        // Прошивка другой платы не стартует, и узел придётся снимать и шить по USB.
        if (fwScanVerdict(&otaImgScan) < 0) {
            Serial.printf("[OTA] образ платы %s, а это " BOARD_CODE " — отказ\n", otaImgScan.other);
            *why = "board mismatch";
            return false;
        }
        if (!Update.end(true)) {
            Update.printError(Serial);
            *why = "end fail";
            return false;
        }
        return true;
    }
    if (!mcOtaDataApply()) {
        *why = "install fail";
        return false;
    }
    return true;
}

static void otaSinkAbort() {
    if (otaKind == OTA_KIND_FIRMWARE) {
        Update.abort();
    } else {
        mcOtaDataAbort();
    }
}

// Нужна ли перезагрузка после успеха. Прошивка — да, иначе она не вступит в силу; пакет —
// нет: он уже установлен, а перезагрузка ради него оборвала бы связь на полминуты.
static bool otaKindNeedsReboot() {
    return otaKind == OTA_KIND_FIRMWARE;
}

static bool otaWriteImage(const uint8_t *data, size_t n) {
    // Распаковщик может выдать больше объявленного: в конце потока он дополняет
    // последний блок. Лишнее не пишем — у прошивки Update.write упрётся в границу раздела.
    if (otaGot + n > otaTotal)
        n = otaTotal - otaGot;
    if (n == 0)
        return true;
    if (!otaSinkWrite(data, n))
        return false;
    otaCrcAcc = crc32_upd(otaCrcAcc, data, n);
    otaGot += n;
    return true;
}

// last = это последний чанк сжатого потока. Флаг «вход ещё будет» на нём снимается:
// с ним распаковщик не берётся за символы, которым не хватает бит в буфере, и
// придерживает хвост образа до следующей порции — которой уже не будет. Так терялись
// последние байты (на образе 1106000 доходило 1103279), и приём падал с «size mismatch».
static void otaSensorAckstart() {
    if (sensorChannelIdx < 0)
        return;
    uint8_t frame[256];
    int f = buildGroupFrameFlood(sensorChannelIdx, OTA_ACKSTART, frame, sizeof(frame));
    if (f <= 0)
        return;
    // Бюджет рукопожатия посчитан для кадра не длиннее OTA_ACKSTART_FRAME_MAX: бот ждёт
    // OTA_FAST_SETTLE_MS и начинает слать пачку, а сенсор к этому моменту обязан успеть
    // договорить обе копии и переключить радио. Длина кадра зависит от имени узла, и с
    // длинным именем бюджет перестаёт сходиться. Отправку не отменяем — сессия нужна, — но
    // молчать нельзя: иначе сорванное рукопожатие выглядит как потеря в эфире.
    if (f > OTA_ACKSTART_FRAME_MAX) {
        Serial.printf(
            "[OTA] ackstart %d Б > бюджета %d Б: имя узла длинное, бот может начать "
            "пачку раньше, чем сенсор уйдёт на быстрый канал\n",
            f, (int)OTA_ACKSTART_FRAME_MAX);
    }
    for (int i = 0; i < OTA_ACKSTART_COPIES; i++) {
        if (i)
            delay(OTA_ACKSTART_GAP_MS);
        sendFrame(sensorChannelIdx, frame, f, i == 0);
    }
}

static bool otaFeed(const uint8_t *in, size_t n, bool last) {
    for (;;) {
        size_t inBytes = n;
        size_t outBytes = TINFL_LZ_DICT_SIZE - otaDictOfs;
        tinfl_status st = tinfl_decompress(otaInfl, in, &inBytes, otaDict, otaDict + otaDictOfs, &outBytes,
                                           TINFL_FLAG_PARSE_ZLIB_HEADER | (last ? 0 : TINFL_FLAG_HAS_MORE_INPUT));
        in += inBytes;
        n -= inBytes;
        if (outBytes > 0 && !otaWriteImage(otaDict + otaDictOfs, outBytes))
            return false;
        otaDictOfs = (otaDictOfs + outBytes) & (TINFL_LZ_DICT_SIZE - 1);
        if (st < 0) {
            Serial.printf("[OTA] inflate error %d\n", (int)st);
            return false;
        }
        if (st == TINFL_STATUS_DONE || (n == 0 && st != TINFL_STATUS_HAS_MORE_OUTPUT))
            return true;
    }
}

static bool otaFlushWindow() {
    while (otaWinMask & 1) {
        uint8_t slot = otaSeqExp % OTA_WINDOW;
        uint32_t off = otaSeqExp * OTA_RAW_CHUNK_BYTES;
        bool last = (off + otaWinLen[slot] >= otaStreamLen);
        if (!otaFeed(otaWin[slot], otaWinLen[slot], last))
            return false;
        otaWinMask >>= 1;
        otaSeqExp++;
    }
    return true;
}

static void otaSendWack() {
    uint8_t mask[2] = {(uint8_t)otaWinMask, (uint8_t)(otaWinMask >> 8)};
    uint8_t f[16];
    rawTxFrame(f, rawBuildFrame(f, RAW_TYPE_WACK, otaSeqExp, mask, 2));
}

static void otaRawFail(const char *why) {
    // Причину кладём прямо в кадр. Раньше он уходил пустым, координатор писал в журнал
    // обобщённое «sensor fail», а настоящая причина (crc mismatch, end fail, write fail)
    // оставалась в консоли узла — к которому в рабочей сети обычно не подключиться.
    uint8_t f[48];
    size_t n = strnlen(why, 20);
    rawTxFrame(f, rawBuildFrame(f, RAW_TYPE_FAIL, 0, (const uint8_t *)why, n));
    otaSensorAbort(why);
}

// Приём raw-фреймов на сенсоре (бот -> сенсор) во время чистой LoRa OTA
void otaHandleRawSensor(const uint8_t *buf, int len) {
    if (!otaActive)
        return;
    if (len < 9 || buf[0] != RAW_MAGIC0 || buf[1] != RAW_MAGIC1)
        return;
    int paylen = len - 2;
    uint16_t c = (uint16_t)(buf[len - 1] << 8) | buf[len - 2];
    if (crc16buf(buf, paylen) != c)
        return;
    uint8_t type = buf[2];
    uint32_t seq = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);

    if (type == RAW_TYPE_ABORT) {
        otaSensorAbort("from bot");
        return;
    }

    if (type == RAW_TYPE_DATA || type == RAW_TYPE_DATA_LAST) {
        if (!otaGotStart)
            return;
        otaLastActivity = millis();
        // Фрейм: [magic 2B][type 1B][seq 4B][MAC 2B][ciphertext][crc16 2B]
        uint32_t i = seq - otaSeqExp;
        uint32_t off = seq * OTA_RAW_CHUNK_BYTES;
        int ct_len = len - 11;
        if (seq >= otaSeqExp && i < OTA_WINDOW && !(otaWinMask & (1u << i)) && off < otaStreamLen && ct_len > 0 &&
            ct_len % 16 == 0 && sensorChannelIdx >= 0) {
            uint8_t slot = seq % OTA_WINDOW;
            // шифр дополнен до 16 байт — настоящую длину чанка даёт длина потока
            uint32_t need = min((uint32_t)OTA_RAW_CHUNK_BYTES, otaStreamLen - off);
            int n = decryptRaw(channels[sensorChannelIdx].secret, &buf[7], &buf[9], ct_len, otaWin[slot],
                               OTA_RAW_CHUNK_BYTES);
            if (n >= (int)need) {
                otaWinLen[slot] = need;
                otaWinMask |= (1u << i);
            } else {
                Serial.printf("[OTA] raw decrypt FAIL seq=%u\n", seq);
            }
        }
        if (type == RAW_TYPE_DATA_LAST) {
            if (!otaFlushWindow()) {
                otaRawFail("write fail");
                return;
            }
            otaSensorDraw();
            otaSendWack();
        }
        return;
    }

    if (type == RAW_TYPE_POLL) {
        if (!otaGotStart)
            return;
        otaLastActivity = millis();
        if (!otaFlushWindow()) {
            otaRawFail("write fail");
            return;
        }
        otaSendWack();
        return;
    }

    if (type == RAW_TYPE_DONE) {
        if (otaGot != otaTotal || !otaGotStart) {
            Serial.printf("[OTA] raw DONE size mismatch got=%u total=%u\n", otaGot, otaTotal);
            otaRawFail("size mismatch");
            return;
        }
        uint32_t actCrc = ~otaCrcAcc;
        if (actCrc != otaCrcExp) {
            Serial.printf("[OTA] CRC MISMATCH exp=%08X got=%08X\n", otaCrcExp, actCrc);
            otaRawFail("crc mismatch");
            return;
        }
        const char *why = "apply fail";
        if (!otaSinkApply(&why)) {
            otaRawFail(why);
            return;
        }
        otaActive = false;
        otaGotStart = false;
        uint8_t f[16];
        rawTxFrame(f, rawBuildFrame(f, RAW_TYPE_DONE_ACK, 0, NULL, 0));
        if (!otaKindNeedsReboot()) {
            // Пакет уже установлен. Перезагружаться ради него незачем, а связь обрывать
            // тем более: узел остаётся в сети и продолжает слушать эфир.
            Serial.println("[OTA] raw DONE: груз принят и применён");
            otaZFree();
            return;
        }
        Serial.println("[OTA] raw DONE, rebooting...");
        delay(300);
        ESP.restart();
    }
}

void otaSensorDraw() {
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw < OTA_DRAW_MS)
        return;
    lastDraw = millis();
    mcUiOtaSensorProgress(otaGot, otaTotal, (uint32_t)packetCount, lastRSSI, lastSNR);
}

void otaSensorAbort(const char *why) {
    bool wasFast = otaFastMode;
    if (otaFastMode) {
        Serial.printf("[OTA] быстрый канал: принято кадров %u, ошибок приёма %u\n", (unsigned)fastRxFrames,
                      (unsigned)fastRxErrors);
    }
    radioSetNormalConfig();
    otaFastMode = false;
    Serial.printf("[OTA] abort (%s), остаёмся на текущей прошивке\n", why);
    // Пока мы не уходили на быстрый канал, бот слушает штатный и ждёт ответа. Без
    // этого сообщения он увидит лишь таймаут и не покажет настоящую причину.
    if (!wasFast && otaGotStart && sensorChannelIdx >= 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "ota:fail:%s", why);
        sensorSendMsg(msg);
    }
    otaSinkAbort();
    otaZFree();
    otaWinMask = 0;
    otaActive = false;
    otaGotStart = false;
    otaAwaitEndMs = 0;
    otaCrcAcc = 0xFFFFFFFF;
    otaSeqExp = 0;
    mcUiOtaSensorAbort(why, fastRxFrames, fastRxErrors);
}

void otaSensorTick() {
    if (!otaActive)
        return;
    // бот не услышал ackstart и остался на штатном конфиге — возвращаемся, чтобы принять повтор ota:start
    if (otaFastMode && otaGot == 0 && millis() - otaLastActivity > OTA_SENSOR_FIRST_CHUNK_MS) {
        otaSensorAbort("no first chunk");
        return;
    }
    // Весь образ принят — остался только фрейм DONE. Не ждём его весь OTA_SENSOR_STALL_MS:
    // бот повторяет DONE ~18 с и сдаётся, а мы глухи к сети, пока сидим в быстром канале.
    if (otaGot == otaTotal) {
        if (otaAwaitEndMs == 0) {
            // Можно проверить целостность уже сейчас: CRC копится по мере записи образа.
            if (~otaCrcAcc != otaCrcExp) {
                otaSensorAbort("crc mismatch before DONE");
                return;
            }
            otaAwaitEndMs = millis();
        } else if (millis() - otaAwaitEndMs > OTA_SENSOR_END_MS) {
            otaSensorAbort("no final DONE");
        }
        return;
    }
    if (millis() - otaLastActivity > OTA_SENSOR_STALL_MS) {
        otaSensorAbort("stall timeout");
    }
}

void otaSensorHandle() {
    String m = lastMessage;

    if (m == "ota:abort") {
        otaSensorAbort("from bot");
        return;
    }

    if (m.startsWith("ota:start:")) {
        // ota:start:<target>:<вид>:<размер образа>:<crc32hex>:z<размер сжатого потока>
        String rest = m.substring(10);
        int p = rest.indexOf(':');
        if (p <= 0)
            return;
        String target = rest.substring(0, p);
        if (target != cfg.name)
            return;
        // Вид груза. Быстрый режим возит ТОЛЬКО прошивку, и это не упрощение: в нём узел
        // уходит на FSK и глохнет ко всей остальной сети, а кончается сессия
        // перезагрузкой. Для прошивки это оправданная плата, для пакета в десятки
        // килобайт — нет; у пакетов есть медленный режим, где узел остаётся на связи.
        rest = rest.substring(p + 1);
        p = rest.indexOf(':');
        if (p <= 0)
            return;
        OtaKind kind;
        if (!otaKindFromWire(rest.substring(0, p).c_str(), &kind)) {
            Serial.printf("[OTA] отказ: неизвестный вид груза «%s»\n", rest.substring(0, p).c_str());
            sensorSendMsg("ota:fail:bad kind");
            return;
        }
        if (kind != OTA_KIND_FIRMWARE) {
            Serial.println("[OTA] отказ: быстрый режим возит только прошивку, пакет — медленным");
            sensorSendMsg("ota:fail:fast fw only");
            return;
        }
        otaKind = kind;
        // Прошивку принимаем только с канала, ключ которого задан нами, а не выведен из
        // его имени: иначе образ смог бы прислать любой, кто угадал имя канала, — а
        // проверка маркера платы от подделки не спасает, это просто строка в образе.
        if (channelKeyIsOpen(sensorChannelIdx)) {
            Serial.println("[OTA] отказ: у канала сенсоров нет своего ключа (sns_key)");
            sensorSendMsg("ota:fail:no psk");
            return;
        }
        rest = rest.substring(p + 1);
        int p2 = rest.indexOf(':');
        if (p2 <= 0)
            return;
        uint32_t total = strtoul(rest.substring(0, p2).c_str(), NULL, 10);
        String tail = rest.substring(p2 + 1);
        uint32_t crc = (uint32_t)strtoul(tail.c_str(), NULL, 16);
        int zp = tail.indexOf(":z");
        uint32_t zsize = (zp > 0) ? strtoul(tail.c_str() + zp + 2, NULL, 10) : 0;
        if (total == 0 || total > OTA_MAX_FW_BYTES || zsize == 0)
            return;
        // прерываем предыдущую сессию, если вдруг была
        if (otaActive && otaGotStart)
            otaSinkAbort();
        otaTotal = total;
        otaCrcExp = crc;
        otaGot = 0;
        otaCrcAcc = 0xFFFFFFFF;
        otaSeqExp = 0;
        otaAwaitEndMs = 0;
        otaActive = true;
        otaGotStart = true;
        otaLastActivity = millis();
#if HAS_OLED
        screenWake(); // прошивка пришла — зажигаем экран, иначе ход не видно
#endif
        otaZFree();
        otaStreamLen = zsize;
        otaWinMask = 0;
        otaDictOfs = 0;
        otaInfl = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
        otaDict = (uint8_t *)malloc(TINFL_LZ_DICT_SIZE);
        if (!otaInfl || !otaDict) {
            otaSensorAbort("no RAM");
            return;
        }
        tinfl_init(otaInfl);
        if (!otaSinkBegin(total)) {
            otaSensorAbort("begin fail");
            return;
        }
        Serial.printf("[OTA] start %s: %u байт crc=%08X\n", cfg.name.c_str(), total, crc);
        otaSensorDraw();
        // ackstart уходит на штатном конфиге; бот после него ждёт OTA_FAST_SETTLE_MS.
        // Своя отправка, а не sensorSendMsg: у того пауза флуда, а здесь бюджет жёсткий —
        // сенсор ещё не на быстром канале и его сторож уже тикает (см. OTA_ACKSTART_GAP_MS).
        otaSensorAckstart();
        radioSetFastConfig();
        if (!radioFastReadyNow()) {
            otaSensorAbort("радио не переключилось");
            return;
        }
        otaFastMode = true;
        Serial.printf("[OTA] fast config: FSK %d кбит/с\n", (int)OTA_FSK_BR);
        return;
    }
}


// ===== ПОТОК ДЛЯ МЕДЛЕННОГО РЕЖИМА =====
// Медленный режим (ota_slow.cpp) отличается только доставкой: вместо сырых FSK-кадров чанки
// приезжают обычными сообщениями канала. Разворачивание потока, запись во флеш, подсчёт CRC
// и проверка маркера платы у него те же — поэтому здесь не новая механика, а три обёртки
// над той, что уже обслуживает быстрый режим. Дублировать её значило бы чинить распаковку
// дважды.
#if FEATURE_MESH_OTA_RECEIVER

bool otaSlowStreamBegin(uint32_t total, uint32_t crc, OtaKind kind) {
    if (otaActive)
        return false; // быстрая сессия уже идёт
    otaKind = kind;   // до otaSinkBegin: он по виду выбирает, куда писать
    otaTotal = total;
    otaCrcExp = crc;
    otaGot = 0;
    otaCrcAcc = 0xFFFFFFFF;
    otaDictOfs = 0;
    otaZFree();
    otaInfl = (tinfl_decompressor *)malloc(sizeof(tinfl_decompressor));
    otaDict = (uint8_t *)malloc(TINFL_LZ_DICT_SIZE);
    if (!otaInfl || !otaDict) {
        otaZFree();
        return false;
    }
    tinfl_init(otaInfl);
    if (!otaSinkBegin(total)) {
        otaZFree();
        return false;
    }
    // Частоту не трогаем и радио не переключаем: чанки идут обычными сообщениями, и узел
    // всё это время остаётся на связи — в отличие от быстрого режима, где он глух ко всему.
    return true;
}

bool otaSlowStreamFeed(const uint8_t *data, size_t n, bool last) {
    if (!otaInfl || !otaDict)
        return false;
    return otaFeed(data, n, last);
}

// true — груз сошёлся по длине и CRC и применён (прошивка записана в раздел, пакет
// установлен). Перезагружать узел нужно только для прошивки — спрашивайте
// otaSlowStreamNeedsReboot(); false — всё откатывается, раздел остаётся прежним.
bool otaSlowStreamEnd(bool apply) {
    bool ok = false;
    if (apply) {
        if (otaGot != otaTotal) {
            Serial.printf("[SLOW] размер не сошёлся: %u из %u\n", (unsigned)otaGot, (unsigned)otaTotal);
        } else if (~otaCrcAcc != otaCrcExp) {
            Serial.printf("[SLOW] CRC не сошёлся\n");
        } else {
            const char *why = "apply fail";
            if (otaSinkApply(&why)) {
                ok = true;
            } else {
                Serial.printf("[SLOW] не применилось: %s\n", why);
            }
        }
    }
    if (!ok)
        otaSinkAbort();
    otaZFree();
    return ok;
}

bool otaSlowStreamNeedsReboot() {
    return otaKindNeedsReboot();
}

#endif // FEATURE_MESH_OTA_RECEIVER

#endif // FEATURE_MESH_OTA_RECEIVER
