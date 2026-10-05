#include "config.h"
#include "globals.h"
#include "crypto.h"
#include "radio.h"
#include "mesh.h"
#include "ota.h"

// ===== Передача =====
// Выделено из mesh.cpp: объявление себя в сети, сборка группового и личного кадра,
// повторы флуда и сообщения узла (heartbeat, проверка связи, синхронизация времени).

// Пауза перед следующей копией. Свою базу вызывающий код может задать (gapBaseMs),
// а может не задавать — тогда пауза берётся из всего заявленного диапазона
// FLOOD_RETRY_MIN_MS…MAX_MS плюс FLOOD_JITTER_MS разброса сверху, чтобы соседи не
// повторяли копии синхронно.
//
// Раньше FLOOD_RETRY_MAX_MS был объявлен, но не использовался: пауза всегда бралась из
// FLOOD_RETRY_MIN_MS, то есть 1000…1500 мс вместо задуманных 1000…1800, и разброс между
// соседями был вдвое уже, чем считалось. Ноль означает «не задано»: иначе база по
// умолчанию, равная FLOOD_RETRY_MIN_MS, снова прижимала бы диапазон к нижней границе.
unsigned int floodGapMs(unsigned int baseMs) {
    if (baseMs == 0) return random(FLOOD_RETRY_MIN_MS, FLOOD_RETRY_MAX_MS + FLOOD_JITTER_MS);
    if (baseMs < FLOOD_RETRY_MIN_MS) baseMs = FLOOD_RETRY_MIN_MS;
    if (baseMs > FLOOD_RETRY_MAX_MS) baseMs = FLOOD_RETRY_MAX_MS;
    return random(baseMs, baseMs + FLOOD_JITTER_MS);
}

void sendAdvert(uint8_t route_type) {
    uint8_t app[32];
    int applen = 0;
    app[applen++] = 0x80 | 0x01;  // ADV_TYPE_CHAT + имя
    const char* name = cfg.name.c_str();
    int nlen = (int)cfg.name.length();
    if (nlen > 31) nlen = 31;
    memcpy(app + applen, name, nlen);
    applen += nlen;

    uint8_t frame[190];
    int f = 0;
    frame[f++] = (uint8_t)((PAYLOAD_TYPE_ADVERT << 2) | (route_type & 0x03));  // ADVERT | route
    frame[f++] = PATH_LEN_INIT;  // path_len: размер хэша и 0 хопов

    memcpy(frame + f, bot_pub, 32); f += 32;
    uint32_t ts = (uint32_t)time(NULL);
    memcpy(frame + f, &ts, 4); f += 4;

    uint8_t msg[32 + 4 + 32];
    int mlen = 0;
    memcpy(msg + mlen, bot_pub, 32); mlen += 32;
    memcpy(msg + mlen, &ts, 4); mlen += 4;
    memcpy(msg + mlen, app, applen); mlen += applen;

    uint8_t sig[64];
    Ed25519::sign(sig, bot_priv, bot_pub, msg, mlen);
    memcpy(frame + f, sig, 64); f += 64;
    memcpy(frame + f, app, applen); f += applen;

    Serial.printf("\n[TX ADV] route=%u (%dB)\n", route_type, f);
    for (int i = 0; i < f; i++) Serial.printf("%02X", frame[i]);
    Serial.println();
    markOwnFrameSeen(frame, f);   // своё эхо, вернувшееся через ретрансляторов, не переиздавать

    // Объявление уходит FLOOD_REPEATS копиями, как и всякое сообщение. Раньше оно уходило
    // ровно один раз, и это была отдельная беда: адверт несёт публичный ключ, и потерянный
    // адверт — это узел, которому нельзя ответить в личку (findPeerPub вернёт NULL), плюс
    // пропавший из реестра сосед. Узлы объявляются раз в пять минут, так что одна неудачная
    // передача означала минуты недоступности, а не секунды.
    for (int i = 0; i < FLOOD_REPEATS; i++) {
        txFrame(frame, f);
        if (i < FLOOD_REPEATS - 1) delay(floodGapMs(0));
    }
}

int buildGroupEnc(int chIdx, const String& msg, uint8_t* enc) {
    if (chIdx < 0 || chIdx >= numChannels) return 0;
    uint8_t plaintext[GROUP_TEXT_MAX_PLAIN];
    // ts — Unix-время в СЕКУНДАХ (как у adverts и личных сообщений): epoch-ms не
    // помещается в uint32. Пока часы не выставлены — millis.
    uint32_t now = (uint32_t)time(NULL);
    uint32_t ts = (now > 1000000000) ? now : (uint32_t)millis();
    memcpy(plaintext, &ts, 4);                      // timestamp (LE)
    plaintext[4] = 0;                               // TXT_TYPE_PLAIN
    size_t plen = 5;
    // "имя: " собирается в рантайме: имя приходит из настроек, а не из макроса
    char prefix[48];
    int pn = snprintf(prefix, sizeof(prefix), "%s: ", cfg.name.c_str());
    if (pn < 0) pn = 0;
    if (pn > (int)sizeof(prefix) - 1) pn = (int)sizeof(prefix) - 1;
    size_t n = min((size_t)pn, sizeof(plaintext) - plen);
    memcpy(plaintext + plen, prefix, n); plen += n;
    n = min((size_t)msg.length(), sizeof(plaintext) - plen);
    memcpy(plaintext + plen, msg.c_str(), n); plen += n;

    return encryptGroupText(channels[chIdx].secret, enc, plaintext, plen);
}

int buildGroupFrameFlood(int chIdx, const String& msg, uint8_t* frame, int maxlen) {
    uint8_t enc[GROUP_TEXT_MAX_PLAIN + 2];
    int enclen = buildGroupEnc(chIdx, msg, enc);
    if (enclen <= 0 || 3 + enclen > maxlen) return 0;
    frame[0] = 0x15;                    // GRP_TXT | ROUTE_TYPE_FLOOD
    frame[1] = PATH_LEN_INIT;           // размер хэша, 0 хопов (путь достроят ретрансляторы)
    frame[2] = channels[chIdx].hash;
    memcpy(frame + 3, enc, enclen);
    return 3 + enclen;
}

int buildPrivateTextFrame(uint8_t dest_hash, const uint8_t* dest_pub,
                          const String& msg, uint8_t* frame, int maxlen) {
    uint8_t secret[32];
    ed25519_key_exchange(secret, dest_pub, bot_prv64);

    uint8_t data[DM_TEXT_MAX + 8];
    int dlen = 0;
    uint32_t ts = (uint32_t)time(NULL);   // Unix-секунды (epoch-ms не лезет в uint32)
    memcpy(data, &ts, 4); dlen += 4;
    data[dlen++] = 0;                        // attempt = 0
    size_t ml = min((size_t)DM_TEXT_MAX, (size_t)msg.length());
    memcpy(data + dlen, msg.c_str(), ml); dlen += ml;
    data[dlen++] = 0;                        // null terminator

    uint8_t enc[DM_TEXT_MAX + 24];           // MAC 2 + шифр, дополненный до кратного 16
    int enclen = encryptGroupText(secret, enc, data, dlen);   // [MAC 2B][cipher]
    if (enclen <= 0 || 4 + enclen > maxlen) return 0;

    int f = 0;
    frame[f++] = 0x09;                       // TXT_MSG | ROUTE_TYPE_FLOOD
    frame[f++] = PATH_LEN_INIT;              // размер хэша, 0 хопов
    frame[f++] = dest_hash;
    frame[f++] = ownShortHash;
    memcpy(frame + f, enc, enclen); f += enclen;
    return f;
}

int sendFrame(int chIdx, const uint8_t* frame, int f, bool logHex) {
    if (chIdx < 0 || chIdx >= numChannels) return RADIOLIB_ERR_UNKNOWN;
    // hex-лог кадра стоит ~40 мс на UART для 245-байтного кадра — в fast-режиме молчим.
    // Копии флуда побайтово одинаковы, поэтому кадр печатается один раз на сообщение
    // (floodSend вызывает это с logHex только для первой копии): три копии давали три
    // одинаковых дампа, а UART на 115200 не успевает и это само по себе тормозит отправку.
    if (logHex && !otaFastMode) {
        for (int i = 0; i < f; i++) Serial.printf("%02X", frame[i]);
        Serial.println();
    }
    return txFrame((uint8_t*)frame, f);
}

// ===== Очередь копий флуда =====
// Копии шли подряд, с delay() между ними, и это останавливало главный цикл на секунды. На
// коротком сообщении кнопки счёт такой: окно серии 700 мс + три копии по ~0.5 с в эфире +
// две паузы по ~1.25 с = около 4.7 с, и всё это время прошивка не обслуживала ни экран, ни
// консоль настроек, ни сторожа сессий. Снаружи это выглядит зависанием.
//
// Приём тот же, что уже применён к ответу на «cfg get» и к очереди ретранслятора: первая
// копия уходит сразу (сообщение не задерживается), остальные ждут своего срока и уходят из
// meshTxTick() по одной за проход. Срок у каждой свой, с теми же паузами, что были раньше,
// поэтому в эфире ничего не меняется — меняется только то, что цикл при этом жив.
// Слотов восемь, а не четыре: при отправке из приложения в очередь уходят ВСЕ копии
// сообщения (см. floodSendQueued), то есть одно сообщение занимает три слота, и двух
// сообщений подряд уже хватало бы на переполнение.
#ifndef FLOOD_TX_QUEUE_MAX
#define FLOOD_TX_QUEUE_MAX 8
#endif

static struct {
    unsigned long dueMs;        // 0 — слот свободен
    int chIdx;
    int len;
    uint8_t frame[256];
} floodQ[FLOOD_TX_QUEUE_MAX];

static uint32_t floodQueueDrops = 0;

void meshTxDropQueued() {
    for (int i = 0; i < FLOOD_TX_QUEUE_MAX; i++) floodQ[i].dueMs = 0;
}

// Одна копия за проход, самая просроченная. По одной — потому что каждая отправка ждёт
// тишины в канале и держит кадр в эфире: отдать всю очередь за раз значило бы вернуть ту же
// остановку цикла, от которой очередь и заведена (так же устроен meshRelayTick).
void meshTxTick() {
    // На быстром канале копии не нужны и вредны: там FSK и другой кадр.
    if (otaFastMode) { meshTxDropQueued(); return; }
    const unsigned long now = millis();
    int best = -1;
    long bestLate = -1;
    for (int i = 0; i < FLOOD_TX_QUEUE_MAX; i++) {
        if (floodQ[i].dueMs == 0) continue;
        const long late = (long)(now - floodQ[i].dueMs);
        if (late < 0) continue;
        if (late > bestLate) { bestLate = late; best = i; }
    }
    if (best < 0) return;
    const int ch = floodQ[best].chIdx;
    floodQ[best].dueMs = 0;
    if (ch >= 0) sendFrame(ch, floodQ[best].frame, floodQ[best].len, false);
    else         txFrame(floodQ[best].frame, floodQ[best].len);
}

static bool floodQueueCopy(int chIdx, const uint8_t* frame, int f, unsigned long dueMs) {
    for (int i = 0; i < FLOOD_TX_QUEUE_MAX; i++) {
        if (floodQ[i].dueMs != 0) continue;
        floodQ[i].dueMs = dueMs ? dueMs : 1;   // 0 занято признаком «свободен»
        floodQ[i].chIdx = chIdx;
        floodQ[i].len = f;
        memcpy(floodQ[i].frame, frame, f);
        return true;
    }
    return false;
}

// Общая часть обеих отправок. inlineFirst решает, уходит первая копия прямо здесь или тоже
// ждёт тика: разница в том, блокирует ли вызывающий код главный цикл на время передачи.
static void floodSendImpl(int chIdx, const uint8_t* frame, int f, unsigned int gapBaseMs,
                          int repeats, bool inlineFirst) {
    markOwnFrameSeen(frame, f);   // своё эхо, вернувшееся через ретрансляторов, не переиздавать
    // База паузы по умолчанию — время ЭТОГО кадра в эфире, а не общая константа. Так же
    // устроено у оригинального MeshCore: там задержка повтора выводится из
    // getEstAirtimeFor() пакета. Причина простая: на наших настройках кадр 16 Б висит в
    // эфире 259 мс, а 255 Б — 1979 мс, и держать для обоих одну паузу значит либо наложить
    // копии большого кадра друг на друга, либо впустую тормозить короткие сообщения
    // (а короткие — это почти весь обмен: heartbeat, пинги, команды).
    //
    // Ноль по-прежнему означает «решай сам»; floodGapMs зажмёт базу в [FLOOD_RETRY_MIN_MS,
    // FLOOD_RETRY_MAX_MS] и добавит разброс.
    if (gapBaseMs == 0) gapBaseMs = radioAirtimeMs(f);
    unsigned long firstDue = millis();
    if (inlineFirst) {
        // Первая копия уходит здесь же: обычная отправка не должна ждать тика.
        if (chIdx >= 0) sendFrame(chIdx, frame, f, true);
        else            txFrame((uint8_t*)frame, f);
    } else if (!floodQueueCopy(chIdx, frame, f, firstDue)) {
        // Очередь занята — отправляем сразу, иначе сообщение потеряется совсем.
        if (chIdx >= 0) sendFrame(chIdx, frame, f, true);
        else            txFrame((uint8_t*)frame, f);
    }

    // Остальные — в очередь, каждая со своим сроком. Пауза перед следующей копией набирается
    // заново для каждой: одинаковые паузы снова собрали бы копии в один залп, и одна помеха
    // убила бы их все. Разброс поверх gapBaseMs нужен ещё и затем, чтобы два узла, начавшие
    // передачу вместе, не повторяли копии синхронно.
    unsigned long due = firstDue;
    for (int i = 1; i < repeats; i++) {
        due += floodGapMs(gapBaseMs);
        if (floodQueueCopy(chIdx, frame, f, due)) continue;
        // Очередь занята. Доставка важнее отзывчивости: досылаем остаток по-старому, с
        // блокирующей паузой, и считаем это — если такое случается часто, слотов мало.
        floodQueueDrops++;
        Serial.printf("[TX] очередь копий занята — досылаю %d-ю блокирующе (всего %lu)\n",
                      i + 1, (unsigned long)floodQueueDrops);
        delay(floodGapMs(gapBaseMs));
        if (chIdx >= 0) sendFrame(chIdx, frame, f, false);
        else            txFrame((uint8_t*)frame, f);
    }
}

void sensorSendMsg(const char* msg, unsigned int gapBaseMs, int repeats) {
    if (sensorChannelIdx < 0) {
        Serial.printf("[SNS] sensor channel not configured, cannot send \"%s\"\n", msg);
        return;
    }
    uint8_t frame[256];
    int f = buildGroupFrameFlood(sensorChannelIdx, msg, frame, sizeof(frame));
    if (f <= 0) return;
    floodSend(-1, frame, f, gapBaseMs, repeats);
    Serial.printf("[SNS] sent \"%s\" to sensor channel\n", msg);
    #ifdef COMPANION_NODE
    // Собственные передачи в приложение иначе не попадают: в очередь кладётся только
    // принятое из эфира, а свой же флуд отбрасывается как эхо. Кладём прямо здесь, с тем
    // же префиксом имени, с каким сообщение ушло в эфир.
    companionOnChannelText(sensorChannelIdx, cfg.name + ": " + msg, 0.0f, PATH_LEN_INIT, false);
    #endif
    #ifdef SENSOR_NODE
    sensorLastSent = msg;
    sensorLastSentMs = millis();
    #endif
}

// "time:<epoch>:<версия бота>" — сенсор выставляет часы и сверяет свою версию с ботом
#ifdef SENSOR_NODE
// hello:<версия>:<заряд %>:<напряжение>:<окружение> — бот публикует эти поля в MQTT.
// Без измерения батареи вместо значений идёт "-", чтобы позиции полей не съезжали.
void sensorSendHello() {
    // Четвёртое поле — окружение сборки. Кода платы в heartbeat больше нет: окружение
    // и так называет плату («heltec_v4_3_sensors»), причём точнее — сенсор и компаньон
    // живут на одной h43, а образы у них разные. Два поля об одном и том же значат, что
    // однажды они разойдутся.
    char msg[96];
    #if HAS_BATTERY
    if (mcBatteryPresent()) {
        char bv[12];
        snprintf(msg, sizeof(msg), "%s:%s:%d:%s:%s", SENSOR_MSG_HELLO, FW_VERSION,
                 mcBatteryPercent(), fmtFix(mcBatteryVoltage(), 2, bv, sizeof(bv)), FW_ENV);
    } else {
        snprintf(msg, sizeof(msg), "%s:%s:-:-:%s", SENSOR_MSG_HELLO, FW_VERSION, FW_ENV);
    }
    #else
    snprintf(msg, sizeof(msg), "%s:%s:-:-:%s", SENSOR_MSG_HELLO, FW_VERSION, FW_ENV);
    #endif
    // Координаты — пятым и шестым полями, и ТОЛЬКО когда решение есть. Без фикса поля не
    // дописываются вовсе: у координатора в MQTT остаётся последняя известная точка (топики
    // retained), а узлы без приёмника шлют ровно ту же строку, что и раньше. Откуда взять
    // координаты, ядро не знает — оно спрашивает точку расширения платы (mcBoardPosition).
    int32_t lat = 0, lon = 0;
    if (mcBoardPosition(&lat, &lon)) {
        char la[16], lo[16];
        size_t n = strlen(msg);
        snprintf(msg + n, sizeof(msg) - n, ":%s:%s",
                 fmtUdeg(lat, la, sizeof(la)), fmtUdeg(lon, lo, sizeof(lo)));
    }
    sensorSendMsg(msg);
}

#endif

#ifdef SENSOR_NODE
// Эхо-запрос: одиночная посылка, чтобы измерять время одного обмена, а не повторов
void sensorSendMsgUnique(const char* prefix) {
    // Сообщение, которое может дословно повториться (нажатие кнопки), уходит с номером
    // отправки. Без номера два одинаковых нажатия в одну секунду дают ПОБАЙТОВО одинаковый
    // кадр: время в кадре хранится с точностью до секунды, а шифрование — ECB без nonce,
    // поэтому совпадает и шифротекст, а с ним и хэш дедупликации. Второе нажатие
    // отбрасывалось всей сетью как дубликат, и до Home Assistant доходило одно.
    //
    // Номер существует только в эфире: получатель убирает его по тому же правилу, что и
    // префикс имени, и в MQTT отдаётся прежнее «button» — иначе сломались бы все
    // автоматизации, подписанные на это значение.
    static uint16_t seq = 0;
    seq++;
    if (seq == 0) seq = 1;   // 0 — «номера нет»
    char msg[64];
    snprintf(msg, sizeof(msg), "%s:%u", prefix, (unsigned)seq);
    sensorSendMsg(msg);
}

void sensorPingSend() {
    if (sensorChannelIdx < 0) return;
    // Номер запроса — счётчик, а не младшие байты millis(): те заворачиваются каждые ~65 с,
    // и «свежий» милисекундный номер совпадёт со старым ответом, застрявшим в эфире.
    static uint16_t pingSeq = 1;
    pingId = (pingSeq == 0xFFFF) ? 1 : pingSeq + 1;
    pingSeq = pingId;
    pingFailed = false;
    pingShowUntil = 0;
    char msg[24];
    snprintf(msg, sizeof(msg), "%s%u", SENSOR_MSG_PING, (unsigned)pingId);
    pingSentMs = millis();
    if (pingStatSent < 0xFFFF) pingStatSent++;
    sensorSendMsg(msg, 0, 1);
}

void pingModeToggle() {
    if (pingModeOn) {
        pingModeOn = false;
        // Последняя выборка остаётся на экране: иначе выключение стирало бы то, ради чего
        // режим и включали.
        pingShowUntil = millis() + PING_SHOW_MS;
        Serial.printf("[PING] режим выключен: ушло %u, потеряно %u\n",
                      (unsigned)pingStatSent, (unsigned)pingStatLost);
        return;
    }
    pingModeOn = true;
    pingModeStartMs = millis();
    pingStatSent = pingStatRecv = pingStatLost = 0;
    pingRttMin = pingRttMax = pingRttSum = 0;
    pingHopsMin = pingHopsMax = 0;
    pingHopsSum = 0;
    pingFailed = false;
    pingShowUntil = 0;
    pingModeNextMs = millis();   // первый запрос — сразу
    Serial.println("[PING] режим включён");
}

void sensorPingTick() {
    // Ждали ответ и не дождались: в режиме это просто потеря в выборке, вне режима — всё,
    // что мы узнали, и его показывает экран.
    if (pingSentMs != 0 && millis() - pingSentMs >= PING_TIMEOUT_MS) {
        pingSentMs = 0;
        pingFailed = true;
        // Вот теперь запрос действительно потерян: ждать его больше не будем, а следующий
        // уйдёт по расписанию. До этого момента он просто в пути.
        if (pingStatLost < 0xFFFF) pingStatLost++;
        if (!pingModeOn) pingShowUntil = millis() + PING_SHOW_MS;
        Serial.println("[PING] ответа нет");
    }
    if (!pingModeOn) return;
    // «Включили и ушли»: режим сам выключится, не посадив аккумулятор.
    if (millis() - pingModeStartMs >= PING_MODE_MAX_MS) {
        Serial.println("[PING] предел режима — выключаю");
        pingModeToggle();
        return;
    }
    // Во время прошивки по радио эфир занят чанками: запрос туда не полезет, расписание
    // сдвинется само.
    if (otaActive) { pingModeNextMs = millis() + PING_MODE_INTERVAL_MS; return; }
    if ((long)(millis() - pingModeNextMs) < 0) return;
    pingModeNextMs = millis() + PING_MODE_INTERVAL_MS;
    #if HAS_OLED
    screenWake();   // экран не должен уснуть посреди проверки — на него и смотрят
    #endif
    sensorPingSend();
}
#endif

void sendSensorTimeSync() {
    char msg[48];
    snprintf(msg, sizeof(msg), "time:%llu:%s", (unsigned long long)time(NULL), FW_VERSION);
    sensorSendMsg(msg);
}

String channelListStr() {
    String s = "";
    for (int i = 0; i < numChannels; i++) {
        if (i > 0) s += " + ";
        s += channels[i].name;
    }
    return s;
}

void floodSend(int chIdx, const uint8_t* frame, int f, unsigned int gapBaseMs, int repeats) {
    floodSendImpl(chIdx, frame, f, gapBaseMs, repeats, true);
}

// Отправка, которая НЕ выходит в эфир из вызывающего кода: в очередь уходят все копии,
// включая первую, и функция возвращается сразу.
//
// Нужна там, где вызывающий обязан ответить, а не ждать: приложение-компаньон после команды
// «отправить сообщение» ждёт подтверждения по BLE, и если мы сначала передаём кадр, а потом
// отвечаем, подтверждение опаздывает на ожидание тишины в канале плюс время кадра в эфире —
// до двух секунд. Снаружи это и есть «приложение притормаживает». У оригинала такого нет
// потому, что там передача асинхронная целиком: пакет кладётся в очередь, ответ уходит
// немедленно, а эфиром занимается диспетчер.
//
// Первая копия уйдёт с ближайшего meshTxTick(), то есть через один проход цикла.
void floodSendQueued(int chIdx, const uint8_t* frame, int f, unsigned int gapBaseMs,
                     int repeats) {
    floodSendImpl(chIdx, frame, f, gapBaseMs, repeats, false);
}
