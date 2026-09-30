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

// Какой чанк окна уходит следующим. На уровне файла, а не статической внутри otaSlowTick:
// та переживала сессию, и сброс `if (sent < otaSlowSeq || sent >= inWindow)` вторую сессию
// не спасал — при otaSlowSeq = 0 и оставшемся от прерванной сессии значении 1…7 оба условия
// ложны, и окно начинало уходить не с нулевого чанка. Узел ждал нулевой, отвечал «жду 0»,
// сессия не двигалась и умирала по числу повторов. Сбрасывается в otaSlowStart.
static uint32_t otaSlowSent = 0;

// Отзывался ли узел в этой сессии хоть раз — любым подтверждением. Нужен ровно для одного
// решения: повторять ли вместе с окном сам старт сессии. Пока узел не отозвался, самое
// вероятное объяснение — он не услышал ota:slow: и потому не в сессии вовсе: чанки для него
// чужие сообщения, он их молча выбрасывает, а мы двадцать раз повторяем окно в пустоту.
// Считать по otaSlowAcked нельзя: узел, ответивший «жду 0», оставляет его нулём.
static bool otaSlowHeard = false;

// Версия образа цели ДО сессии и версия, услышанная от неё ВО ВРЕМЯ сессии. Разошлись —
// значит узел перезагрузился в новую прошивку, то есть образ применён, даже если ota:sdone
// до нас не дошёл. Обе запоминаются, а вывод делается позже и в одном месте: heartbeat
// мог прийти ДО того, как докатилось последнее подтверждение, и версия, услышанная в
// неподходящий момент, не должна теряться.
//
// otaSlowTargetVer пуст — узла мы до сессии ни разу не услышали, сравнивать не с чем, и
// запасного пути тогда нет: ждём ota:sdone как обычно. Это осознанный отказ от догадок,
// а не недосмотр: объявлять прошивку успешной по любому heartbeat в канале нельзя.
static String otaSlowTargetVer;
static String otaSlowSeenVer;

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

// Объявление сессии узлу. Вынесено отдельно, потому что зовётся дважды: на старте и на
// повторе окна, пока узел не отозвался. Одиночной посылкой, как чанки и подтверждения:
// старт — тоже рукопожатие, узел отвечает через OTA_SLOW_ACK_DELAY_MS, то есть пока флуд
// слал бы вторую и третью копию (1–2.3 с на копию, до 4.6 с на весь флуд), его
// подтверждение падало бы в занятый эфир. Маятник «в канале говорит только один» ломался
// ровно на старте сессии.
static void otaSlowSendStart() {
    char msg[96];
    snprintf(msg, sizeof(msg), "%s%s:%u:%08X:%u", OTA_SLOW_MSG_START, otaSlowTarget.c_str(),
             (unsigned)otaSlowTotal, (unsigned)otaSlowCrc, (unsigned)otaSlowChunks);
    sensorSendMsg(msg, 0, 1);
    slog("[SLOW] старт -> '%s': %u байт, %u чанков по %d\n", otaSlowTarget.c_str(),
         (unsigned)otaSlowTotal, (unsigned)otaSlowChunks, (int)OTA_SLOW_CHUNK_BYTES);
}

bool otaSlowStart(const String& target) {
    if (otaAnySessionActive()) return false;   // и быстрая, и уже идущая медленная
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
    // Узлу объявляется размер РАСПАКОВАННОГО образа (otaImgSize), а не длина сжатого потока
    // (otaFwSize). Здесь стоял otaFwSize, и медленная прошивка не могла завершиться ни при
    // каких условиях: приёмник открывал раздел под сжатый размер, otaWriteImage обрезал по
    // нему распакованный поток, длина сходилась ровно, а CRC32 считается по полному образу и
    // не совпадал никогда. Сколько байт уедет в эфир, узлу говорит otaSlowChunks ниже —
    // размер в этом поле нужен ему для Update.begin() и для проверки того, что он собрал.
    otaSlowTotal   = otaImgSize;
    otaSlowCrc     = otaFwCrc;
    otaSlowChunks  = (zsize + OTA_SLOW_CHUNK_BYTES - 1) / OTA_SLOW_CHUNK_BYTES;
    otaSlowSeq     = 0;
    otaSlowSent    = 0;
    otaSlowHeard   = false;
    otaSlowTargetVer = sensorVersionOf(target);
    otaSlowSeenVer   = "";
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

    otaSlowSendStart();
    return true;
}

// Подтверждение от узла. Слышат его и отправитель, и координатор — здесь его разбирает
// отправитель, чтобы двинуть окно.
void otaSlowOnAck(uint32_t next) {
    if (!otaSlowOn) return;
    // Узел отозвался — значит он в сессии и старт повторять больше незачем. Отмечаем ДО
    // разбора номера: важен сам факт ответа, а не то, двинулось ли окно.
    otaSlowHeard = true;
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

// Heartbeat из сенсорного канала. Пока идёт сессия, запоминаем версию ЦЕЛИ: после её
// перезагрузки в новую прошивку heartbeat придёт уже с новой версией, и это всё, что
// останется, если ota:sdone потеряется. Вывод делает otaSlowAppliedByHello() — здесь
// только запоминание, чтобы версия, услышанная до последнего подтверждения, не пропала.
//
// От чужих отличаемся именем отправителя, пустую версию не берём: узел постарше шлёт в
// heartbeat просто «hello», и это ничего не говорит о том, перезагружался ли он.
void otaSlowOnHello() {
    if (!otaSlowOn) return;
    if (lastSender != otaSlowTarget) return;
    if (!lastHello.isHello || lastHello.ver.length() == 0) return;
    otaSlowSeenVer = lastHello.ver;
}

// Образ применён, а подтверждение не дошло. Решается здесь и только здесь, потому что
// здесь известно главное условие: весь образ уже сдан и подтверждён узлом.
//
// Как это выглядит в жизни. Узел принял последний чанк, проверил образ, отправил ota:sdone
// и перезагрузился в новую прошивку. Если ota:sdone потерялся — а он идёт через флуд в
// момент, когда канал busiest за всю сессию, — ведущий двадцать повторов ждал подтверждения
// и заканчивал самым бесполезным из возможных сообщений: «узел не подтверждает». То есть
// успешная прошивка докладывалась как провал, и следующая попытка шла заново на уже
// прошитый узел — а узел после первой попытки ещё и не в сессии, потому что он перезагрузился.
static bool otaSlowAppliedByHello() {
    if (otaSlowTargetVer.length() == 0) return false;   // до сессии версии не знали
    if (otaSlowSeenVer.length() == 0) return false;      // heartbeat от цели ещё не было
    return otaSlowSeenVer != otaSlowTargetVer;
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
        // why на успехе — не причина отказа, а чем итог подтверждён. У сессии, завершившейся
        // по потерянному ota:sdone, подтверждение другое, и по журналу это обязано читаться:
        // иначе «ГОТОВО» рядом с «узел не подтверждает» выглядит противоречием.
        if (why && *why) slog("[SLOW] чем подтверждено: %s\n", why);
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
        // Узел не отозвался ни разу — возможно, он просто не услышал объявление сессии.
        // Тогда для него наши чанки не чанки, а чужие сообщения, и он выбрасывает их молча:
        // окно можно повторять сколько угодно, ничего не изменится. Объявляем сессию заново.
        // Повтор безопасен и позже: узел, который уже принимает ЭТОТ образ, от повторного
        // объявления не начинает заново — он отвечает, где остановился (см. otaSlowRxStart).
        if (!otaSlowHeard) otaSlowSendStart();
        return;
    }

    if ((long)(millis() - otaSlowNextMs) < 0) return;

    // Всё отправлено и подтверждено — ждём, пока узел применит образ и скажет об этом.
    if (otaSlowSeq >= otaSlowChunks) {
        // Подтверждение могло потеряться, а узел — уже перезагрузиться в новую прошивку и
        // отметиться. Тогда образ применён, и ждать больше нечего: версию цели мы слышали
        // и запомнили, otaSlowAppliedByHello() это и проверяет. Проверка стоит здесь, а не
        // в разборе heartbeat, потому что «весь образ сдан» известно только тут.
        if (otaSlowAppliedByHello()) {
            otaSlowDone(true, "подтверждение потерялось, образ применён (услышана новая "
                              "версия цели)");
            return;
        }
        otaSlowAckMs = millis() + OTA_SLOW_ACK_TIMEOUT_MS;
        return;
    }

    // Окно: сколько чанков ушло с последнего подтверждения.
    const uint32_t inWindow = otaSlowSeq + OTA_SLOW_WINDOW;
    if (otaSlowSent < otaSlowSeq || otaSlowSent >= inWindow) otaSlowSent = otaSlowSeq;

    uint8_t raw[OTA_SLOW_CHUNK_BYTES];
    const int n = otaSlowReadChunk(otaSlowSent, raw);
    if (n <= 0) { otaSlowAbort("образ не читается"); return; }

    // base64: hex удвоил бы объём, а образ и без того едет часами.
    unsigned char b64[OTA_SLOW_CHUNK_BYTES * 2];
    size_t b64len = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64), &b64len, raw, (size_t)n) != 0) {
        otaSlowAbort("чанк не кодируется");
        return;
    }
    char msg[OTA_SLOW_CHUNK_BYTES * 2 + 24];
    snprintf(msg, sizeof(msg), "%s%u:%.*s", OTA_SLOW_MSG_DATA, (unsigned)otaSlowSent,
             (int)b64len, (const char*)b64);
    // Одной посылкой, без повторов: окно и так повторяется целиком, а лишняя копия каждого
    // чанка удвоила бы и без того долгую сессию.
    sensorSendMsg(msg, 0, 1);

    otaSlowSent++;
    otaSlowDraw();
    if (otaSlowSent >= inWindow || otaSlowSent >= otaSlowChunks) {
        // Окно кончилось — ждём подтверждения.
        otaSlowAckMs = millis() + OTA_SLOW_ACK_TIMEOUT_MS;
    } else {
        otaSlowNextMs = millis() + OTA_SLOW_GAP_MS;
    }
}

#endif // FEATURE_MESH_OTA_SENDER

// Какой чанк ждём — приёмная часть, но живёт здесь, а не внутри блока приёмника: счётчику
// нужно быть видимым обеим ролям (см. otaSlowRxChunksGot ниже).
uint32_t slowRxExpect = 0;

// «Получено 812 из 2400» для экрана. Знаменатель берётся здесь, а не на экране, потому что
// единица приёма медленной сессии — чанк по OTA_SLOW_CHUNK_BYTES, и знает её только ядро;
// прошивка из otaGot/otaTotal может вывести только килобайты, а это другой вопрос.
//
// Именно чанки, а не байты и не кадры: `otaGot` — это записанные байты, `pkts` в хуке — сырые
// кадры эфира вместе с повторами, и ни то, ни другое не показывает ход сессии. Повторы в
// счётчик не попадают, потому что чанк с неверным номером отбрасывается до записи.
//
// Объявлено и определено БЕЗ условий — как общий предикат занятости (см. ota.h). Экранный
// код есть у обеих ролей: переопределение `mcUiOtaSensorProgress()` в прошивке не обёрнуто
// в `#if`, поэтому координатор тоже зовёт эти функции из своего кода. У координатора, который
// образ только раздаёт, оба значения нулевые — и это честно: принял он ноль чанков.
uint32_t otaSlowRxChunksGot() { return slowRxExpect; }
uint32_t otaSlowRxChunksTotal() { return otaSlowChunks; }

// ===== СТОРОНА УЗЛА =====
// Принимает чанки из канала, разворачивает поток тем же распаковщиком, что и быстрый режим
// (otaFeedStream), и подтверждает кумулятивно. Приём не требует переключения радио: это
// обычные сообщения, поэтому узел всё время остаётся на связи — в отличие от быстрого
// режима, где он сорок секунд глух ко всему, кроме чанков.
#if FEATURE_MESH_OTA_RECEIVER

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

    const uint32_t total  = (uint32_t)strtoul(args.substring(p1 + 1, p2).c_str(), NULL, 10);
    const uint32_t crc    = (uint32_t)strtoul(args.substring(p2 + 1, p3).c_str(), NULL, 16);
    const uint32_t chunks = (uint32_t)strtoul(args.substring(p3 + 1).c_str(), NULL, 10);
    if (total == 0 || chunks == 0) return;
    // Размер приходит из эфира. Тот же предел, что у быстрого режима: с заведомо
    // невозможным числом Update.begin() всё равно откажет, но по журналу было бы не понять,
    // отказала память или образ объявлен чужой.
    if (total > OTA_MAX_FW_BYTES) {
        Serial.printf("[SLOW] отказ: объявлено %u байт, предел %u\n",
                      (unsigned)total, (unsigned)OTA_MAX_FW_BYTES);
        return;
    }

    // Объявление ТОГО ЖЕ образа во время приёма — это не «начать заново», а «ведущий нас не
    // слышит». Он повторяет старт, пока не получит от нас ни одного подтверждения (см.
    // otaSlowSendStart), и начинать поток с нуля тут означало бы выбросить часы уже
    // принятого. Отвечаем, где остановились, и продолжаем с того же места.
    //
    // Отсюда же берётся продолжение сессии после перезагрузки ВЕДУЩЕГО: он объявляет тот же
    // образ заново, мы называем свой номер чанка, и окно прыгает туда, а не в ноль.
    if (slowRxOn) {
        if (total == otaSlowTotal && crc == otaSlowCrc && chunks == otaSlowChunks) {
            slowRxLastMs = millis();
            slowRxAckLater();
            Serial.printf("[SLOW] повтор объявления: продолжаем с чанка %u\n",
                          (unsigned)slowRxExpect);
            return;
        }
        // Образ другой — старую сессию закрываем молча. Молча потому, что ota:sfail сейчас
        // сказал бы ведущему, что провалилась ЭТА сессия, а она только что началась.
        Serial.println("[SLOW] объявлен другой образ — прежний приём закрыт");
        otaSlowStreamEnd(false);
        slowRxOn = false;
        otaSlowOn = false;
    }

    otaSlowTotal  = total;
    otaSlowCrc    = crc;
    otaSlowChunks = chunks;

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
    // Записать его некуда — поток разворачивается строго по порядку, — поэтому напоминаем
    // отправителю, с чего продолжать.
    //
    // Напоминаем в ОБОИХ случаях, и это не симметрия ради красоты. Раньше на повтор уже
    // записанного (seq < ожидаемого) узел молчал, и одно потерянное подтверждение вешало
    // сессию намертво: ведущий переотправлял окно, которое узел давно принял, узел на эти
    // чанки не отвечал, ведущий так и не узнавал, где узел на самом деле, — и сессия умирала
    // по числу повторов. Повтор чанка и есть сигнал «он не знает, где я»: ровно так же
    // переподтверждает дубликат TCP.
    //
    // Лавины ответов не будет: slowRxAckLater заявку вперёд не двигает, поэтому целое
    // переотправленное окно даёт одно подтверждение, а не восемь.
    if (seq != slowRxExpect) {
        slowRxAckLater();
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
        // Сначала ПРОВЕРЯЕМ образ, и только потом подтверждаем и гасим флаги. Порядок был
        // обратный, и это стоило медленному режиму всякой диагностики: подтверждение
        // последнего чанка уходило до проверки, флаг slowRxOn гасился до вызова
        // otaSlowRxAbort — а тот на первой строке делает `if (!slowRxOn) return;`. В итоге
        // при несошедшемся образе в эфир не уходило ни ota:sfail, ни что-либо ещё, а ведущий,
        // уже считавший образ доставленным, двадцать повторов ждал ota:sdone и заканчивал
        // самым бесполезным из возможных сообщений — «узел не подтверждает».
        const bool ok = otaSlowStreamEnd(true);
        if (!ok) {
            // slowRxOn ещё поднят, поэтому отказ дойдёт до ведущего и до экрана.
            otaSlowRxAbort("образ не сошёлся");
            return;
        }
        // Подтверждаем только проверенный образ: «принял всё» на несошедшемся образе — это
        // ложь в эфир, по которой ведущий заканчивает сессию довольным.
        slowRxAckNow();       // последнее подтверждение шлём без паузы: ждать уже нечего
        slowRxOn = false;
        otaSlowOn = false;
        char msg[64];
        snprintf(msg, sizeof(msg), "%s%s", OTA_SLOW_MSG_DONE, cfg.name.c_str());
        sensorSendMsg(msg);
        Serial.printf("[SLOW] образ принят и проверен — применяю\n");
        delay(400);           // дать сообщению уйти в эфир до перезагрузки
        ESP.restart();
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
