#include "config.h"
#include "globals.h"
#include "crypto.h"
#include "radio.h"
#include "mesh.h"
#include "ota.h"

// ===== Приём из эфира =====
// Выделено из главного цикла: опрос радио, разбор кадров и поддержание приёмника.
// Здесь же сброс АРУ по расписанию и разбор сырых кадров быстрого канала во время
// прошивки по радио — всё, что связано с приёмом, собрано в одном месте.

// ===== Приём: прерывание вместо опроса =====
//
// Прежде каждый проход цикла читал у радио регистр прерываний по SPI — 360 раз за пять
// секунд по 0.6 мс, то есть 207 мс, и 359 раз из 360 ответом было «пакета нет». У платы
// вывод DIO1 заведён на пин (Module(..., LORA_DIO1, ...)), и радио умеет сказать о приёме
// само.
//
// Но прерывание здесь — только ПОДСКАЗКА, а не единственный источник правды, и это
// осознанно. Пропущенное прерывание (сброс АРУ, перенастройка после передачи, гонка при
// очистке флага) означало бы глухоту узла до перезагрузки — цена ошибки несопоставима с
// экономией. Поэтому регистр всё равно читается, просто редко: раз в RX_POLL_MS. В худшем
// случае пакет заметят на пятьдесят миллисекунд позже, чем могли бы, а не никогда.
#define RX_POLL_MS 50

static volatile bool dio1Fired = false;

static void IRAM_ATTR radioDio1Isr() { dio1Fired = true; }

void radioRxAttachIsr() {
    radio.setDio1Action(radioDio1Isr);
}

void radioRxTick() {
    // Замер по частям. Профиль на T-Deck показывает в слоте приёма проходы по 1000-2001 мс,
    // причём РОВНЫЕ — и только когда по Wi-Fi идёт закачка. Ни одной передачи в это время
    // нет, значит дело не в эфире. Гадать дороже, чем померить: лог включается только на
    // медленных тиках и ничего не стоит на обычных.
    const bool rxDbg = mcDebugLog();
    const uint32_t rxT0 = rxDbg ? millis() : 0;
    uint32_t rxTFlags = rxT0, rxTRead = rxT0;
    bool rxDidRead = false;
    // ===== ПРИЁМ =====
    if (isListening) {
        // Периодический сброс AGC, если не идёт приём пакета прямо сейчас.
        // Не сбрасываем, пока стоит RX_DONE (иначе потеряем пакет).
        // Спрашиваем радио, только если оно позвало или если пришёл срок редкого опроса.
        static uint32_t lastPollMs = 0;
        const bool hint = dio1Fired;
        if (hint) dio1Fired = false;
        const bool duePoll = (uint32_t)(millis() - lastPollMs) >= RX_POLL_MS;
        bool rxPending = false;
        if (hint || duePoll) {
            lastPollMs = millis();
            rxPending = (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) != 0;
        }
        if (!rxPending && (millis() - lastReArmMs > RADIO_REARM_INTERVAL_MS)) {
            rearmRadioAGC();
            rxPending = (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) != 0;
        }

        if (rxDbg) rxTFlags = millis();
        // читать только если радио действительно получило пакет (RX_DONE)
        if (rxPending) {
            rxDidRead = true;
            uint8_t buffer[256];
            int state = radio.readData(buffer, sizeof(buffer));
            if (rxDbg) rxTRead = millis();
            if (state == RADIOLIB_ERR_NONE) {
                int pktLen = radio.getPacketLength();
                float rssi = radio.getRSSI();
                float snr = radio.getSNR();
                // Быстрый канал закрыт целиком, и это внешняя развилка, а не порядок условий.
                // Раньше она была плоской — `if (pktLen >= 9 && otaFastMode && магия) … else …`,
                // и кадр, принятый В БЫСТРОМ РЕЖИМЕ, но короче девяти байт или без магии (битый
                // приём на FSK), уходил в else, то есть в meshcore. Там его метил общий дедуп
                // (кадр не meshcore-овский, и следующий настоящий кадр с тем же хэшем счёлся бы
                // дубликатом) и звал radio.startReceive() поверх ещё не ушедшего в эфир WACK —
                // ровно та поломка, из-за которой прошивка по быстрому каналу вставала.
                if (otaFastMode) {
                    // 9 байт — минимальный сырой кадр (заголовок 7 + crc16). Меньше — читать
                    // buffer[1] значило бы смотреть в мусор от предыдущего пакета.
                    if (pktLen >= 9 && buffer[0] == RAW_MAGIC0 && buffer[1] == RAW_MAGIC1) {
                        // mesh OTA: сырые кадры вне meshcore
                        fastRxFrames++;
                        otaRawDidTx = false;
                        otaHandleRawFrame(buffer, pktLen);
                        lastReArmMs = millis();
                        if (!otaRawDidTx) radio.startReceive();
                    } else {
                        // Не сырой кадр на быстром канале — мусор: своих meshcore-кадров здесь
                        // быть не может, а чужие нам не адресованы. Выбрасываем, не трогая ни
                        // дедуп, ни разбор. Переподписаться нужно: мы ничего не передавали, и без
                        // startReceive приёмник остался бы глухим до конца сессии.
                        fastRxErrors++;
                        radio.startReceive();
                    }
                } else {
                    // Сырой кадр — приложению, ДО дедупа и разбора. Так в оригинале
                    // (Dispatcher::checkRecv -> logRxRaw): приложение видит собственный
                    // пакет, переизданный ретранслятором, и ставит в переписке отметку
                    // «принято ретранслятором». После дедупа этого события не остаётся.
                    if (pktLen > 0) {
                        mcOnRawRx(buffer, pktLen, snr, rssi);
                        rxAirtimeMs += (uint32_t)radioAirtimeMs(pktLen);
                    }
                    // Лог каждого кадра стоит миллисекунды UART, поэтому в быстром режиме его нет
                    // вовсе — а здесь мы уже заведомо не в нём.
                    if (pktLen > 0) {
                        char sr[12], ss[12];
                        Serial.printf("\n[RX] len=%d RSSI=%s SNR=%s ", pktLen,
                                      fmtFix(rssi, 1, sr, sizeof(sr)), fmtFix(snr, 1, ss, sizeof(ss)));
                        for (int i = 0; i < min(pktLen, 24); i++) Serial.printf("%02X", buffer[i]);
                        Serial.println();
                    }
                        // Вызов остаётся без #if: при выключенном FEATURE_RELAY (это значение по
                        // умолчанию — ретранслятором не должен быть никто) maybeQueueRelay
                        // пустышка, и условие живёт в одном месте, в mesh_relay.cpp. Всё
                        // объяснение ниже относится к сборке, где признак включён.
                        //
                        // Ретрансляция — ДО дедупа, и это не перестановка для красоты. maybeQueueRelay
                        // держит собственный кэш и сам разбирается, переиздавался ли кадр; общий
                        // дедуп для этой роли не годился. Он помечает кадр навсегда, и если очередь
                        // переизданий в этот момент была полна, кадр выпадал из неё вместе со
                        // всеми копиями: вторая копия отправителя приходила, узнавалась как
                        // дубликат и больше никуда не переиздавалась. Теперь зовём ретрансляцию
                        // для каждой принятой копии — если очередь уже разгрузилась, кадр в неё
                        // попадёт.
                        maybeQueueRelay(buffer, pktLen);
                        // Дедуп и разбор — ВНУТРИ этой же ветки, и это не вопрос вкуса. Сырой
                        // кадр быстрого OTA не должен попадать сюда: checkAndMarkSeen пометил бы
                        // его в общем кольцевом буфере (он не meshcore-кадр, и следующий
                        // настоящий кадр с тем же хэшем счёлся бы дубликатом), а разбор ниже
                        // вызвал бы radio.startReceive() поверх ещё не ушедшего в эфир WACK —
                        // приёмник переподписался бы, не дождавшись конца собственной передачи.
                        // Раньше это было завязано как `else if`, то есть обе ветви были
                        // взаимоисключающими; при переносе ретрансляции выше дедупа цепочка
                        // осталась снаружи else и стала общей для обоих режимов.
                        if (checkAndMarkSeen(buffer, pktLen)) {
                            duplicateCount++;
                            Serial.printf("[DUP] skipped (total dups=%lu)\n", duplicateCount);
                        } else {
                            // Свежий чужой кадр.
                            // «Вторые уши»: свежий кадр можно переслать координатору по сети.
                            // Хук срабатывает только для радио (forwarded-кадры уже прошли через
                            // него на другом конце): повторять по WiFi то, что и так пришло по WiFi,
                            // — замкнутый круг. Дети-подделки вроде отсюда не нужны.
                            mcOnFreshFrame(buffer, pktLen, rssi, snr);

                            MeshRxMeta meta;
                            meta.origin = MESH_RX_RADIO;
                            meta.rssi = rssi;
                            meta.snr = snr;
                            bool parsed = parseMeshCorePacket(buffer, pktLen, meta);

                            // hex-экран только для GRP_TXT, который не расшифровался
                            // (рекламные/служебные пакеты экран не трогаем) — рисует прошивка.
                            #ifndef SENSOR_NODE
                            if (pktLen > 0 && !parsed && ((buffer[0] >> 2) & 0x0F) == 0x05) {
                                mcUiHexScreen(pktLen, rssi, snr, buffer);
                                lastRxDisplay = millis();
                            }
                            #endif

                            // не перезатираем экран 5 сек после сообщения
                            if (parsed) {
                                lastRxDisplay = millis();
                                #if FEATURE_MQTT
                                publishMessage();
                                #endif
                            }
                            lastReArmMs = millis();  // был приём — сброс AGC откладываем
                            radio.startReceive();
                        }
                }
            } else {
                // Захват сорвался (CRC и т.п.) — флаг RX_DONE мог остаться,
                // что приведёт к бесконечному циклу. Сбрасываем флаги и ре-армим.
                rxErrorCount++;
                Serial.printf("[RX] readData error %d, re-arming (всего %lu)\n", state,
                              (unsigned long)rxErrorCount);
                if (otaFastMode) fastRxErrors++;
                radio.clearIrqStatus();
                rearmRadioAGC();
            }
        }
    }
    const uint32_t rxMs = rxDbg ? millis() - rxT0 : 0;
    if (rxDbg && rxMs >= 200)
        Serial.printf("[RX] тик %lu мс: флаги %lu, чтение %lu, разбор %lu%s\n",
                      (unsigned long)rxMs, (unsigned long)(rxTFlags - rxT0),
                      (unsigned long)(rxDidRead ? rxTRead - rxTFlags : 0),
                      (unsigned long)(rxDidRead ? millis() - rxTRead : 0),
                      rxDidRead ? "" : " (пакета не было)");
}
