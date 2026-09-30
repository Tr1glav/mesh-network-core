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

void radioRxTick() {
    // ===== ПРИЁМ =====
    if (isListening) {
        // Периодический сброс AGC, если не идёт приём пакета прямо сейчас.
        // Не сбрасываем, пока стоит RX_DONE (иначе потеряем пакет).
        bool rxPending = (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) != 0;
        if (!rxPending && (millis() - lastReArmMs > RADIO_REARM_INTERVAL_MS)) {
            rearmRadioAGC();
            rxPending = (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) != 0;
        }

        // читать только если радио действительно получило пакет (RX_DONE)
        if (rxPending) {
            uint8_t buffer[256];
            int state = radio.readData(buffer, sizeof(buffer));
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
                Serial.printf("[RX] readData error %d, re-arming\n", state);
                if (otaFastMode) fastRxErrors++;
                radio.clearIrqStatus();
                rearmRadioAGC();
            }
        }
    }
}
