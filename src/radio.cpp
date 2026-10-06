#include "config.h"
#include "globals.h"
#include "crypto.h"   // fmtFix: печать чисел без float-printf
#include "mesh.h"     // meshTxDropQueued: копии флуда не должны уехать после FSK
#include "radio.h"
#include "ota.h"      // slog: сверка бюджета должна быть видна без USB

void rearmRadioAGC() {
    radio.sleep();
    radio.startReceive();
    lastReArmMs = millis();
}

// Дождаться тишины в канале. Возвращает true, если канал освободился, false — если предел
// исчерпан и передавать придётся поверх (см. CAD_WAIT_BUDGET_MS).
//
// Предел общий по времени, а не только по числу попыток. Предел по попыткам давал хвост до
// 8 * 480 = 3.8 с на КАЖДУЮ передачу, а floodSend отправляет копию за копией: сообщение
// уходило в эфир через десяток секунд после нажатия, и ответ на него уже не догонял
// отправителя. Теперь ждать дольше CAD_WAIT_BUDGET_MS бессмысленно — сообщение к этому
// моменту уже никому не нужно.
//
// В быстром режиме OTA проверка пропускается: там FSK, а CAD — это про LoRa; к тому же
// быстрый канал отдельный, и на нём кроме нас никого нет.
// Время кадра в эфире по текущим настройкам радио. RadioLib считает это по выставленным
// SF/BW/CR/преамбуле и отдаёт микросекунды.
//
// Округление ВВЕРХ и только вверх: число идёт в паузы и бюджеты ожидания, и ошибка вниз
// означает передачу поверх собственной предыдущей копии. В быстром режиме (FSK) спрашивать
// бессмысленно — там другая модуляция и отдельный канал, на котором кроме нас никого нет;
// возвращаем бюджет сборки, чтобы вызывающий не получил ноль.
uint32_t radioAirtimeMs(int len) {
    if (otaFastMode) return FRAME_AIRTIME_MS;
    if (len <= 0) len = 1;
    if (len > 255) len = 255;
    const uint32_t us = radio.getTimeOnAir((size_t)len);
    if (us == 0) return FRAME_AIRTIME_MS;      // радио не настроено — не врём нулём
    const uint32_t ms = (us + 999) / 1000;
    return ms ? ms : 1;
}

static bool waitChannelFree() {
    if (otaFastMode) return true;
    const unsigned long startedMs = millis();
    for (int i = 0; i < CAD_MAX_TRIES; i++) {
        const int16_t st = radio.scanChannel();
        if (st == RADIOLIB_CHANNEL_FREE) return true;
        // Занято (LORA_DETECTED / PREAMBLE_DETECTED) либо ошибка сканирования — ждём и
        // пробуем снова. Пауза случайная: иначе два узла, дождавшиеся конца чужой передачи,
        // столкнутся уже друг с другом.
        const int pauseMs = random(CAD_RETRY_MIN_MS, CAD_RETRY_MAX_MS);
        if ((unsigned long)(millis() - startedMs) + pauseMs >= CAD_WAIT_BUDGET_MS) break;
        delay(pauseMs);
    }
    cadGiveUps++;
    Serial.printf("[TX] канал занят дольше %d мс — передаю поверх (всего %lu)\n",
                  CAD_WAIT_BUDGET_MS, (unsigned long)cadGiveUps);
    return false;
}

int txFrame(uint8_t* frame, int f) {
    // Проверка занятости канала уводит чип в standby и затирает ожидающий RX_DONE, а вместе
    // с ним уже принятый пакет: он был получен, но не прочитан. Пока идёт передача, мы всё
    // равно глухи, поэтому честнее сначала разобрать то, что пришло, — и не терять кадр
    // только из-за того, что мы сами что-то собрались слать. В быстром режиме путь другой
    // (там разбор сырых кадров может сам слать), поэтому пропускаем.
    static bool draining = false;   // защита от повторного входа из разбора принятого
    if (!otaFastMode && !draining && isListening &&
        (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE)) {
        draining = true;
        radioRxTick();
        draining = false;
    }

    waitChannelFree();
    framesSentCount++;
    // Время эфира складываем расчётом по длине кадра: по нему приложение и страница
    // показывают занятость эфира, а она решает, сколько сеть ещё выдержит.
    txAirtimeMs += (uint32_t)radioAirtimeMs(f);
    #if HAS_FEM
    digitalWrite(FEM_TX_PIN, HIGH);
    #endif
    int st = radio.transmit(frame, f);
    #if HAS_FEM
    digitalWrite(FEM_TX_PIN, LOW);
    #endif

    if (st == RADIOLIB_ERR_NONE) {
        Serial.println("[TX] OK");
    } else {
        Serial.printf("[TX] FAILED %d\n", st);
    }

    // после TX обязательно вернуться в RX; при ошибке — полный ре-арм AGC
    if (radio.startReceive() != RADIOLIB_ERR_NONE) {
        rearmRadioAGC();
    }
    return st;
}

// SX1262-специфичные настройки включаются build_flags'ами
// (см. platformio.ini / board_config.h), чтобы на других платах
// не применять опции, нужные только Heltec V4. begin/beginFSK их сбрасывают.
static void applyBoardRadioOptions() {
    #ifdef SX126X_DIO2_AS_RF_SWITCH
    radio.setDio2AsRfSwitch(true);
    #endif
    #ifdef SX126X_RX_BOOSTED_GAIN
    radio.setRxBoostedGainMode(true);
    #endif
    #ifdef SX126X_CURRENT_LIMIT
    radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
    #endif
    #ifdef SX126X_REGISTER_PATCH
    // патч регистра 0x8B5 для улучшенного приёма на Heltec v4
    uint8_t r_data = 0;
    radio.readRegister(0x8B5, &r_data, 1);
    r_data |= 0x01;
    radio.writeRegister(0x8B5, &r_data, 1);
    #endif
}

bool initLoRa() {
    Serial.println("Init LoRa...");
    
    // Вызов begin() БЕЗ параметра TCXO.
    // RadioLib возьмёт значение из макроса SX126X_DIO3_TCXO_VOLTAGE,
    // который мы определим в platformio.ini.
    char fr[16], bw[12];
    Serial.printf("[RADIO] %s МГц BW %s SF%u CR%u sync 0x%02X %d дБм\n",
                  fmtFix(cfg.loraFreq, 4, fr, sizeof(fr)), fmtFix(cfg.loraBw, 1, bw, sizeof(bw)),
                  cfg.loraSf, cfg.loraCr, cfg.loraSync, cfg.loraTx);
    int state = radio.begin(
        cfg.loraFreq, cfg.loraBw, (uint8_t)cfg.loraSf, (uint8_t)cfg.loraCr,
        (uint8_t)cfg.loraSync, (int8_t)cfg.loraTx, (uint16_t)cfg.loraPre, 1.8
    );
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("LoRa OK (TCXO from macro)");
        radio.setCRC(true);
        applyBoardRadioOptions();
        // Сверка бюджета с действительностью. Все таймауты протокола выведены из
        // FRAME_AIRTIME_MS — времени, которое САМЫЙ БОЛЬШОЙ кадр занимает эфир. Это
        // константа сборки, а SF и полоса лежат в NVS и меняются из консоли: узел,
        // переведённый на SF11, висит в эфире в несколько раз дольше, и тогда каждый
        // бюджет ожидания короче, чем нужно, — ответы начинают засчитываться потерями
        // без единой ошибки в коде. Молчать об этом нельзя, а падать не за что: сеть
        // работает, просто пороги подобраны не под эти настройки.
        // Через slog, а не только в Serial: USB к узлу не подключён почти никогда, а
        // настройки радио меняются по сети — со страницы координатора. Предупреждение,
        // видное лишь по кабелю, для работающего узла не существует.
        //
        // Признак здесь — FEATURE_MESH_OTA_SENDER, и это не опечатка: сам slog объявлен в
        // ota.h, но ОПРЕДЕЛЁН внутри `#if FEATURE_MESH_OTA_SENDER` в ota.cpp, то есть у
        // сенсора его нет вовсе (линковка падает на undefined reference — так и выяснилось).
        // Журнал страницы к раздаче прошивки отношения не имеет, и держать его за этим
        // признаком странно, но разбирать это здесь не место: у кого страницы нет, тому и
        // журнал читать негде, остаётся Serial.
        const uint32_t airWorst = radioAirtimeMs(255);
        const char* fmtWarn = "[RADIO] ВНИМАНИЕ: кадр 255 Б висит в эфире %lu мс, а бюджет "
                              "сборки FRAME_AIRTIME_MS = %d мс. Все выведенные из него "
                              "таймауты короче нужного: верните прежний SF/полосу или "
                              "пересоберите ядро\n";
        const char* fmtOk = "[RADIO] кадр 255 Б в эфире %lu мс, бюджет %d мс\n";
        const char* fmt = (airWorst > FRAME_AIRTIME_MS) ? fmtWarn : fmtOk;
        #if FEATURE_MESH_OTA_SENDER
        slog(fmt, (unsigned long)airWorst, (int)FRAME_AIRTIME_MS);
        #else
        Serial.printf(fmt, (unsigned long)airWorst, (int)FRAME_AIRTIME_MS);
        #endif
        return true;
    }
    
    Serial.printf("LoRa FAILED: %d\n", state);
    return false;
}

void radioSetParams(float freq, float bw, int sf, int cr) {
    radio.standby();
    radio.setFrequency(freq);
    radio.setBandwidth(bw);
    radio.setSpreadingFactor(sf);
    radio.setCodingRate(cr);
    lastReArmMs = 0;           // разрешить AGC rearm сразу
    radio.startReceive();
    isListening = true;
}

static bool radioFsk = false;
// Годится ли радио после последнего перехода в быстрый режим: beginFSK возвращает ошибку,
// если чип не отвечает или состояние не переключается. Если так — прошивку в эфир не
// начинаем (она всё равно не дойдёт), а сессия отменяется явно, а не молча в пустоту.
static bool radioFastReady = false;

void radioSetNormalConfig() {
    radioFastReady = false;
    if (radioFsk) {
        radioFsk = false;
        if (!initLoRa()) {
            // Радио не поднялось — честно сообщаем, что приём выключен, а не ставим
            // isListening=true на мёртвом чипе (по нему mesh решит, что сеть есть).
            Serial.printf("[RADIO] возврат в LoRa не удался — приём ВЫКЛЮЧЕН\n");
            isListening = false;
            return;
        }
        lastReArmMs = 0;
        radio.startReceive();
        isListening = true;
        return;
    }
    radioSetParams(cfg.loraFreq, cfg.loraBw, (int)cfg.loraSf, (int)cfg.loraCr);
}

void radioSetFastConfig() {
    // Неотправленные копии флуда отбрасываем: дальше другая модуляция, и копия, уехавшая
    // после переключения, уйдёт в эфир мусором на чужом канале.
    meshTxDropQueued();
    fastRxFrames = 0;
    fastRxErrors = 0;
    int st = radio.beginFSK(OTA_FAST_FREQ, OTA_FSK_BR, OTA_FSK_DEV, OTA_FSK_RXBW,
                            (int8_t)cfg.loraTx, OTA_FSK_PREAMBLE, 1.8);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("[RADIO] beginFSK failed %d\n", st);
        radioFastReady = false;
        // radioFsk не трогаем: переход не состоялся, остаёмся в LoRa
        isListening = false;
        return;
    }
    radioFastReady = true;
    radioFsk = true;
    applyBoardRadioOptions();
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    uint8_t sw[] = { 0xBE, 0xEF, 0x07, 0xA5 };
    radio.setSyncWord(sw, sizeof(sw));
    lastReArmMs = 0;
    radio.startReceive();
    isListening = true;
}

bool radioFastReadyNow() { return radioFastReady; }
