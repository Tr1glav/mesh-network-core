#include "mc_platform.h"

// Слабые реализации хуков по умолчанию: no-op / «не существует». Прошивка
// переопределяет их сильными (см. mc_platform.h).

// ===== ЭКРАН =====
__attribute__((weak))
void mcUiIncoming(const String& channelName, const String& sender, const String& msg,
                  float rssi, float snr, int hopCount, const String& path) {
    (void)channelName; (void)sender; (void)msg; (void)rssi; (void)snr;
    (void)hopCount; (void)path;
}

__attribute__((weak))
void mcUiSensorRx(bool snsPub, float rssi) {
    (void)snsPub; (void)rssi;
}

__attribute__((weak))
void mcUiHexScreen(int pktLen, float rssi, float snr, const uint8_t* buffer) {
    (void)pktLen; (void)rssi; (void)snr; (void)buffer;
}

__attribute__((weak))
void mcUiSetBrightness(uint8_t bri) {
    (void)bri;
}

__attribute__((weak))
void mcUiOtaAbort(const char* why) {
    (void)why;
}

__attribute__((weak))
void mcUiOtaProgress(const String& target, uint32_t pct, uint32_t pkts, float rssi, float snr) {
    (void)target; (void)pct; (void)pkts; (void)rssi; (void)snr;
}

__attribute__((weak))
void mcUiOtaDone(const String& target) {
    (void)target;
}

__attribute__((weak))
void mcUiOtaSensorProgress(uint32_t got, uint32_t total, uint32_t pkts, float rssi, float snr) {
    (void)got; (void)total; (void)pkts; (void)rssi; (void)snr;
}

__attribute__((weak))
void mcUiOtaSensorAbort(const char* why, uint32_t rxFrames, uint32_t rxErr) {
    (void)why; (void)rxFrames; (void)rxErr;
}

__attribute__((weak))
void screenWake() {
}

__attribute__((weak))
void mcScreenToggle() {
}

// ===== БАТАРЕЯ =====
__attribute__((weak))
bool mcBatteryPresent() {
    return false;
}

__attribute__((weak))
int mcBatteryPercent() {
    return -1;
}

__attribute__((weak))
float mcBatteryVoltage() {
    return 0.0f;
}

// ===== ПОЗИЦИЯ (GPS) =====
__attribute__((weak))
bool mcBoardPosition(int32_t* latUdeg, int32_t* lonUdeg) {
    (void)latUdeg; (void)lonUdeg;
    return false;
}

// ===== СЕТЬ =====
__attribute__((weak))
bool mcWifiConnected() {
    return false;
}

// ===== ПЕРИОДИЧЕСКОЕ В ЦИКЛЕ УЗЛА =====
__attribute__((weak))
void mcUiTick() {}

// ===== КНОПКА: ЖЕЛЕЗО =====
// «Кнопки на этой плате нет» — правильный ответ по умолчанию: ядро тогда не разбирает фронты
// и не читает уровень несуществующего пина.
__attribute__((weak))
bool mcButtonAttach() {
    return false;
}

__attribute__((weak))
bool mcButtonDown() {
    return false;
}

__attribute__((weak))
void mcCompanionTick() {}

// ===== ПОДТВЕРЖДЕНИЕ ДОСТАВКИ =====
__attribute__((weak))
void mcOnAckRecv(const uint8_t ack4[4]) {
    (void)ack4;
}

__attribute__((weak))
void mcOnPathRecv(uint8_t srcHash, uint8_t pathLen, const uint8_t* path,
                  uint8_t extraType, const uint8_t* extra, int extraLen) {
    (void)srcHash; (void)pathLen; (void)path; (void)extraType; (void)extra; (void)extraLen;
}

// ===== ОТВЕТ РЕТРАНСЛЯТОРА =====
__attribute__((weak))
void mcOnResponseRecv(uint8_t srcHash, const uint8_t* srcPub, const uint8_t* data, int len) {
    (void)srcHash; (void)srcPub; (void)data; (void)len;
}

// ===== СЫРОЙ ЖУРНАЛ ПРИЁМА =====
__attribute__((weak))
void mcOnRawRx(const uint8_t* raw, int len, float snr, float rssi) {
    (void)raw; (void)len; (void)snr; (void)rssi;
}

// ===== «ВТОРЫЕ УШИ» =====
__attribute__((weak))
void mcOnFreshFrame(const uint8_t* buf, size_t len, float rssi, float snr) {
    (void)buf; (void)len; (void)rssi; (void)snr;
}

__attribute__((weak))
bool mcRelayFrameToSupport(const char* supName, const uint8_t* frame, int len) {
    (void)supName; (void)frame; (void)len;
    return false;
}

// ===== УЗЕЛ-ПРОШИВАЛЬЩИК =====
__attribute__((weak))
bool supportPresent() {
    return false;
}

__attribute__((weak))
int supportIndexFor(const String& target) {
    (void)target;
    return -1;
}

__attribute__((weak))
int supportIndexForAny(const String& target) {
    (void)target;
    return -1;
}

__attribute__((weak))
bool supportBusy() {
    return false;
}

__attribute__((weak))
bool supportJobStart(uint8_t kind, int supIdx, const String& target) {
    (void)kind; (void)supIdx; (void)target;
    return false;
}

__attribute__((weak))
bool supportJobFinished(uint8_t& kind, bool& ok, String& target, String& who) {
    (void)ok; (void)target; (void)who;
    kind = SUP_JOB_NONE;
    return false;
}