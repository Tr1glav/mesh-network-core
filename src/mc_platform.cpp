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

__attribute__((weak))
String mcLocalIp() {
    return String("0.0.0.0");
}

// ===== «ВТОРЫЕ УШИ» =====
__attribute__((weak))
void mcOnFreshFrame(const uint8_t* buf, size_t len, float rssi, float snr) {
    (void)buf; (void)len; (void)rssi; (void)snr;
}

__attribute__((weak))
bool mcRelayFrameToSupport(const uint8_t* frame, int len) {
    (void)frame; (void)len;
    return false;
}

// ===== УЗЕЛ-ПРОШИВАЛЬЩИК =====
__attribute__((weak))
bool supportPresent() {
    return false;
}

__attribute__((weak))
bool supportHearsDirect(const String& target) {
    (void)target;
    return false;
}

__attribute__((weak))
bool supportBusy() {
    return false;
}

__attribute__((weak))
bool supportJobStart(uint8_t kind, const String& target) {
    (void)kind; (void)target;
    return false;
}

__attribute__((weak))
bool supportJobFinished(uint8_t& kind, bool& ok, String& target) {
    (void)ok; (void)target;
    kind = SUP_JOB_NONE;
    return false;
}