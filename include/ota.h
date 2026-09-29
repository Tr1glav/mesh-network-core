#pragma once

#include "config.h"

void slog(const char* fmt, ...);

// Потоковый поиск маркера платы (FW_MARKER) в образе прошивки. Образ приходит кусками,
// маркер может лечь на границу двух кусков — поэтому храним хвост предыдущего.
struct FwScan {
    char carry[40];    // хвост предыдущего куска
    uint8_t carryLen;
    bool mine;         // встретился маркер нашей платы
    char other[12];    // код чужой платы, если встретился
};
// Кадр сырого протокола прошивки: [BE EF][тип][seq 4][данные][crc16]. Собирают и шлют
// обе стороны, поэтому объявления живут здесь, а не внутри файла одной из сторон.
int rawBuildFrame(uint8_t* frm, uint8_t type, uint32_t seq, const uint8_t* data, int n);
int rawTxFrame(const uint8_t* frm, int f, bool listenAfter = true);

void fwScanReset(FwScan* s);
void fwScanFeed(FwScan* s, const uint8_t* data, size_t n);
// 1 — образ нашей платы, 0 — маркера нет (сборка старше проверки), -1 — чужая плата
int fwScanVerdict(const FwScan* s);
// Раздающая сторона прошивки и страница. Объявления вместе: у координатора и прошивальщика
// есть и то, и другое, а лишнее объявление без определения никому не мешает.
#if FEATURE_MESH_OTA_SENDER || FEATURE_WEB
void logGetSnapshot(String& tailOut, uint32_t& totalOut);
void otaTxGroup(const String& msg);
void otaBotAbort(const char* why);
void otaDrawProgress();
void otaSendStart();
void otaSendEnd();
void otaHandleAck();
void otaBotTick();
void otaInspectStoredFw();
bool otaSessionActive();
// Запуск прошивки сенсора без участия веб-запроса — нужен автообновлению
bool otaStartSession(const String& target);
// Достижим ли узел для сессии по радио при таком числе хопов (0xFF — ни разу не слышали).
// Правило живёт здесь одной функцией: его спрашивают и запуск сессии, и сторона
// прошивальщика, и порог не должен разъезжаться по копиям.
bool otaHopsReachable(uint8_t hops);
// Итог фоновой передачи образа прошивальщику: зовётся из главного цикла
void otaSupportTick();
String buildDiagReport();
void setupOtaServer();
#endif
#if FEATURE_WEB
// Очередь сообщений настройки узла: страница ставит их в очередь и сразу отвечает,
// а в эфир они уходят по одному отсюда, из главного цикла.
void webTick();
#endif
// Причина последнего отказа — её показывает страница. Определена в ota.cpp ядра, но до сих
// пор объявлялась только в заголовке ПРОШИВКИ: внутри ota.cpp она видна как своё определение,
// а другим файлам ядра — нет.
extern char otaLastErr[48];

// ===== МЕДЛЕННЫЙ РЕЖИМ (ota_slow.cpp) =====
#if FEATURE_MESH_OTA_SENDER
// Начать медленную сессию: образ едет обычными сообщениями канала и доходит туда же, куда
// доходит любое сообщение сети, — в отличие от быстрого режима, ограниченного прямой
// слышимостью. Платой становится время.
bool otaSlowStart(const String& target);
// Запустить медленный режим независимо от хопов — для проверки самого режима.
bool otaStartSessionSlow(const String& target);
void otaSlowTick();                       // шаг отправителя, из главного цикла
void otaSlowOnAck(uint32_t next);         // подтверждение узла: «жду чанк N»
void otaSlowDone(bool ok, const char* why);
void otaSlowAbort(const char* why);
#endif
#if FEATURE_MESH_OTA_RECEIVER
void otaSlowRxStart(const String& args);  // "ota:slow:<цель>:<байт>:<crc>:<чанков>"
void otaSlowRxData(const String& args);   // "osd:<seq>:<base64>"
void otaSlowRxTick();                     // сторож тишины в канале
void otaSlowRxAbort(const char* why);
// Обёртки над потоком быстрого режима: распаковка, запись и проверка у них общие.
bool otaSlowStreamBegin(uint32_t total, uint32_t crc);
bool otaSlowStreamFeed(const uint8_t* data, size_t n, bool last);
bool otaSlowStreamEnd(bool apply);
#endif

#if FEATURE_MESH_OTA_RECEIVER
void otaSensorDraw();
void otaSensorAbort(const char* why);
void otaSensorTick();
void otaSensorHandle();
// Разбор входящего кадра на стороне узла — зовёт диспетчер в ota.cpp
void otaHandleRawSensor(const uint8_t* buf, int len);
#endif
void otaHandleRawFrame(const uint8_t* buf, int len);
