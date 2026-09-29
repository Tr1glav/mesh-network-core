#pragma once

// ===== Контракт ядро <-> прошивка =====
// Ядро meshcore (mesh-network-core) содержит только протокол: радио, mesh, крипто,
// прошивку по радио. Всё, что зависит от конкретной платы, живёт в прошивке и
// доходит сюда через хуки: экран (mcUi*), батарея (mcBattery*), сеть (mcWifi*),
// узел-прошивальщик (support*), MQTT/компаньон.
//
// В src/mc_platform.cpp лежат слабые реализации по умолчанию (обычно no-op): прошивка
// переопределяет их сильными. Так ядро собирается даже на платформе, которая не
// реализует какой-то хук вообще. check: проверять через #ifdef/#if, как и остальные
// признаки сборки.

#include <Arduino.h>

// ===== ЭКРАН =====
// Прошивка рисует по своему усмотрению (SSD1306, T-Deck LCD, без экрана — no-op).
// По умолчанию ничего не делаем; где экран нужен, прошивка переопределяет хук.

// Входящее сообщение канала, которое ядро приняло и расшифровало.
// path — маршрут ("aa bb .."), пустой — кадр пришёл напрямую.
void mcUiIncoming(const String& channelName, const String& sender, const String& msg,
                  float rssi, float snr, int hopCount, const String& path);

// Сообщение сенсорного канала: snsPub — ушло ли оно в MQTT (см. mcMqttPublishSensor).
void mcUiSensorRx(bool snsPub, float rssi);

// Групповое сообщение, которое не расшифровалось (нет ключа канала): показать hex.
void mcUiHexScreen(int pktLen, float rssi, float snr, const uint8_t* buffer);

// Применить яркость экрана сразу (команда "bri" из настройки по радио).
void mcUiSetBrightness(uint8_t bri);

// Mesh OTA, раздающая сторона (координатор/прошивальщик).
void mcUiOtaAbort(const char* why);
void mcUiOtaProgress(const String& target, uint32_t pct, uint32_t pkts, float rssi, float snr);
void mcUiOtaDone(const String& target);

// Mesh OTA, принимающая сторона (узел).
void mcUiOtaSensorProgress(uint32_t got, uint32_t total, uint32_t pkts, float rssi, float snr);
void mcUiOtaSensorAbort(const char* why, uint32_t rxFrames, uint32_t rxErr);

// Зажечь экран: используется, когда пришла прошивка и нужно показать её ход.
void screenWake();

// ===== БАТАРЕЯ (узел) =====
bool mcBatteryPresent();
int mcBatteryPercent();      // 0..100; -1 — измерения нет или аккумулятор не подключён
float mcBatteryVoltage();    // вольты; 0 — платы без измерения батареи

// ===== ПОЗИЦИЯ (GPS) =====
// Координаты узла в микроградусах для hello: плата с приёмником отвечает true и пишет
// широту/долготу; без приёмника те поля в heartbeat не дописываются вовсе.
bool mcBoardPosition(int32_t* latUdeg, int32_t* lonUdeg);

// ===== СЕТЬ (узел-прошивальщик) =====
// Объявление "support:<адрес>" полезно только когда WiFi поднялся.
bool mcWifiConnected();
String mcLocalIp();          // на случай, если прошивке самому знать свой адрес

// ===== «ВТОРЫЕ УШИ» =====
// Свежий кадр принят с радио (прошёл дедуп, не сырой fast-кадр). Вызывается на любом
// узле; смысл имеет на прошивальщике: он слышит дальние узлы лучше координатора, и его
// прошивка перекрывает хук, чтобы слать такие кадры координатору по сети (POST /ears).
// Остальные платформы живут со слабой заглушкой.
void mcOnFreshFrame(const uint8_t* buf, size_t len, float rssi, float snr);

// Передать кадр прошивальщику по сети, чтобы он вывел его в эфир СО СВОЕГО радио.
// Используется для ответов на пинг, пришедший через «вторые уши»: узел, которого слышит
// только прошивальщик, ответ с радио координатора не услышит. Реализует координатор
// (POST /radiotx); вернёт false, если прошивальщика нет или сеть молчит, — тогда ядро
// уходит на обычный локальный флуд.
bool mcRelayFrameToSupport(const char* supName, const uint8_t* frame, int len);

// ===== MQTT (координатор) =====
// Опубликовать последнее принятое сообщение (сводный топик координатора) / данные датчика.
// Реализует прошивка (mqtt.cpp) — и только публикует: реестр узлов и разбор heartbeat живут
// в ядре (sensor_registry.cpp), а здесь берутся готовыми из lastHello.
#if FEATURE_MQTT
void publishMessage();
bool publishSensorMessage();
#endif

// ===== КОМПАНЬОН (BLE-приложение) =====
// Протокол телефонного приложения. Реализует прошивка (companion.cpp); вызовы под
// #ifdef COMPANION_NODE.
#ifdef COMPANION_NODE
void companionOnChannelText(int channelIdx, const String& text, float snr, uint8_t pathLen,
                            bool notify = true);
void companionOnAdvert(const uint8_t* pub, const uint8_t* app, int applen,
                       uint8_t pathLen, const uint8_t* path);
#endif

// ===== УЗЕЛ-ПРОШИВАЛЬЩИК (support) =====
// Сессию mesh OTA вместо координатора может вести отдельный узел-прошивальщик: он
// стоит там, где слышно узлы напрямую. Реализует прошивка (support.cpp); пока её нет —
// слабые заглушки говорят «прошивальщика не существует», и ядро ведёт сессии само.
enum : uint8_t { SUP_JOB_NONE = 0, SUP_JOB_SELF, SUP_JOB_HANDOFF };

// Есть ли в сети хоть один живой прошивальщик.
bool supportPresent();
// Кто из них дотягивается до этой цели лучше всех (наименьшее число хопов, тот же порог
// otaHopsReachable) — индекс в supports[] или -1. Спрашивает у каждого по сети, поэтому
// живёт в прошивке, а не в ядре.
int supportIndexFor(const String& target);
bool supportBusy();
// Сессию ведёт конкретный прошивальщик: supIdx — индекс в supports[].
bool supportJobStart(uint8_t kind, int supIdx, const String& target);
// who — имя того, кто вёл: его показывает страница и по нему ставится otaDelegate.
bool supportJobFinished(uint8_t& kind, bool& ok, String& target, String& who);