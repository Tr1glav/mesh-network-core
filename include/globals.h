#pragma once

#include "config.h"

extern unsigned long lastRxDisplay;
extern unsigned long lastDisplayUpdate;
extern SX1262 radio;
extern MeshChannel channels[MAX_CHANNELS];
extern int numChannels;
extern String privateChannelName;
extern int privateChannelIdx;
extern String sensorChannelName;
extern int sensorChannelIdx;
extern String sensorDeviceDisc[SENSOR_DEV_CACHE_MAX];
extern int sensorDeviceDiscCount;
extern unsigned long sensorLastActive[SENSOR_DEV_CACHE_MAX];
extern bool sensorOnlineNow[SENSOR_DEV_CACHE_MAX];
extern bool sensorDiscPublished[SENSOR_DEV_CACHE_MAX];
// Сущность device_tracker публикуется не вместе с остальными, а при первых пришедших
// координатах: у узла без приёмника её быть не должно — иначе в Home Assistant висела бы
// точка на карте, которая никогда не обновится.
extern bool sensorPosPublished[SENSOR_DEV_CACHE_MAX];
extern String sensorFwVersion[SENSOR_DEV_CACHE_MAX];
extern String sensorEnv[SENSOR_DEV_CACHE_MAX];   // окружение сборки из hello
extern int sensorBattery[SENSOR_DEV_CACHE_MAX];
extern float sensorRssi[SENSOR_DEV_CACHE_MAX];
// Через сколько ретрансляторов пришёл последний пакет от узла, ПРИНЯТЫЙ НАМИ ИЗ ЭФИРА.
// 0 — слышим напрямую, 0xFF — из эфира не слышали ни разу. По нему решается, можно ли
// прошивать узел по радио, поэтому кадры, принесённые прошивальщиком по сети, это значение
// не трогают: в них хопы посчитаны от ЕГО антенны, а не от нашей.
extern uint8_t sensorHops[SENSOR_DEV_CACHE_MAX];
// Кто из прошивальщиков слышал узел последним и с каким качеством. 0 — никто (кадры
// приходят к нам из эфира), 0xFF — слышал, но его уже нет в реестре, иначе индекс+1 в
// supports[]. Раньше эта оценка ложилась в sensorRssi и sensorHops, затирая наши: узел,
// который координатор слышит напрямую, после одного форварда выглядел неслышимым — и шить
// его сам координатор уже не брался.
extern uint8_t sensorViaSup[SENSOR_DEV_CACHE_MAX];
extern float sensorViaRssi[SENSOR_DEV_CACHE_MAX];
// ===== РЕЕСТР ПРОШИВАЛЬЩИКОВ =====
// Их может быть несколько (SUPPORT_MAX): каждый стоит там, где слышно свою часть сети, и
// объявляется в эфире вместе со своим heartbeat. Образ уходит к нему по сети, поэтому
// адрес и нужен. Запись живёт SUPPORT_STALE_MS с последнего объявления.
struct SupportNode {
    String name;
    String ip;
    unsigned long seenMs;
    // Версия и окружение прошивальщика. Их координатор берёт не из радио-heartbeat, а из
    // его же /info по сети: heartbeat приходит раз в десять минут, и до первого из них узел
    // на странице висел без версии, хотя по сети его можно спросить в любой момент — и
    // ответ будет точнее, чем запомненный с прошлого раза.
    String ver;
    String env;
    unsigned long infoMs;     // когда последний раз спрашивали /info
};
extern SupportNode supports[SUPPORT_MAX];
extern int supportCount;

// Объявление из эфира: добавляет запись или обновляет существующую (ключ — имя узла).
// Реестр полон — вытесняется тот, кто дольше всех молчит. Возвращает индекс.
int supportSeen(const String& name, const String& ip);
// Индекс по имени, или -1. Заодно отвечает на вопрос «эта цель — сам прошивальщик?».
int supportFind(const String& name);
// Жива ли запись: объявлялся не дольше SUPPORT_STALE_MS назад и адрес у него есть.
bool supportLive(int idx);

// Координатор, из которого растут «вторые уши». Адрес прошивальщик знает из своих настроек
// (cfg.coordHost), а здесь лежит подтверждённый: его ставит успешная отметка (POST /support).
// Пусто — отметка ещё не дошла, и слать кадры некуда.
extern String coordIp;
extern unsigned long coordSeenMs;
// Кому передана текущая сессия; пусто — ведём сами. Пока не пусто, страница координатора
// показывает ход сессии, спрашивая его у прошивальщика.
extern String otaDelegate;
extern unsigned long otaDelegateMs;   // когда передали: до старта он ещё в фазе 0
// Сообщение о том, ЧТО ПРОИЗОШЛО, а не о том, что сломалось. Раньше такие сообщения
// клались в otaLastErr — и страница честно показывала «Ошибка: сессию ведёт ...» на
// успешно идущей прошивке.
extern char otaNote[64];
extern unsigned long timeSyncMs;
// Качество последнего пакета синхронизации времени. Такие пакеты шлёт в сенсорный канал
// именно координатор, поэтому их RSSI/SNR — это измерение связи С НИМ, а не со случайным
// соседом, чей пакет просто пришёл последним. По ним узел и показывает качество сети.
extern float timeSyncRssi, timeSyncSnr;
extern bool isListening;
extern int packetCount;
extern String lastMessage;
extern String lastSender;
extern String lastChannelName;
extern char lastPath[100];
extern float lastRSSI;
extern float lastSNR;
extern uint8_t lastHopCount;
extern int lastChannelIdx;
// Разобранный heartbeat последнего сообщения и его запись в реестре. Разбор делает ядро
// (sensor_registry.cpp), а прошивка берёт готовые поля: публикация в Home Assistant не
// должна во второй раз разбирать ту же строку.
struct SensorHello {
    String ver, batPct, batVolt, env, lat, lon;
    bool isHello = false;
};
#if FEATURE_MESH_OTA_SENDER || FEATURE_MQTT
extern SensorHello lastHello;
extern int lastSensorIdx;            // запись отправителя в реестре
extern bool lastSensorCameOnline;    // ...и он только что вернулся на связь
#endif

// Кадр пришёл не из эфира, а через форвард «вторых ушей» (MESH_RX_FORWARDED): RSSI/SNR —
// чужая оценка связи, и «напрямую» (0 хопов от нас) он не означает.
extern bool lastRxViaSupport;
// ...и через КАКОЙ прошивальщик он пришёл: ответ должен уйти тем же путём, иначе узел,
// которого слышит только этот прошивальщик, ответа не получит. Пусто — кадр из эфира.
extern String lastRxSupport;
extern PeerEntry peerCache[PEER_CACHE_MAX];
extern uint8_t bot_priv[32];
extern uint8_t bot_pub[32];
extern uint8_t bot_prv64[64];
extern uint8_t ownShortHash;
extern unsigned long lastDirectAdvertMs;
extern unsigned long lastFloodAdvertMs;
extern bool advertBootSent;
extern bool otaFastMode;
extern volatile bool otaRawDidTx;   // rawTxFrame выставляет = true; main сбрасывает перед otaHandleRawFrame
extern unsigned long lastReArmMs;
extern uint32_t fastRxFrames;
extern uint32_t fastRxErrors;
extern uint8_t seen_hashes[SEEN_HASH_COUNT * SEEN_HASH_SIZE];
extern int seen_next_idx;
extern uint8_t seen_advert_hashes[SEEN_ADVERT_HASH_COUNT * SEEN_HASH_SIZE];
extern int seen_advert_next_idx;
extern uint32_t duplicateCount;
extern uint32_t relayForwardedCount;   // чужих флуд-кадров, переизданных этим узлом
extern String logTail;
extern const char fwMarker[];   // FW_MARKER, зашит в образ для проверки платы

// Индекс канала для команд снаружи: ядро выставляет его при разборе каналов, а кто читает
// этот канал с другой стороны (брокер, приложение, что-то ещё), ядру безразлично.
extern int txChannelIdx;

// Состояние сессии раздачи прошивки — у того, кто сессию ведёт. WiFi-клиент, PubSubClient,
// WebServer, NTP и флаги Home Assistant отсюда убраны: ядро ими не пользовалось, они живут
// в прошивке (её mqtt.h и ota_internal.h).
#if FEATURE_MESH_OTA_SENDER
extern uint8_t otaPhase;
extern String otaTarget;
extern File otaFile;
extern bool otaSaving;
extern bool otaSaveOk;
extern bool otaFwReady;
extern uint32_t otaFwSize;
extern uint32_t otaFwCrc;
extern uint32_t otaSeq;
extern uint32_t otaSentBytes;
extern uint8_t otaRetries;
extern unsigned long otaSince;
extern unsigned long otaWriteCalls;
extern unsigned long otaWriteBytes;
extern unsigned long otaWriteSkipped;
#endif // FEATURE_MESH_OTA_SENDER

#ifdef SENSOR_NODE
extern bool otaActive;
extern bool otaGotStart;
extern uint32_t otaTotal;
extern uint32_t otaGot;
extern uint32_t otaCrcExp;
extern uint32_t otaCrcAcc;
extern uint32_t otaSeqExp;
extern unsigned long otaLastActivity;
extern unsigned long otaAwaitEndMs;   // таймер «образ принят, жду DONE» (0 — не вошло)
extern String sensorLastSent;
extern unsigned long sensorLastSentMs;
extern unsigned long sensorHelloDueMs;
extern bool fwVersionDiffers;
// Проверка связи по тройному нажатию: запрос, ответ и что показать на экране
extern uint16_t pingId;             // номер текущего запроса, 0 — запроса не было
extern unsigned long pingSentMs;    // когда ушёл запрос; 0 — ответа не ждём
extern unsigned long pingRttMs;     // время ответа, мс
extern float pingRssi, pingSnr;     // как сенсор слышит ответ
extern int pingPeerRssi;            // как координатор слышит сенсор (из ответа)
extern uint8_t pingHops;            // через сколько ретрансляторов пришёл ответ
extern unsigned long pingShowUntil; // до какого времени держать результат на экране
extern bool pingFailed;             // ответа на последний запрос не было
// Режим проверки доступности: запросы идут подряд, экран показывает выборку. Включается и
// выключается тем же действием, которым раньше посылался одиночный запрос.
extern bool pingModeOn;
extern unsigned long pingModeNextMs;  // когда уйдёт следующий запрос
extern unsigned long pingModeStartMs; // когда включили — по нему срабатывает предел
extern uint16_t pingStatSent;         // ушло запросов за сессию режима
extern uint16_t pingStatRecv;         // на сколько пришёл ответ
// Потери считаются отдельным счётчиком, а не как «отправлено минус отвеченные»: запрос,
// который ещё в пути, в такой разнице выглядел бы потерянным, и на экране мелькала бы
// потеря, которой нет. Здесь потеря — это сработавший таймаут ожидания.
extern uint16_t pingStatLost;
extern unsigned long pingRttMin, pingRttMax, pingRttSum;   // разброс времени ответа
// Разброс числа ретрансляторов за сессию. Хопы скачут там, где маршрут неустойчив: ответ то
// приходит напрямую, то через соседа, — и по одному последнему значению этого не видно.
extern uint8_t pingHopsMin, pingHopsMax;
extern unsigned long pingHopsSum;
#endif
