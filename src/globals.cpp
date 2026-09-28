#include "config.h"
#include "globals.h"
#include "ota.h"      // slog: реестр прошивальщиков пишет о смене адреса

unsigned long lastRxDisplay = 0;        // millis() последнего экрана, связанного с приёмом RX
unsigned long lastDisplayUpdate = 0;    // millis() последнего обновления idle-экрана

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
MeshChannel channels[MAX_CHANNELS];
int numChannels = 0;
String privateChannelName = "";
int privateChannelIdx = -1;
String sensorChannelName = "";
int sensorChannelIdx = -1;
String sensorDeviceDisc[SENSOR_DEV_CACHE_MAX];
int sensorDeviceDiscCount = 0;
unsigned long sensorLastActive[SENSOR_DEV_CACHE_MAX];
bool sensorOnlineNow[SENSOR_DEV_CACHE_MAX];
bool sensorDiscPublished[SENSOR_DEV_CACHE_MAX]; // HA discovery сенсора отправлен в текущее подключение к брокеру
bool sensorPosPublished[SENSOR_DEV_CACHE_MAX];  // ...и его точка на карте, если координаты приходили
String sensorFwVersion[SENSOR_DEV_CACHE_MAX];   // версия прошивки из hello
String sensorEnv[SENSOR_DEV_CACHE_MAX];         // окружение сборки: по нему берётся файл релиза
int sensorBattery[SENSOR_DEV_CACHE_MAX];        // заряд % из hello; -1 — сенсор его не шлёт
float sensorRssi[SENSOR_DEV_CACHE_MAX];         // RSSI последнего пакета от сенсора
uint8_t sensorHops[SENSOR_DEV_CACHE_MAX];       // хопов до узла; 0 — напрямую, 0xFF — неизвестно
SupportNode supports[SUPPORT_MAX];   // реестр прошивальщиков: имя, адрес, когда объявлялся
int supportCount = 0;

int supportFind(const String& name) {
    for (int i = 0; i < supportCount; i++)
        if (supports[i].name == name) return i;
    return -1;
}

bool supportLive(int idx) {
    if (idx < 0 || idx >= supportCount) return false;
    return supports[idx].ip.length() > 0 &&
           (millis() - supports[idx].seenMs) < SUPPORT_STALE_MS;
}

int supportSeen(const String& name, const String& ip) {
    int idx = supportFind(name);
    if (idx < 0) {
        if (supportCount < SUPPORT_MAX) {
            idx = supportCount++;
        } else {
            // Реестр полон: вытесняем того, кто молчит дольше всех. Отказ означал бы, что
            // новый прошивальщик не появится вовсе — а старый, скорее всего, уже снят.
            idx = 0;
            for (int i = 1; i < supportCount; i++)
                if ((long)(supports[i].seenMs - supports[idx].seenMs) < 0) idx = i;
            slog("[SUP] реестр полон (%d): '%s' вытеснен узлом '%s'\n",
                 (int)SUPPORT_MAX, supports[idx].name.c_str(), name.c_str());
        }
        supports[idx].name = name;
        supports[idx].ip = "";
    }
    // Объявление приходит с каждым heartbeat, поэтому в журнал оно идёт только когда
    // адрес действительно сменился (или прошивальщик появился впервые).
    if (supports[idx].ip != ip) {
        supports[idx].ip = ip;
        // Адрес сменился — узел перезагрузился или переподключился, и версия на нём может
        // быть уже другой. Спросим заново (supportFetchInfo в прошивке).
        supports[idx].ver = "";
        supports[idx].infoMs = 0;
        slog("[SUP] прошивальщик %s на %s\n", name.c_str(), ip.c_str());
    }
    supports[idx].seenMs = millis();
    return idx;
}
bool supportAnnounceDue = false; // опрос "hello?": прошивальщика попросили назвать адрес
String coordIp = "";            // «вторые уши»: адрес координатора, принимающего /ears
unsigned long coordSeenMs = 0;
String otaDelegate = "";        // кому передана сессия; пусто — ведём сами
unsigned long otaDelegateMs = 0;
char otaNote[64] = "";          // «что произошло»: не ошибка, а сообщение
unsigned long timeSyncMs = 0;                   // millis() последнего "time:" из канала сенсоров
float timeSyncRssi = 0, timeSyncSnr = 0;        // и качество приёма этого пакета
bool isListening = false;
int packetCount = 0;
String lastMessage = "";
String lastSender = "";
String lastChannelName = "#public";
char lastPath[100] = "";
float lastRSSI = 0;
float lastSNR = 0;
uint8_t lastHopCount = 0;
int lastChannelIdx = -1;   // индекс канала последнего сообщения (-1 = не определён)
bool lastRxViaSupport = false;
String lastRxSupport = "";      // через какой прошивальщик пришёл кадр; пусто — из эфира
PeerEntry peerCache[PEER_CACHE_MAX];
uint8_t bot_priv[32];
uint8_t bot_pub[32];
uint8_t bot_prv64[64];   // ed25519 private key (seed-расширенный) для X25519
uint8_t ownShortHash = 0;
unsigned long lastDirectAdvertMs = 0;
unsigned long lastFloodAdvertMs = 0;
bool advertBootSent = false;
bool otaFastMode = false;
volatile bool otaRawDidTx = false;
unsigned long lastReArmMs = 0;
uint32_t fastRxFrames = 0;   // кадров mesh OTA, принятых на быстром канале с момента переключения
uint32_t fastRxErrors = 0;   // ошибок приёма (CRC и т.п.) на быстром канале
uint8_t seen_hashes[SEEN_HASH_COUNT * SEEN_HASH_SIZE];
int seen_next_idx = 0;
uint8_t seen_advert_hashes[SEEN_ADVERT_HASH_COUNT * SEEN_HASH_SIZE];
int seen_advert_next_idx = 0;
uint32_t duplicateCount = 0;
uint32_t relayForwardedCount = 0;   // чужих флуд-кадров, переизданных этим узлом
String logTail;
// used + печать в setup(): иначе линковщик с --gc-sections выбросит строку из образа
const char fwMarker[] __attribute__((used)) = FW_MARKER;
// Индекс канала, в который уходят команды снаружи (из Home Assistant через брокер).
// Выставляется при разборе каналов (mesh_channels.cpp) — поэтому и живёт в ядре; кто читает
// этот канал с другой стороны, ядру безразлично.
int txChannelIdx = 1;   // по умолчанию #connections

// Состояние сессии раздачи прошивки. Признак честный: это нужно тому, кто сессию ВЕДЁТ, а не
// тому, у кого собран брокер. Раньше весь блок стоял под #ifdef MQTT_ENABLED — вместе с
// WiFi-клиентом, PubSubClient, WebServer, NTP и флагами Home Assistant, которых ядро не
// касалось ни в одной строке. Всё это переехало в прошивку (mqtt.cpp, web.cpp).
#if FEATURE_MESH_OTA_SENDER
uint8_t otaPhase = OTA_PHASE_IDLE;
String otaTarget = "";
File otaFile;               // открытый /ota.bin (LittleFS)
bool otaSaving = false;     // идёт HTTP-загрузка файла на бот
bool otaSaveOk = false;     // флаг успеха сохранения (для POST-ответа)
bool otaFwReady = false;    // на боте лежит годный .otaz
uint32_t otaFwSize = 0;     // длина сжатого потока
uint32_t otaFwCrc = 0;      // CRC32 распакованного образа
uint32_t otaSeq = 0;        // первый неподтверждённый чанк
uint32_t otaSentBytes = 0;  // байт, подтверждённых сенсором
uint8_t otaRetries = 0;     // повторы подряд без прогресса
unsigned long otaSince = 0; // millis() последней отправки
unsigned long otaWriteCalls = 0;    // сколько раз вызвали WRITE
unsigned long otaWriteBytes = 0;    // сколько байт otaFile.write() подтвердил
unsigned long otaWriteSkipped = 0;  // WRITE-колбэков, где guard не прошёл
#endif // FEATURE_MESH_OTA_SENDER
#ifdef SENSOR_NODE
bool otaActive = false;     // OTA-сессия идёт (receiving)
bool otaGotStart = false;   // получили ota:start
uint32_t otaTotal = 0;      // ожидаемый размер образа (байт)
uint32_t otaGot = 0;        // записано байт образа
uint32_t otaCrcExp = 0;     // ожидаемый CRC32 образа
uint32_t otaCrcAcc = 0xFFFFFFFF;  // накапливаемый CRC32
uint32_t otaSeqExp = 0;     // следующий ожидаемый seq
unsigned long otaLastActivity = 0; // millis() последнего OTA-пакета
unsigned long otaAwaitEndMs = 0;   // millis() начала ожидания DONE после приёма всего образа
String sensorLastSent = "";        // последнее сообщение, отправленное сенсором (для экрана)
unsigned long sensorLastSentMs = 0; // millis() его отправки
unsigned long sensorHelloDueMs = 0; // когда ответить на "hello?" (0 — запроса нет)
bool fwVersionDiffers = false;      // версия бота из "time:" не совпала со своей
uint16_t pingId = 0;
unsigned long pingSentMs = 0;
unsigned long pingRttMs = 0;
float pingRssi = 0, pingSnr = 0;
int pingPeerRssi = 0;
uint8_t pingHops = 0;
unsigned long pingShowUntil = 0;
bool pingFailed = false;
#endif // SENSOR_NODE
