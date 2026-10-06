#pragma once

#include "config.h"

void initAdvertIdentity();
uint8_t* findPeerPub(uint8_t hash);
void rememberPeerPub(uint8_t hash, const uint8_t* pub);
// Хэш кадра для дедупликации: [тип пакета 1B][тело после пути], путь в хэш не входит.
bool meshFrameHashOf(const uint8_t* data, int len, uint8_t out[32]);
bool checkAndMarkSeen(uint8_t* data, int len);
// Пометить СВОЙ ушедший кадр как «уже виденный» — эхо, вернувшееся от ретранслятора,
// не должно заново пройти dedup и быть переизданным (см. mesh.cpp).
void markOwnFrameSeen(const uint8_t* data, int len);
// openKey = true: ключ канала не секрет (выведен из имени или общеизвестен)
void addChannelKey16(const char* name, const uint8_t* key16, bool openKey = false);
void deriveChannels();
int findChannelByName(const char* name);
// Завести или заменить канал в ячейке idx (idx == numChannels — добавить в конец).
// Возвращает номер канала или -1. Сохранением занимается вызывающий.
int channelSetSlot(int idx, const char* name, const uint8_t* key16, bool openKey = false);
// Ключ канала не секрет: автоключ по имени либо общеизвестный PSK (#public). Такой канал
// читает и пишет любой, кто знает имя, поэтому прошивку по радио на нём не раздаём и не
// принимаем. Неизвестный номер канала тоже считается открытым.
bool channelKeyIsOpen(int idx);
void loadPrivateChannel();
void loadSensorChannel();
#if FEATURE_MQTT
void loadTxChannel();
#endif
void buildPingReply(char* out, size_t outlen, const uint8_t* path, uint8_t hop_count, uint8_t path_hash_size);
// outTs — куда вернуть метку, которая легла в кадр: с ней же эхо собственного сообщения
// кладётся в очередь приложения, иначе приложение не найдёт его в сыром журнале 0x88.
// msgTs — метка ИЗ КОМАНДЫ приложения; 0 означает «по часам узла» (сообщения самого узла).
int buildGroupEnc(int chIdx, const String& msg, uint8_t* enc, uint32_t* outTs = NULL,
                  uint32_t msgTs = 0);
int buildGroupFrameFlood(int chIdx, const String& msg, uint8_t* frame, int maxlen,
                         uint32_t* outTs = NULL, uint32_t msgTs = 0);
// expectedAck4 — куда положить хэш подтверждения, которое ПРИДЁТ на это сообщение. Считается
// здесь, а не у вызывающего, потому что считается по тому же открытому тексту, который
// собирается внутри: вынеси расчёт наружу — и он разойдётся с кадром при первой правке
// раскладки. Получатель считает тот же хэш по нашему публичному ключу (см. dmAckSend).
// msgTs/attempt — метка времени и номер попытки из КОМАНДЫ приложения: они входят в
// открытый текст (и потому в хэш подтверждения), а приложение показывает сообщение по своей
// метке и отличает повторы по номеру попытки. Раньше кадр собирался с time(NULL)/0, и
// подтверждение, которое прилетит на этот кадр, не совпадало ни с чем, что знает приложение.
// msgTs == 0 означает «по часам узла» — для ответов самого узла (mesh_rx.cpp), у которых
// метки приложения нет вовсе.
// txtType — тип текста из приложения: 0 обычное сообщение, 1 команда командной строки
// (ею управляют ретранслятором). Уезжает в эфир в старших битах байта флагов, иначе узел на
// той стороне прочитает команду как беседу.
int buildPrivateTextFrame(uint8_t dest_hash, const uint8_t* dest_pub,
                          const String& msg, uint8_t* frame, int maxlen,
                          uint8_t* expectedAck4 = NULL,
                          uint32_t msgTs = 0, uint8_t attempt = 0, uint8_t txtType = 0);
// logHex печатает кадр в журнал. Выключайте, когда кадр уже напечатан: копии флуда
// побайтово равны, и дамп на каждую копию только тормозит UART, ничего не добавляя.
// Вход на ретранслятор: кадр ANON_REQ с паролем. Конверт несёт НАШ ПОЛНЫЙ ключ — узел нас
// ещё не знает, и общий секрет ему вывести больше неоткуда. outTag — метка, с которой
// придёт ответ.
int buildLoginFrame(uint8_t dest_hash, const uint8_t* dest_pub, const char* password,
                    uint32_t* outTag, uint8_t* frame, int maxlen);
// Запрос к ретранслятору: конверт как у лички, содержимое [метка 4][тело]. Ответ придёт с
// той же меткой — по ней и сопоставляется, кому он принадлежит.
int buildReqFrame(uint8_t dest_hash, const uint8_t* dest_pub,
                  const uint8_t* reqData, int reqLen,
                  uint32_t* outTag, uint8_t* frame, int maxlen);
// Он же для запроса одного типа (REQ_TYPE_*): тело [тип 1][резерв 4][случайные 4].
int buildSimpleReqFrame(uint8_t dest_hash, const uint8_t* dest_pub, uint8_t reqType,
                        uint32_t* outTag, uint8_t* frame, int maxlen);
// Возврат маршрута отправителю: дорога ДО НАС плюс необязательный довесок (обычно
// подтверждение доставки). Конверт тот же, что у лички; см. buildPathReturnFrame.
int buildPathReturnFrame(uint8_t dest_hash, const uint8_t* dest_pub,
                         uint8_t pathLen, const uint8_t* path,
                         uint8_t extraType, const uint8_t* extra, int extraLen,
                         uint8_t* frame, int maxlen);
int sendFrame(int chIdx, const uint8_t* frame, int f, bool logHex = true);
// Пауза между копиями флуда: случайная величина из FLOOD_RETRY_MIN_MS…MAX_MS плюс
// FLOOD_JITTER_MS разброса поверх, чтобы соседи не повторяли копии синхронно.
// 0 — «пауза по конфигурации»; своё значение вызывающий код задаёт только когда ему нужна
// именно такая БАЗА. Объявлена здесь, потому что ею пользуются и sendAdvert, и floodSend.
//
// Параметр называется gapBaseMs, а не gapMs, и это важно. Раньше он назывался gapMs, то есть
// «пауза», а смысл у него другой с тех пор, как паузу стали брать из всего диапазона флуда:
// переданное значение — лишь БАЗА, которую зажимают в FLOOD_RETRY_MIN_MS…MAX_MS. Вызывающий,
// попросивший 20 мс, молча получал не меньше 1000. Имя теперь говорит, что это база.
unsigned int floodGapMs(unsigned int baseMs = 0);

// ===== Очередь копий флуда =====
// Копии сообщения уходят не подряд: первая сразу, остальные из тика по одной, когда
// подойдёт их срок. Так главный цикл не стоит на delay() между копиями — на коротком
// сообщении это было около 4.7 с от нажатия кнопки до конца отправки, и всё это время
// прошивка не обслуживала ни экран, ни консоль настроек, ни сторожа сессий.
//
// Главный цикл КАЖДОЙ прошивки обязан звать meshTxTick(), иначе вторая и третья копии не
// уйдут никогда: сообщение станет одноразовым, а выглядеть будет как потеря в эфире.
void meshTxTick();
// Отбросить неотправленные копии. Зовётся при уходе на быстрый канал: там другая
// модуляция, и копия, уехавшая после переключения, в эфир уйдёт мусором.
void meshTxDropQueued();
// Поставить готовый кадр в очередь с задержкой: для ответов, которые обязаны уйти позже
// (подтверждение доставки ждёт конца повторов отправителя). true — встал в очередь.
bool meshTxQueueFrame(const uint8_t* frame, int f, unsigned long delayMs);
// Одно и то же сообщение уходит в эфир FLOOD_REPEATS раз, с паузой между копиями
// разнесённой случайно. Пауза заведомо больше времени в эфире, поэтому чужая помеха
// накрывает одну копию, а не все сразу; см. FLOOD_RETRY_* в config.h.
void floodSend(int chIdx, const uint8_t* frame, int f, unsigned int gapBaseMs = 0,
               int repeats = FLOOD_REPEATS);
// То же, но БЕЗ выхода в эфир из вызывающего кода: все копии, включая первую, уходят из
// meshTxTick(). Для тех, кто обязан ответить сразу, — например обработчика команды
// приложения-компаньона: иначе подтверждение опаздывает на ожидание канала и время кадра.
// Сколько копий ждёт своей очереди в эфир: метрика для приложения и страницы.
int meshTxQueuedCount();
void floodSendQueued(int chIdx, const uint8_t* frame, int f, unsigned int gapBaseMs = 0,
                     int repeats = FLOOD_REPEATS);
void sensorSendMsg(const char* msg, unsigned int gapBaseMs = 0, int repeats = FLOOD_REPEATS);
// Сообщение, которое может дословно повториться, — с номером отправки. Без него два
// одинаковых повтора в одну секунду дают одинаковый шифротекст, а значит и одинаковый хэш:
// сеть отбросила бы второй как дубликат. Номер читатель убирает (см. sensorSendMsgUnique).
void sensorSendMsgUnique(const char* prefix);
#ifdef SENSOR_NODE
void sensorSendHello();
void sensorPingSend();      // один эхо-запрос
void sensorPingTick();      // сторож ожидания ответа и расписание режима
// Включить или выключить режим проверки доступности. При включении статистика обнуляется и
// первый запрос уходит сразу; при выключении последняя выборка остаётся на экране
// PING_SHOW_MS, чтобы её можно было прочитать.
void pingModeToggle();
#endif
#ifndef SENSOR_NODE
// Отложенный ответ на пинг/личку: заявку ставит разбор пакета, отправляет главный цикл.
// Пауза нужна, чтобы не попасть в повторы отправителя, и не должна останавливать цикл.
void meshReplyTick();
#endif
void sendAdvert(uint8_t route_type);
void sendSensorTimeSync();
String channelListStr();
bool parseMeshCorePacket(uint8_t* data, int len);

// Откуда пришёл кадр. Для radio метрику приёма даёт само радио; для forwarded — то, что
// сообщил форвардер по WiFi (хук на «вторые уши»). RSSI/SNR чужие, но это лучшая оценка
// связи с узлом, которая у нас есть; реестр узлов на них и опирается.
enum MeshRxOrigin : uint8_t {
    MESH_RX_RADIO = 0,
    MESH_RX_FORWARDED = 1
};

struct MeshRxMeta {
    MeshRxOrigin origin = MESH_RX_RADIO;
    float rssi = 0;
    float snr  = 0;
    // Для MESH_RX_FORWARDED — имя прошивальщика, который кадр принёс (он называет себя в
    // заголовке запроса). Ответ на пинг уходит через него же: с радио координатора узел,
    // которого слышит только этот прошивальщик, ответа не получит.
    String via;
};

// Единый вход для принятых кадров (радио и forwarded): дедуп по checkAndMarkSeen,
// свежий кадр разбирается как обычный. Возвращает true, если кадр был принят и обработан.
bool meshRxFrame(uint8_t* data, int len, const MeshRxMeta& meta);

// Парсинг с явной метой приёма (forwarded-кадры не читают radio.getRSSI()).
bool parseMeshCorePacket(uint8_t* data, int len, const MeshRxMeta& meta);

// Ретрансляция чужих флуд-кадров. ВЫКЛЮЧЕНА по умолчанию: ретранслятором не должен быть
// никто, FEATURE_RELAY = 0 (см. config.h). В обычной сборке обе функции ниже — пустышки, и
// звать их можно без условия; включается ретрансляция сборкой с -DFEATURE_RELAY=1. Всё
// описанное дальше относится к включённому признаку.
//
// maybeQueueRelay решает по принятому кадру и ставит его в очередь, meshRelayTick переиздаёт
// просроченные (зовётся из главного цикла).
//
// maybeQueueRelay зовётся для КАЖДОГО принятого кадра, включая копии, — и держит собственный
// кэш, чтобы не поставить один кадр в очередь дважды. Общий дедуп для этого не годится: он
// помечает кадр «увиденным» навсегда, и если очередь в этот момент оказалась полна, кадр
// терялся целиком — вторая копия отправителя приходила, признавалась дубликатом и больше не
// переиздавалась. Ответ функции решает именно эту судьбу.
#define RELAY_QUEUED  0   // кадр поставлен в очередь
#define RELAY_SKIPPED 1   // не наш, петля, либо уже переиздавали
#define RELAY_NOROOM  2   // очередь полна: хэш намеренно не запомнен, ждём следующую копию
int maybeQueueRelay(const uint8_t* data, int len);
void meshRelayTick();

// Учесть сообщение из сенсорного канала в реестре узлов: обновляет запись отправителя и
// разбирает heartbeat в lastHello. Публикацией наружу занимается прошивка — она берёт уже
// разобранное.
#if FEATURE_MESH_OTA_SENDER || FEATURE_MQTT
void sensorRegistryNote();
#endif
// Версия образа, последний раз услышанная от узла (пусто, если его нет в реестре). Отдельная
// функция, а не поле: слот в реестре — дело ядра, и снаружи его не видно.
#if FEATURE_MESH_OTA_SENDER
String sensorVersionOf(const String& name);
#endif

// forward decls referenced from parseMeshCorePacket
#if FEATURE_MESH_OTA_SENDER
void otaHandleAck();
#endif
#if FEATURE_MQTT
bool publishSensorMessage();
#endif
#ifdef SENSOR_NODE
void otaSensorHandle();
#endif
