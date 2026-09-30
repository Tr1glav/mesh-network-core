#include "config.h"
#include "globals.h"
#include "mesh.h"
#include "ota.h"      // slog

// ===== РЕЕСТР УЗЛОВ =====
// Что этот узел знает о других из их сообщений в сенсорном канале: когда выходили на связь,
// с каким качеством мы их слышим, через сколько ретрансляторов, какая на них версия и заряд.
//
// Место этому в ядре: это протокол, а не чья-то интеграция. Раньше реестр жил в mqtt.cpp
// прошивки, и из-за этого весь файл собирался под флагом MQTT_ENABLED даже там, где брокера
// нет вовсе, — у прошивальщика, который без реестра не может ответить координатору, кого он
// слышит. Разбор heartbeat лежал там же, вперемешку с публикацией в Home Assistant.
//
// Признак: реестр нужен тому, кто ведёт сессии прошивки (выбор цели по хопам), и тому, кто
// публикует состояние узлов наружу. Узлу, который только принимает прошивку, он не нужен.
#if FEATURE_MESH_OTA_SENDER || FEATURE_MQTT

// Срок годности оценки — тот же, по которому узел считается ушедшим: если он уже офлайн, то
// и «сколько до него хопов» — сведения о прошлом, а не о сети.
uint8_t sensorHopsFresh(int idx) {
    if (idx < 0 || idx >= sensorDeviceDiscCount) return 0xFF;
    if (sensorHops[idx] == 0xFF || sensorHopsMs[idx] == 0) return 0xFF;
    if (millis() - sensorHopsMs[idx] > SENSOR_OFFLINE_MS) return 0xFF;
    return sensorHops[idx];
}

SensorHello lastHello;
int lastSensorIdx = -1;
bool lastSensorCameOnline = false;

// Запись отправителя: найти или завести. Реестр полон — вытесняем того, кто дольше всех не
// выходил в эфир: он либо снят, либо всё равно не на связи, а живому узлу место нужнее.
// Отказ («реестр полон») означал бы, что узел сверх последнего просто не появляется ни на
// странице, ни в MQTT — без единой строки в журнале.
static int sensorSlot(const String& name) {
    for (int i = 0; i < sensorDeviceDiscCount; i++)
        if (sensorDeviceDisc[i] == name) return i;

    int idx;
    if (sensorDeviceDiscCount < SENSOR_DEV_CACHE_MAX) {
        idx = sensorDeviceDiscCount++;
    } else {
        idx = 0;
        for (int i = 1; i < sensorDeviceDiscCount; i++)
            if ((long)(sensorLastActive[i] - sensorLastActive[idx]) < 0) idx = i;
        slog("[REG] реестр узлов полон (%d): '%s' вытеснен узлом '%s'\n",
             (int)SENSOR_DEV_CACHE_MAX, sensorDeviceDisc[idx].c_str(), name.c_str());
    }
    // Слот может быть переиспользован, поэтому чистим ВСЕ поля узла, а не только те, у
    // которых нет разумного нуля: версия и окружение прошлого жильца иначе показывались бы
    // как версия нового.
    sensorDeviceDisc[idx] = name;
    sensorDiscPublished[idx] = false;
    sensorPosPublished[idx] = false;
    sensorBattery[idx] = -1;
    sensorHops[idx] = 0xFF;      // пока не услышали — не «напрямую», а «неизвестно»
    sensorViaSup[idx] = 0;
    sensorViaRssi[idx] = 0;
    sensorHopsMs[idx] = 0;
    sensorOnlineNow[idx] = false;
    sensorLastActive[idx] = millis();
    sensorFwVersion[idx] = "";
    sensorEnv[idx] = "";
    sensorRssi[idx] = 0;
    return idx;
}

void sensorRegistryNote() {
    const int idx = sensorSlot(lastSender);
    lastSensorIdx = idx;

    sensorLastActive[idx] = millis();
    if (lastRxViaSupport) {
        // Кадр принёс прошивальщик по сети: и RSSI, и хопы в нём — ЕГО измерения, от его
        // антенны. Свои этим не затираем — иначе узел, который мы слышим напрямую, после
        // одного форварда выглядел бы неслышимым, и прошить его сами мы бы не взялись.
        const int si = supportFind(lastRxSupport);
        sensorViaSup[idx] = (si >= 0) ? (uint8_t)(si + 1) : 0xFF;
        sensorViaRssi[idx] = lastRSSI;
    } else {
        sensorRssi[idx] = lastRSSI;
        sensorHops[idx] = lastHopCount;
        sensorHopsMs[idx] = millis();
    }
    lastSensorCameOnline = !sensorOnlineNow[idx];
    sensorOnlineNow[idx] = true;

    // heartbeat "hello:<версия>:<заряд %>:<напряжение>:<окружение>[:<широта>:<долгота>]".
    // "-" = поля нет; полей может быть меньше — координаты шлют только узлы с приёмником и
    // только когда решение есть, цикл сам остановится на конце строки. Узлы постарше шлют
    // просто "hello".
    //
    // Кода платы в heartbeat больше нет: окружение и так называет плату, причём точнее
    // (сенсор и компаньон живут на одной h43). Узел со старой прошивкой шлёт его пятым
    // полем, и здесь оно прочтётся как окружение — такой узел нужно обновить один раз
    // вручную, дальше он снова понятен.
    lastHello = SensorHello();
    lastHello.isHello = (lastMessage == SENSOR_MSG_HELLO ||
                         lastMessage.startsWith(SENSOR_MSG_HELLO ":"));
    if (lastHello.isHello) {
        String rest = lastMessage.substring(strlen(SENSOR_MSG_HELLO) + 1);
        String* fields[] = { &lastHello.ver, &lastHello.batPct, &lastHello.batVolt,
                             &lastHello.env, &lastHello.lat, &lastHello.lon };
        for (int i = 0; i < 6 && rest.length() > 0; i++) {
            int p = rest.indexOf(':');
            *fields[i] = (p < 0) ? rest : rest.substring(0, p);
            rest = (p < 0) ? String() : rest.substring(p + 1);
            if (*fields[i] == "-") *fields[i] = "";
        }
    }
    if (lastHello.ver.length() > 0)    sensorFwVersion[idx] = lastHello.ver;
    if (lastHello.env.length() > 0)    sensorEnv[idx] = lastHello.env;
    if (lastHello.batPct.length() > 0) sensorBattery[idx] = lastHello.batPct.toInt();
}

// Версия образа, которую узел называл последний раз. Поиск только по существующим записям:
// sensorSlot() не годится — он заводит запись, и чтение версии от узла, которого в реестре
// нет, создавало бы фантом на странице и в MQTT, а в полном реестре ещё и вытесняло бы
// кого-то живого. Нужно медленной сессии: она начинается, не зная, что было у цели, и
// сравнивает это с тем, что услышала после.
String sensorVersionOf(const String& name) {
    for (int i = 0; i < sensorDeviceDiscCount; i++)
        if (sensorDeviceDisc[i] == name) return sensorFwVersion[i];
    return String();
}

#endif // FEATURE_MESH_OTA_SENDER || FEATURE_MQTT
