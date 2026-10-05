#include "config.h"
#include "globals.h"
#include "mesh.h"
#include "ota.h"
#include "appconfig.h"
#include "mc_platform.h"   // mcUiTick / mcCompanionTick: платформенная часть
#include "button.h"        // кнопка: логика в ядре, пин и прерывание — за хуками
#include "sensor_tasks.h"

// ===== Периодические задачи узла =====
// Файл переехал сюда из lib/meshcore обеих прошивок: он лежал двумя копиями и разошёлся, хотя
// платформенными в нём были ровно три вызова — экран, кнопка и телефонное приложение. Они
// остались в прошивках и зовутся через хуки mc_platform.h, а расписание теперь одно на все
// платы. Проверку «копий больше нет» держит sensor_tasks_in_core_test.
//
// Выделено из главного цикла: сторож прошивки по радио, ожидание ответа на проверку
// связи, откат несохранённых настроек, обслуживание телефонного приложения, гашение
// экрана, частота процессора и расписание heartbeat. Всё это работает по времени и
// ничего не ждёт, поэтому собрано в одном месте отдельно от приёма радио.

#if FEATURE_SENSOR

void sensorTasksTick() {
    mcUiTick();        // экран гаснет в простое, будит длинное нажатие кнопки
    // Полную частоту держим на всю прошивку, а не только пока открыт быстрый канал.
    // Кадры идут каждые 8 мс, но куда важнее конец сессии: запись во флеш и проверка
    // образа на пониженной частоте — самое подозрительное место, а узел дважды отвергал
    // уже полностью принятый и заведомо исправный образ.
    bool wantFastCpu = otaFastMode || otaActive;
    static bool cpuFast = false;
    if (wantFastCpu != cpuFast) {
        cpuFast = wantFastCpu;
        setCpuFrequencyMhz(cpuFast ? CPU_MHZ_FAST : CPU_MHZ_IDLE);
        Serial.printf("[PWR] частота процессора %d МГц\n", cpuFast ? CPU_MHZ_FAST : CPU_MHZ_IDLE);
    }
    otaSensorTick();   // mesh OTA: сторожевое время — при зависании прерываем сессию
    otaSlowRxTick();   // медленный приём: своя очередь ответа и сторож тишины в канале
    sensorPingTick();  // не дождались ответа на проверку связи — показать это
    cfgPendingTick();  // правки настроек по радио без "save" откатываются перезагрузкой
    cfgReplyTick();    // части ответа на "cfg get" — из очереди, а не с delay() в разборе
    mcCompanionTick(); // кадры от приложения разбираем здесь, а не в колбэке BLE
    // Sensor node: button = trigger ("button"), hello = heartbeat раз в N минут
    // Во время OTA mesh-отправки подавляем: радио слушает raw-чанки на быстром канале.
    static unsigned long lastHeartbeat = 0;
    static bool bootHelloSent = false;
    // Во время медленного приёма узел тоже молчит: heartbeat пришёлся бы на фазу данных и
    // сбил бы чанк, который ведущий передаёт в этот момент. Отметиться он успеет после —
    // в канале по очереди говорит кто-то один.
    if (!otaActive && !otaSlowOn && cfgReady()) {
        if (!bootHelloSent) {
            bootHelloSent = true;
            sensorSendHello();   // стартовый hello сразу после включения
            // не упреждать первый периодический heartbeat после boot-привета
            lastHeartbeat = millis();
        } else if (sensorHelloDueMs != 0 && millis() >= sensorHelloDueMs) {
            // бот попросил отметиться (кнопка «Опросить» на странице OTA)
            sensorHelloDueMs = 0;
            lastHeartbeat = millis();
            sensorSendHello();
        } else if (millis() - lastHeartbeat >= SENSOR_HEARTBEAT_MS) {
            lastHeartbeat = millis();
            sensorSendHello();
        }
    }
    // Кнопка зовётся напрямую, а не хуком: её логика с 5 октября 2026 живёт в ядре
    // (src/button.cpp), и за хуками остались только пин, уровень и переключение экрана.
    // Хук mcButtonTick здесь был и удалён — он стал пустой пересылкой из прошивки в ядро, а
    // хук «на всякий случай» стоит ровно столько же обязательств, сколько настоящий. Платы без
    // кнопки отсекает признак сборки, как и раньше.
    #if FEATURE_BUTTON
    buttonTick();      // счёт нажатий и переключение экрана, без блокировки
    #endif
}

#endif // FEATURE_SENSOR
