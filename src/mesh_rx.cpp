#include "config.h"
#include "globals.h"
#include "crypto.h"
#include <esp_random.h>
#include "radio.h"
#include "mesh.h"
#include "ota.h"

// ===== Разбор входящего пакета =====
// Выделено из mesh.cpp: распознавание типа пакета, проверка адресата, расшифровка
// группового и личного сообщения, служебные команды сенсорного канала и ответы на них.

// Формат ответа как в bot.py: "hops:direct" либо "hops:N, route:aa → bb → cc"
void buildPingReply(char* out, size_t outlen, const uint8_t* path, uint8_t hop_count, uint8_t path_hash_size) {
    if (hop_count == 0) {
        snprintf(out, outlen, "hops:direct");
        return;
    }
    size_t pos = snprintf(out, outlen, "hops:%u, route:", hop_count);
    for (int h = 0; h < hop_count; h++) {
        size_t need = (h > 0 ? 4 : 0) + 2 * path_hash_size + 1;
        if (pos + need > outlen) break;
        if (h > 0) { out[pos++] = ' '; out[pos++] = 0xE2; out[pos++] = 0x86; out[pos++] = 0x92; }  // " →"
        for (int b = 0; b < path_hash_size; b++) {
            pos += snprintf(out + pos, outlen - pos, "%02x", path[h * path_hash_size + b]);
        }
    }
}

// Подтверждение доставки стоит ВЫШЕ #ifndef SENSOR_NODE намеренно: личку получает и
// сенсор, и подтверждать доставку обязан он тоже. Внутри блока функция была не видна
// там, где нужна, и сборка падала на неразрешённом вызове.
// ===== Подтверждение доставки личного сообщения =====
// Копия поведения оригинального MeshCore (BaseChatMesh::onPeerDataRecv): приняв личку, узел
// отвечает пакетом PAYLOAD_TYPE_ACK с хэшем принятого, и по нему отправитель показывает
// «доставлено». Без этого ответа сообщение у отправителя навсегда остаётся неподтверждённым —
// именно так наши узлы и выглядели для чужих клиентов.
//
// Хэш считается ровно как там: sha256 по открытому тексту до конца текста ([время 4][тип 1]
// [текст]) И публичному ключу ОТПРАВИТЕЛЯ, обрезанный до четырёх байт. Пятый байт — байт
// расширенной попытки из того же открытого текста, шестой случайный: они не участвуют в
// сверке (принимающая сторона читает первые четыре), а нужны, чтобы хэш самого пакета
// подтверждения не повторялся и не отбрасывался дедупом как дубликат.
static void dmAckSend(const uint8_t* plain, int plainLen, const uint8_t* peerPub,
                      uint8_t srcHash, bool viaFlood, uint8_t inPathLen,
                      const uint8_t* inPath) {
    if (plainLen <= 5 || peerPub == NULL) return;
    int textLen = 0;
    while (5 + textLen < plainLen && plain[5 + textLen]) textLen++;

    uint8_t ack[6];
    sha256Trunc(ack, 4, plain, 5 + textLen, peerPub, 32);
    const int attemptAt = 5 + textLen + 1;
    ack[4] = (attemptAt < plainLen) ? plain[attemptAt] : 0;
    ack[5] = (uint8_t)random(0, 256);

    // Пришло флудом — возвращаем отправителю дорогу до нас, а подтверждение кладём
    // довеском: так делает оригинал, и это один пакет вместо двух. Отправитель по этому
    // маршруту и покажет «обратный маршрут» в приложении, и сможет звать нас не флудом.
    // Пришло не флудом — путь ему уже известен, и хватит голого подтверждения.
    uint8_t frame[GROUP_TEXT_MAX + 32];
    int f = 0;
    if (viaFlood) {
        f = buildPathReturnFrame(srcHash, peerPub, inPathLen, inPath,
                                 PAYLOAD_TYPE_ACK, ack, (int)sizeof(ack),
                                 frame, (int)sizeof(frame));
    }
    if (f <= 0) {
        f = 0;
        frame[f++] = (uint8_t)((PAYLOAD_TYPE_ACK << 2) | ROUTE_TYPE_FLOOD);
        frame[f++] = PATH_LEN_INIT;      // путь достроят ретрансляторы
        memcpy(frame + f, ack, sizeof(ack)); f += sizeof(ack);
    }

    // Через очередь с задержкой, а не сразу: отправитель ещё доканчивает свои копии и, пока
    // передаёт, нас не слышит (см. ACK_DELAY_* в config.h).
    const unsigned long delayMs = random(ACK_DELAY_MIN_MS, ACK_DELAY_MAX_MS);
    if (meshTxQueueFrame(frame, f, delayMs)) {
        ackSentCount++;
        Serial.printf("[DM] подтверждение %02X%02X%02X%02X уйдёт через %lu мс\n",
                      ack[0], ack[1], ack[2], ack[3], delayMs);
    } else {
        ackQueueFull++;
        Serial.printf("[DM] очередь занята — подтверждение не отправлено (всего %lu)\n",
                      (unsigned long)ackQueueFull);
    }
}

#ifndef SENSOR_NODE
// Ответ на пинг и на личку уходит не сразу: пока отправитель доканчивает свои повторы, он
// нас не слышит (радио полудуплексное), а при одинаковой у всех паузе несколько узлов
// отвечают разом и глушат друг друга. Раньше эта пауза была обычным delay() прямо здесь,
// в разборе пакета: до 3.5 с координатор не обслуживал ни радио, ни MQTT, ни страницу, ни
// сессию прошивки — и лавина direct-пакетов держала его в этом состоянии сколь угодно
// долго. Теперь ответ откладывается заявкой, а отправляет его главный цикл.
static struct {
    unsigned long dueMs;        // 0 — заявки нет
    bool dm;                    // отвечаем в личку, а не в канал
    uint8_t dmSrc;              // адресат лички (его короткий хэш)
    int chIdx;                  // канал для группового ответа
    bool viaSupport;            // запрос пришёл через прошивальщика («вторые уши»)
    char viaName[CFG_NAME_MAX + 1];   // и через какого именно: ответ уйдёт тем же путём
    char text[100];
} pendingReply;

static void scheduleReply(bool dm, uint8_t dmSrc, int chIdx, const char* text) {
    // Заявка ровно одна. Ответить на лавину пакетов мы всё равно не сможем — эфир один, —
    // а очередь ответов заняла бы его надолго и превратилась бы в ту же остановку цикла.
    if (pendingReply.dueMs != 0) {
        replyDropped++;
        Serial.printf("[PING] предыдущий ответ ещё не ушёл — этот пропускаем (всего %lu)\n",
                      (unsigned long)replyDropped);
        return;
    }
    pendingReply.dm = dm;
    pendingReply.dmSrc = dmSrc;
    pendingReply.chIdx = chIdx;
    // Ответ на пинг, пришедший через прошивальщика («вторые уши»), должен уйти туда же:
    // узел, которого слышит только прошивальщик, ответ с радио координатора не услышит.
    pendingReply.viaSupport = lastRxViaSupport;
    strlcpy(pendingReply.viaName, lastRxSupport.c_str(), sizeof(pendingReply.viaName));
    strlcpy(pendingReply.text, text, sizeof(pendingReply.text));
    pendingReply.dueMs = millis() + random(PING_REPLY_DELAY_MIN_MS, PING_REPLY_DELAY_MAX_MS);
    if (pendingReply.dueMs == 0) pendingReply.dueMs = 1;   // 0 занято признаком «нет заявки»
}


void meshReplyTick() {
    if (pendingReply.dueMs == 0) return;
    if ((long)(millis() - pendingReply.dueMs) < 0) return;
    // Идёт медленная прошивка — ждём тишины. Наш ответ поверх чанка потерял бы и себя, и
    // чанк: эфир общий и полудуплексный. Заявку не отменяем, а сдвигаем — проверка связи
    // ответит позже, зато ответит, а прошивка не начнёт заново из-за нас.
    if (otaSlowAirMs != 0 && millis() - otaSlowAirMs < OTA_SLOW_AIR_BUSY_MS) {
        pendingReply.dueMs = millis() + OTA_SLOW_AIR_BUSY_MS;
        if (pendingReply.dueMs == 0) pendingReply.dueMs = 1;
        replyDeferred++;
        return;
    }
    pendingReply.dueMs = 0;
    uint8_t frame[256];

    // === Личное сообщение: ответ уходит В ЛИЧКУ (TXT_MSG), а не в канал ===
    if (pendingReply.dm) {
        // Ключ ищем сейчас, а не при постановке заявки: за время паузы адверт узла мог
        // как раз прийти.
        uint8_t* peerPub = findPeerPub(pendingReply.dmSrc);
        if (peerPub == NULL) {
            dmNoPubkey++;
            Serial.printf("[DM] pubkey <%02X> неизвестен (нет advert) — ответ не отправлен "
                          "(всего %lu: advert узла до нас не дошёл)\n",
                          pendingReply.dmSrc, (unsigned long)dmNoPubkey);
            return;
        }
        int dl = buildPrivateTextFrame(pendingReply.dmSrc, peerPub, pendingReply.text,
                                       frame, sizeof(frame));
        if (dl > 0) {
            Serial.printf("\n[TX DM] to <%02X>: %s (%dB, флудом)\n",
                          pendingReply.dmSrc, pendingReply.text, dl);
            if (pendingReply.viaSupport &&
                mcRelayFrameToSupport(pendingReply.viaName, frame, dl)) {
                // Ответ ушёл через прошивальщика; наше эхо, вернувшееся от него по радио,
                // не переиздавать (обычно это делает floodSend через markOwnFrameSeen).
                markOwnFrameSeen(frame, dl);
            } else {
                floodSend(-1, frame, dl);
            }
        }
        return;
    }

    int f = buildGroupFrameFlood(pendingReply.chIdx, pendingReply.text, frame, sizeof(frame));
    if (f > 0) {
        Serial.printf("\n[TX] %s: %s: %s (%dB, флудом)\n", channels[pendingReply.chIdx].name,
                      cfg.name.c_str(), pendingReply.text, f);
        // Пинг из-за «вторых ушей»: кадр ответа уходит прошивальщику, тот передаёт его
        // со своего радио, где слышно отправителя. Хук вернёт false, если прошивальщика
        // нет или сеть молчит, — тогда падаем на обычный локальный флуд.
        if (pendingReply.viaSupport &&
            mcRelayFrameToSupport(pendingReply.viaName, frame, f)) {
            markOwnFrameSeen(frame, f);
        } else {
            floodSend(pendingReply.chIdx, frame, f);
        }
    }
}
#endif  // !SENSOR_NODE

// Метка времени из кадра — по часам ОТПРАВИТЕЛЯ, и они бывают неверны: наш узел после
// включения живёт по времени СБОРКИ прошивки, пока не услышит «time:» от координатора, а
// узел без синхронизации вовсе кладёт в кадр millis() («1970 год плюс пара дней»).
//
// Приложение показывает эту метку временем сообщения и по ней же ищет кадр в сыром журнале
// 0x88, чтобы нарисовать маршрут. Поэтому метку отправителя отдаём, только пока она близка
// к нашим часам (SENDER_TS_MAX_SKEW_S) — тогда верны и время, и маршрут. Разошлись сильнее —
// отдаём своё время приёма: сообщение недельной давности в переписке заметнее, чем
// ненарисованный маршрут у одного узла. См. SENDER_TS_MAX_SKEW_S в config.h.
static uint32_t senderTsOrOurs(uint32_t senderTs) {
    const uint32_t now = (uint32_t)time(NULL);
    if (now <= 1000000000) return senderTs;          // наши часы сами не выставлены
    const long skew = (long)senderTs - (long)now;
    if (senderTs > 1000000000 && skew > -(long)SENDER_TS_MAX_SKEW_S
                              && skew < (long)SENDER_TS_MAX_SKEW_S) {
        return senderTs;
    }
    Serial.printf("[RTC] часы отправителя разошлись с нашими на %ld с — приложению отдаём "
                  "время приёма (маршрут этого сообщения не покажется)\n", skew);
    return now;
}

bool parseMeshCorePacket(uint8_t* data, int len, const MeshRxMeta& meta) {
    if (len < 6) return false;

    uint8_t header = data[0];
    uint8_t payload_type = (header >> 2) & 0x0F;
    uint8_t route_type = header & 0x03;

    // ПОДТВЕРЖДЕНИЕ ДОСТАВКИ (0x03). Разбирается здесь, а не вместе с текстом: у него нет ни
    // адресата, ни канала — только хэш принятого сообщения, и сверяет его тот, кто это
    // сообщение отправлял. Раньше этот тип не разбирался вовсе: чужие подтверждения доходили
    // и молча отбрасывались, а значит «доставлено» мы не могли показать никогда.
    //
    // Сверяются ПЕРВЫЕ ЧЕТЫРЕ байта, как в оригинале (Mesh::onRecvPacket): пятый и шестой
    // нужны только для того, чтобы хэш самого пакета не повторялся.
    if (payload_type == PAYLOAD_TYPE_ACK) {
        int o = 1;
        if (route_type == ROUTE_TYPE_TRANSPORT_FLOOD || route_type == ROUTE_TYPE_TRANSPORT_DIRECT) o += 4;
        if (o < len) {
            uint8_t pl = data[o++];
            o += (pl & 0x3F) * (((pl >> 6) & 3) + 1);   // путь
            if (o + 4 <= len) {
                ackRecvCount++;
                Serial.printf("[ACK] подтверждение %02X%02X%02X%02X (всего %lu)\n",
                              data[o], data[o + 1], data[o + 2], data[o + 3],
                              (unsigned long)ackRecvCount);
                mcOnAckRecv(&data[o]);
            }
        }
        return false;
    }

    // GRP_TXT (0x05) — групповые, TXT_MSG (0x02) — личные, PATH (0x08) — возврат маршрута,
    // RESPONSE (0x01) — ответ ретранслятора на вход, запрос состояния или телеметрии.
    // У всех трёх, кроме группового, конверт один и тот же ([адресат][отправитель][MAC]
    // [шифр]) — отличается только содержимое, поэтому разбирает их одна ветка.
    if (payload_type != 0x05 && payload_type != PAYLOAD_TYPE_TXT_MSG
        && payload_type != PAYLOAD_TYPE_PATH
        && payload_type != PAYLOAD_TYPE_RESPONSE) {
        // ADVERT (0x04): кэшируем публичные ключи нод — без них не ответить
        // в личку (нужен полный pubkey для X25519). В кадре: [pub 32][ts 4][sig 64][app...].
        if (payload_type == 0x04) {
            int o = 1;
            if (route_type == 0x00 || route_type == 0x03) o += 4;
            if (o < len) {
                uint8_t pl = data[o++];
                const uint8_t* advPath = &data[o];          // хэши ретрансляторов, через которые пришёл адверт
                o += (pl & 0x3F) * (((pl >> 6) & 3) + 1);   // path bytes
                if (o + 32 + 4 + 64 <= len) {
                    uint8_t* pub = &data[o];
                    uint8_t* ts  = &data[o + 32];
                    uint8_t* sig = &data[o + 36];
                    uint8_t* app = &data[o + 100];
                    int applen = len - (o + 100);
                    if (applen < 0) applen = 0;
                    if (applen > 64) applen = 64;
                    uint8_t msg[32 + 4 + 64];
                    int mlen = 0;
                    memcpy(&msg[mlen], pub, 32); mlen += 32;
                    memcpy(&msg[mlen], ts, 4); mlen += 4;
                    memcpy(&msg[mlen], app, applen); mlen += applen;
                    if (Ed25519::verify(sig, pub, msg, mlen)) {
                        rememberPeerPub(pub[0], pub);
                        // Приложение строит список собеседников из адвертов: без этого
                        // добавить кого-либо в контакты попросту неоткуда.
                        companionOnAdvert(pub, app, applen, pl, advPath);
                        Serial.printf("[ADV] cached pubkey for <%02X>\n", pub[0]);
                    } else {
                        Serial.printf("[ADV] bad signature for <%02X>\n", pub[0]);
                    }
                }
            }
        }
        return false;
    }

    int offset = 1;
    if (route_type == 0x00 || route_type == 0x03) offset += 4;  // transport codes

    if (offset >= len) return false;
    uint8_t path_len = data[offset++];
    uint8_t path_hash_size = (path_len >> 6) + 1;
    uint8_t hop_count = path_len & 0x3F;
    int pathBytes = hop_count * path_hash_size;
    bool pathOk = hop_count > 0 && offset + pathBytes <= len;

    // путь (хэши ретрансляторов) для экрана и MQTT; длина пути приходит из эфира — пишем с проверкой
    lastPath[0] = 0;
    if (pathOk) {
        size_t pos = 0;
        for (int h = 0; h < hop_count && pos + 6 < sizeof(lastPath); h++) {
            const uint8_t* ph = &data[offset + h * path_hash_size];
            pos += (path_hash_size == 1)
                ? snprintf(lastPath + pos, sizeof(lastPath) - pos, "%02X ", ph[0])
                : snprintf(lastPath + pos, sizeof(lastPath) - pos, "%02X%02X ", ph[0], ph[1]);
        }
    }

    #ifndef SENSOR_NODE
    // копия пути для обратного маршрута в ответе на /ping
    uint8_t replyPath[MAX_REPLY_PATH];
    uint8_t replyHops = 0;
    if (pathOk && pathBytes <= MAX_REPLY_PATH) {
        memcpy(replyPath, &data[offset], pathBytes);
        replyHops = hop_count;
    }
    #endif
    // Путь, которым кадр пришёл, нужен дальше целиком: его мы вернём отправителю, чтобы он
    // узнал дорогу до нас. Берём указатель ДО сдвига смещения — после него путь уже позади.
    const uint8_t* inPath = &data[offset];
    const uint8_t inPathLen = path_len;        // байт разметки как пришёл

    offset += pathBytes;

    if (offset >= len) return false;

    // === Разбор тела пакета: ДМ (TXT_MSG) или групповое (GRP_TXT) ===
    bool personalDm = false;
    uint8_t dmSrc = 0;
    int chIdx = -1;
    // Метка времени ОТПРАВИТЕЛЯ канального сообщения (из открытого текста кадра). Приложение
    // сопоставляет сообщение из очереди с сырым журналом 0x88 именно по этой метке, поэтому
    // ей здесь и место — на уровне функции, рядом с остальным, что отдаётся приложению.
    uint32_t chSenderTs = 0;

    if (payload_type == PAYLOAD_TYPE_TXT_MSG || payload_type == PAYLOAD_TYPE_PATH
        || payload_type == PAYLOAD_TYPE_RESPONSE) {
        // Личное сообщение: payload = [dest_hash 1B][src_hash 1B][MAC 2B][cipher...].
        // Шифруется общим секретом X25519 — текст без ключей ноды не прочитать,
        // но dest_hash (первый байт) показывает, адресовано ли сообщение НАМ.
        if (offset + 2 > len) return false;
        uint8_t dest_hash = data[offset];
        dmSrc = data[offset + 1];
        if (dest_hash != ownShortHash) {
            Serial.printf("[DM] dest=%02X (не нам, наш=%02X) src=%02X, игнор\n", dest_hash, ownShortHash, dmSrc);
            return false;
        }
        personalDm = true;

        // В ДМ нет хэша канала — отвечаем в #connections (личный канал).
        chIdx = 1;
        if (chIdx >= numChannels) chIdx = 0;
        lastChannelIdx = chIdx;
        lastChannelName = "DM #connections";

        char senderHex[8];
        snprintf(senderHex, sizeof(senderHex), "<%02X>", dmSrc);
        lastSender = senderHex;
        // ТЕКСТ ЛИЧНОГО СООБЩЕНИЯ РАСШИФРОВЫВАЕТСЯ. Раньше здесь стояла константа
        // «(личное сообщение)»: кадр доходил, адресат проверялся, а содержимое не читалось
        // никогда. Снаружи это «сообщения не доходят» — сообщение в сети есть, на экране узла
        // и в приложении вместо него заглушка. Расшифровывать умеет только отправитель: общий
        // секрет X25519 считается из нашего приватного ключа и ПУБЛИЧНОГО ключа отправителя,
        // а он лежит в кэше — rememberPeerPub кладут туда на каждом адверте.
        //
        // Раскладка открытого текста та же, что у группового: [время 4][тип 1][текст][0],
        // и decryptGroupText пропускает ровно эти пять байт. Отличается только ключ.
        uint8_t* dmPeerPub = findPeerPub(dmSrc);
        if (dmPeerPub == NULL) {
            // Ключа нет — прочитать нечем. Это не «сообщение пустое», и молчать об этом нельзя:
            // advert отправителя до нас не дошёл, и повторная отправка ничего не изменит, пока
            // он снова не придёт.
            dmNoPubkey++;
            Serial.printf("[DM] pubkey <%02X> неизвестен (нет advert) — личное сообщение не "
                          "прочитано (всего %lu: advert узла до нас не дошёл)\n",
                          dmSrc, (unsigned long)dmNoPubkey);
            lastMessage = "";
        } else {
            uint8_t dmSecret[32];
            ed25519_key_exchange(dmSecret, dmPeerPub, bot_prv64);
            uint8_t* dmMac = &data[offset + 2];          // 2 байта MAC
            int dmCtLen = len - (offset + 4);
            dmCtLen &= ~15;                              // хвост не кратного 16 блока — мусор
            // Расшифровываем В БУФЕР, а не сразу в строку: подтверждение доставки
            // считается по открытому тексту ЦЕЛИКОМ ([время 4][тип 1][текст]), как в
            // оригинале, а не по одному тексту.
            uint8_t dmPlain[GROUP_TEXT_MAX];
            int dmPlainLen = dmCtLen > 0
                ? decryptGroupRaw(dmSecret, dmMac, &data[offset + 4], dmCtLen,
                                  dmPlain, sizeof(dmPlain)) : 0;
            String text = "";
            if (dmPlainLen > 5) {
                for (int i = 5; i < dmPlainLen && dmPlain[i]; i++) text += (char)dmPlain[i];
            }
            // ОТВЕТ РЕТРАНСЛЯТОРА (RESPONSE). Содержимое двоичное, а не текст:
            // [метка 4][тело]. Метка — та, с которой уходил запрос (вход, состояние,
            // телеметрия): по ней прошивка и узнаёт, на что этот ответ. Разбирать тело
            // ядру незачем — оно разное у каждого типа запроса и нужно только приложению.
            if (payload_type == PAYLOAD_TYPE_RESPONSE) {
                if (dmPlainLen < 4) {
                    Serial.println("[REQ] ответ короче метки — HMAC не совпал или кадр битый");
                } else {
                    uint32_t tag = 0;
                    memcpy(&tag, dmPlain, 4);
                    Serial.printf("[REQ] ответ от <%02X>: метка %08lX, тело %d Б\n",
                                  dmSrc, (unsigned long)tag, dmPlainLen - 4);
                    mcOnResponseRecv(dmSrc, dmPeerPub, dmPlain, dmPlainLen);
                }
                lastMessage = "";
                return false;        // в канале показывать нечего: это служебный кадр
            }

            // ВОЗВРАТ МАРШРУТА. Конверт тот же, содержимое другое:
            // [длина пути 1][путь][тип довеска 1][довесок]. Отправитель присылает его,
            // приняв нашу личку флудом, — так мы узнаём дорогу ДО НЕГО, и именно её
            // приложение показывает маршрутом. Довеском обычно идёт подтверждение
            // доставки: оригинал кладёт его сюда, чтобы не слать два пакета.
            if (payload_type == PAYLOAD_TYPE_PATH) {
                if (dmPlainLen < 1) {
                    Serial.println("[PATH] HMAC не совпал или возврат маршрута пуст");
                } else {
                    const uint8_t pl = dmPlain[0];
                    const uint8_t hsize = (pl >> 6) + 1;
                    const uint8_t hops = pl & 0x3F;
                    const int pathBytes = (int)hops * hsize;
                    if (1 + pathBytes > dmPlainLen) {
                        Serial.printf("[PATH] путь длиннее содержимого (%u хопов по %u Б) — "
                                      "кадр битый\n", hops, hsize);
                    } else {
                        int k = 1 + pathBytes;
                        const uint8_t extraType = (k < dmPlainLen) ? (dmPlain[k++] & 0x0F) : 0xFF;
                        const uint8_t* extra = &dmPlain[k];
                        const int extraLen = dmPlainLen - k;
                        Serial.printf("[PATH] маршрут до <%02X>: %u хопов по %u Б\n",
                                      dmSrc, hops, hsize);
                        mcOnPathRecv(dmSrc, pl, &dmPlain[1], extraType, extra, extraLen);
                        // Подтверждение приезжает довеском, чтобы не слать второй пакет.
                        if (extraType == PAYLOAD_TYPE_ACK && extraLen >= 4) {
                            ackRecvCount++;
                            mcOnAckRecv(extra);
                        } else if (extraType == PAYLOAD_TYPE_RESPONSE && extraLen >= 4) {
                            // Так приходит ответ на запрос, ушедший ФЛУДОМ: ретранслятор
                            // кладёт его довеском к возврату маршрута, чтобы не слать
                            // второй пакет (BaseChatMesh::onPeerDataRecv, ветка REQ).
                            // Без этой ветки ответ на первый запрос — пока путь ещё не
                            // известен — терялся бы всегда.
                            mcOnResponseRecv(dmSrc, dmPeerPub, extra, extraLen);
                        }
                    }
                }
                lastMessage = "";
                return false;        // показывать в канале нечего: это служебный кадр
            }

            if (text.length() == 0) {
                Serial.println("[DM] HMAC не совпал или личное сообщение пустое");
            } else {
                // Отправитель не присылает своё имя (в личке для этого нет поля), поэтому
                // отдаём то, что видно в кадре: короткий хэш. Имя узла приложение знает из
                // адверта и подставит само по контакту.
                lastMessage = text;
                dmAckSend(dmPlain, dmPlainLen, dmPeerPub, dmSrc,
                          route_type == ROUTE_TYPE_FLOOD, inPathLen, inPath);
                // ЛИЧКА УХОДИТ В ПРИЛОЖЕНИЕ КОНТАКТОМ, А НЕ КАНАЛОМ. Раньше она попадала
                // туда же, куда групповой текст (канал #connections), и приложение не
                // знало ни отправителя, ни маршрута: показать «маршрут сообщения» ему
                // неоткуда — в кадре канала нет ключа собеседника. Оригинал отдаёт личку
                // отдельным кодом с шестибайтовым началом ключа и байтом длины пути.
                uint32_t dmSenderTs = 0;
                memcpy(&dmSenderTs, dmPlain, 4);        // часы ОТПРАВИТЕЛЯ, как в оригинале
                // Тип текста лежит в старших шести битах байта флагов, в младших двух —
                // номер попытки отправки (оригинал: flags = data[4] >> 2).
                const uint8_t dmTxtType = dmPlain[4] >> 2;
                // SNR берём из метрики кадра, а не из lastSNR: тот обновляется ниже по
                // функции, и здесь в нём ещё лежит качество ПРЕДЫДУЩЕГО пакета.
                // Для пути, пришедшего не флудом, оригинал шлёт маркер 0xFF: хопов в таком
                // кадре нет, и показывать в приложении нечего.
                companionOnDirectText(dmPeerPub, text, meta.snr,
                                      route_type == ROUTE_TYPE_FLOOD ? path_len : 0xFF,
                                      senderTsOrOurs(dmSenderTs), dmTxtType);
            }
        }
        if (lastMessage.length() == 0) lastMessage = "(личное сообщение)";
    } else {
        uint8_t channel_hash = data[offset++];
        if (offset + 2 > len) return false;

        // ищем канал по хэшу
        for (int i = 0; i < numChannels; i++) {
            if (channel_hash == channels[i].hash) { chIdx = i; break; }
        }
        if (chIdx < 0) return false;
        lastChannelIdx = chIdx;
        lastChannelName = channels[chIdx].name;

        uint8_t* mac = &data[offset];         // 2 байта MAC
        uint8_t* ciphertext = &data[offset + 2];  // шифротекст после MAC
        int ciphertext_len = len - (offset + 2);

        // обрезаем до кратного 16
        int ciphertext_len_trunc = ciphertext_len & ~15;
        if (ciphertext_len_trunc <= 0) return false;
        // Расшифровываем В БУФЕР, а не сразу в строку: приложению нужна МЕТКА ВРЕМЕНИ
        // ОТПРАВИТЕЛЯ из первых четырёх байт открытого текста, а decryptGroupText их
        // выбрасывает вместе с байтом типа. По этой метке приложение сопоставляет
        // сообщение из очереди с сырым журналом 0x88 (кадр оттуда он расшифровывает сам) —
        // отсюда и маршрут. Со временем ПРИЁМА совпадение держалось только на часах,
        // идущих секунда в секунду: у своих узлов хопы показывались, у чужих нет.
        //
        // Раскладка та же, что у лички: [время 4][тип 1][текст][0].
        uint8_t chPlain[GROUP_TEXT_MAX];
        const int chPlainLen = decryptGroupRaw(channels[chIdx].secret, mac, ciphertext,
                                               ciphertext_len_trunc, chPlain, sizeof(chPlain));
        String message = "";
        if (chPlainLen > 5) {
            memcpy(&chSenderTs, chPlain, 4);
            for (int i = 5; i < chPlainLen && chPlain[i]; i++) message += (char)chPlain[i];
        }

        if (message.length() == 0) {
            Serial.println("[!] HMAC не совпал или пустое сообщение");
            return false;
        }

        int colonPos = message.indexOf(": ");
        if (colonPos > 0) {
            lastSender = message.substring(0, colonPos);
            lastMessage = message.substring(colonPos + 2);
        } else {
            lastSender = "?";
            lastMessage = message;
        }
    }

    packetCount++;
    // Флуд или направленный — разбивка нужна приложению: по ней видно, пользуется ли сеть
    // известными путями или всё ещё разливается по всем.
    if (route_type == ROUTE_TYPE_FLOOD || route_type == ROUTE_TYPE_TRANSPORT_FLOOD) recvFloodCount++;
    else recvDirectCount++;
    lastRSSI = meta.rssi;
    lastSNR = meta.snr;
    lastHopCount = hop_count;
    lastRxViaSupport = (meta.origin == MESH_RX_FORWARDED);
    lastRxSupport = lastRxViaSupport ? meta.via : String("");

    // убираем хвостовые пробелы/переносы (у некоторых клиентов "/ping \n")
    lastMessage.trim();

    // не обрабатываем собственные сообщения (эхо собственного флуда)
    if (lastSender == cfg.name) return false;

    // Показываем всё, что пришло в канал, включая служебный обмен узлов (hello, ping,
    // pong, time, ota, cfg): по нему видно жизнь сети, а отличить служебное от беседы
    // можно и по самому тексту.
    // Личное сообщение уже ушло в приложение отдельным кадром контакта (см. ветку DM
    // выше): второй раз, да ещё каналом, оно показалось бы дважды и в чужой переписке.
    if (chIdx >= 0 && !personalDm) {
        // Отправителя приложение достаёт из начала текста ("Имя: сообщение") — так
        // устроен формат группового сообщения в сети. Мы же разбирали строку на имя и
        // текст и отдавали только текст, поэтому в приложении сообщения были безымянными.
        String forApp = (lastSender.length() > 0 && lastSender != "?")
                      ? lastSender + ": " + lastMessage : lastMessage;
        // Приложению нужен сам байт длины пути, а не число хопов: в нём закодированы и
        // размер хэша ретранслятора, и счётчик, по нему приложение и показывает маршрут.
        // Для пакетов, пришедших не флудом (direct), как в оригинале шлём маркер 0xFF.
        uint8_t appPathLen = (route_type == 0x00 || route_type == 0x01) ? path_len : 0xFF;
        companionOnChannelText(chIdx, forApp, lastSNR, appPathLen,
                               senderTsOrOurs(chSenderTs));
    }

    Serial.printf("\n=== PACKET #%d (%s) ===\n", packetCount, lastChannelName.c_str());
    Serial.printf("From: %s\n", lastSender.c_str());
    Serial.printf("Msg: %s\n", lastMessage.c_str());
    Serial.printf("Route: %s (hops=%u)\n", lastPath[0] ? lastPath : "direct", hop_count);
    char r1[12], s1[12];
    Serial.printf("RSSI: %s dBm, SNR: %s dB\n", fmtFix(lastRSSI, 1, r1, sizeof(r1)),
                  fmtFix(lastSNR, 1, s1, sizeof(s1)));

    lastRxDisplay = millis();

    // Показать сообщение на экране. Во время mesh OTA экран не трогаем — иначе каждый
    // чанк мигает сообщением между кадрами прогресса (otaDrawProgress/otaSensorDraw).
    // Сенсор показывает только свой статус (drawIdleStatus), входящие пакеты не рисует.
    // Экран рисует прошивка (хук mcUiIncoming) по своему усмотрению.
    #ifndef SENSOR_NODE
    if (!otaFastMode) {
        mcUiIncoming(lastChannelName, lastSender, lastMessage, lastRSSI, lastSNR,
                     (int)hop_count, String(lastPath[0] ? lastPath : "direct"));
    }
    #endif

    // === СЕНСОРНЫЙ КАНАЛ: сообщение уходит в MQTT как отдельное устройство ===
    if (sensorChannelIdx >= 0 && chIdx == sensorChannelIdx) {
        // === MESH OTA: бот принимает ack, сенсор — чанки/управление ===
        // Чанк медленной прошивки слышен всем в канале, а не только его участникам. Отмечаем
        // это до разбора и независимо от признаков: по отметке любой узел понимает, что эфир
        // сейчас чужой, и не лезет со своим ответом.
        if (lastMessage.startsWith(OTA_SLOW_MSG_DATA)) otaSlowAirMs = millis();

        // ===== МЕДЛЕННЫЙ РЕЖИМ =====
        // Разбираем ДО общей ветки "ota:", потому что часть его сообщений начинается так же.
        // Чанки данных идут с отдельным префиксом: их много, и лишнее сравнение на каждом —
        // это работа на каждый принятый кадр всей сети.
        #if FEATURE_MESH_OTA_RECEIVER
        if (lastMessage.startsWith(OTA_SLOW_MSG_DATA)) {
            otaSlowRxData(lastMessage.substring(strlen(OTA_SLOW_MSG_DATA)));
            return true;
        }
        if (lastMessage.startsWith(OTA_SLOW_MSG_START)) {
            otaSlowRxStart(lastMessage.substring(strlen(OTA_SLOW_MSG_START)));
            return true;
        }
        #endif
        #if FEATURE_MESH_OTA_SENDER
        // Подтверждение слышат оба — и отправитель, и координатор. Отправитель двигает по
        // нему окно; у координатора, который сессию не ведёт, otaSlowOn ложно, и вызов
        // ничего не делает, кроме обновления того, что видно на странице.
        if (lastMessage.startsWith(OTA_SLOW_MSG_ACK)) {
            otaSlowOnAck((uint32_t)strtoul(lastMessage.c_str() + strlen(OTA_SLOW_MSG_ACK), NULL, 10));
            return true;
        }
        if (lastMessage.startsWith(OTA_SLOW_MSG_DONE)) {
            otaSlowDone(true, nullptr);
            return true;
        }
        if (lastMessage.startsWith(OTA_SLOW_MSG_FAIL)) {
            otaSlowDone(false, lastMessage.c_str() + strlen(OTA_SLOW_MSG_FAIL));
            return true;
        }
        #endif

        if (lastMessage.startsWith("ota:")) {
            // Ведущий сессии разбирает подтверждения, принимающий — чанки и управление.
            // Порядок важен: у прошивальщика есть и то, и другое, и он ведущий.
            #if FEATURE_MESH_OTA_SENDER
            otaHandleAck();
            #elif FEATURE_MESH_OTA_RECEIVER
            otaSensorHandle();
            #endif
            return true;
        }

        // === Настройка по радио: команда адресована конкретному узлу по имени ===
        if (lastMessage.startsWith("cfg:")) {
            #ifdef SENSOR_NODE
            String rest = lastMessage.substring(4);
            int p = rest.indexOf(':');
            if (p > 0 && rest.substring(0, p) == cfg.name) cfgHandleMeshCfg(rest.substring(p + 1));
            #elif FEATURE_WEB
            // ответы узла видно в журнале на странице, в MQTT их не публикуем
            slog("[CFG] %s: %s\n", lastSender.c_str(), lastMessage.c_str());
            #endif
            return true;
        }

        // === Проверка связи: сенсор шлёт ping:<номер>, координатор сразу отвечает
        //     pong:<номер>:<свой rssi>:<свой snr>, сенсор считает время обмена ===
        if (lastMessage.startsWith(SENSOR_MSG_PING)) {
            // Отвечают ВСЕ, кто может вести сессию прошивки, — и координатор, и
            // прошивальщик. Это решение, а не совпадение признаков: проверка связи отвечает
            // на вопрос «дошло ли моё сообщение до инфраструктуры», а у узла, поставленного
            // там, где координатора не слышно, единственный слышащий — прошивальщик.
            //
            // Два ответа не мешают друг другу: пауза перед ответом случайная
            // (PING_REPLY_DELAY_MIN/MAX_MS), поэтому в эфире они разъезжаются, а узел
            // засчитывает первый — второй отбрасывается проверкой pingSentMs != 0 на разборе
            // pong. RSSI и SNR в ответе — оценка ТОГО, кто ответил, и по ней видно, кто узел
            // слышит: это ровно то, что нужно знать при проверке связи.
            #if FEATURE_MESH_OTA_SENDER
            char reply[48];
            snprintf(reply, sizeof(reply), "%s%s:%d:%d", SENSOR_MSG_PONG,
                     lastMessage.c_str() + strlen(SENSOR_MSG_PING),
                     (int)lround(lastRSSI), (int)lround(lastSNR));
            slog("[PING] %s -> %s\n", lastSender.c_str(), reply);
            sensorSendMsg(reply, 0, 1);
            #endif
            return true;
        }
        if (lastMessage.startsWith(SENSOR_MSG_PONG)) {
            #ifdef SENSOR_NODE
            const char* p = lastMessage.c_str() + strlen(SENSOR_MSG_PONG);
            unsigned id = (unsigned)strtoul(p, NULL, 10);
            if (pingSentMs != 0 && id == pingId) {
                pingRttMs = millis() - pingSentMs;
                pingSentMs = 0;
                // Выборка режима: разброс времени ответа говорит о связи больше, чем одно
                // измерение. Второй ответ на тот же запрос (отвечают оба) сюда не попадёт —
                // pingSentMs уже обнулён.
                if (pingStatRecv < 0xFFFF) pingStatRecv++;
                // У хопов ноль — законное значение («напрямую»), поэтому первый ответ
                // задаёт обе границы, а не сравнивается с нулевой заготовкой.
                if (pingStatRecv == 1) {
                    pingRttMin = pingRttMax = pingRttMs;
                    pingHopsMin = pingHopsMax = lastHopCount;
                } else {
                    if (pingRttMs < pingRttMin) pingRttMin = pingRttMs;
                    if (pingRttMs > pingRttMax) pingRttMax = pingRttMs;
                    if (lastHopCount < pingHopsMin) pingHopsMin = lastHopCount;
                    if (lastHopCount > pingHopsMax) pingHopsMax = lastHopCount;
                }
                pingRttSum += pingRttMs;
                pingHopsSum += lastHopCount;
                pingRssi = lastRSSI;
                pingSnr = lastSNR;
                pingHops = lastHopCount;
                pingPeerRssi = 0;
                const char* c1 = strchr(p, ':');
                if (c1) pingPeerRssi = atoi(c1 + 1);
                pingFailed = false;
                pingShowUntil = millis() + PING_SHOW_MS;
                Serial.printf("[PING] ответ за %lu мс, хопов %u\n", pingRttMs, pingHops);
            }
            #endif
            return true;
        }

        // === Опрос со страницы OTA: каждый сенсор ответит hello:<версия> со случайной задержкой,
        //     чтобы ответы нескольких сенсоров не столкнулись в эфире ===
        if (lastMessage == SENSOR_MSG_HELLO_REQ) {
            #ifdef SENSOR_NODE
            // "hello всем": отвечает каждый услышавший. Свои сообщения сюда не доходят —
            // эхо собственного флуда отброшено выше по lastSender == cfg.name.
            //
            // Пауза от esp_random(), а не от Arduino random(): без зерна серия у всех узлов
            // одна и та же, и после одновременного опроса ответы легли бы в один слот.
            // esp_random() — аппаратный ГСЧ, у каждого узла своя пауза.
            sensorHelloDueMs = millis() + HELLO_REPLY_DELAY_MIN_MS +
                (esp_random() % (HELLO_REPLY_DELAY_MAX_MS - HELLO_REPLY_DELAY_MIN_MS));
            #endif
            return true;
        }

        // === Синхронизация времени: бот шлёт "time:<epoch>:<версия бота>" от NTP ===
        if (lastMessage.startsWith("time:")) {
            // Качество приёма запоминаем независимо от того, годен ли epoch: пакет всё
            // равно пришёл от координатора, а значит меряет связь именно с ним — по этим
            // числам узел и показывает качество сети.
            timeSyncMs = millis();
            timeSyncRssi = lastRSSI;
            timeSyncSnr = lastSNR;
            char* end = NULL;
            uint64_t epoch = (uint64_t)strtoull(lastMessage.c_str() + 5, &end, 10);
            #ifdef SENSOR_NODE
            // версия бота не совпала со своей — экран покажет звёздочку у версии
            if (end && *end == ':') fwVersionDiffers = strcmp(end + 1, FW_VERSION) != 0;
            #endif
            if (epoch > (uint64_t)BUILD_UNIX_TIME) {
                struct timeval tv;
                tv.tv_sec = (time_t)epoch;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                time_t local = (time_t)epoch + (time_t)cfg.tzOffset * 3600;
                struct tm tm_now;
                gmtime_r(&local, &tm_now);
                char tbuf[32];
                strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_now);
                Serial.printf("[RTC] SYNCED from channel: %s\n", tbuf);
            }
        }
        // Реестр — дело ядра и не зависит от того, есть ли куда публиковать: по нему
        // выбирается цель прошивки, и он же отвечает на вопрос «кого мы слышим».
        #if FEATURE_MESH_OTA_SENDER || FEATURE_MQTT
        sensorRegistryNote();
        #endif
        #if FEATURE_MESH_OTA_SENDER
        // Heartbeat разобран выше, в lastHello. Медленной сессии он важен как доказательство
        // применения образа: узел шлёт привет сразу после включения, то есть сразу после
        // перезагрузки в новую прошивку. Стоит рядом с разбором, а не в otaSlowTick,
        // потому что отсюда версия и берётся — и версия нужна вся, а не «изменилась ли».
        otaSlowOnHello();
        #endif
        #if FEATURE_MQTT
        bool snsPub = publishSensorMessage();
        mcUiSensorRx(snsPub, lastRSSI);
        #else
        Serial.printf("[SNS] %s: %s\n", lastSender.c_str(), lastMessage.c_str());
        #endif
        return true;
    }

    // === ОТВЕТ НА СООБЩЕНИЕ ===
    // Только для MQTT-бота. Сенсорный узел — пассивный: отвечает ТОЛЬКО кнопкой
    // ("button") и heartbeat ("hello"), пинг/DM он не обслуживает.
    #ifndef SENSOR_NODE
    // - Личное (TXT_MSG) с dest_hash == BOT_ID_HASH: отвечаем ВСЕГДА, как на /ping.
    // - Групповой GRP_TXT: на текст "/ping" в #connections либо на любое
    //   DIRECT-сообщение, адресованное устройству.
    bool isDirect = (route_type == 0x02 || route_type == 0x03);
    if (personalDm || (chIdx == 1 && lastMessage == "/ping") || isDirect) {
        char reply[100];
        buildPingReply(reply, sizeof(reply), replyPath, replyHops, path_hash_size);
        Serial.printf("[PING] reply: %s\n", reply);
        // Сам ответ уйдёт из главного цикла, когда истечёт пауза (см. meshReplyTick).
        scheduleReply(personalDm, dmSrc, chIdx, reply);
    }
    #endif  // !SENSOR_NODE (ответ на пинг/DM — только MQTT-бот)

    return true;
}

// Совместимая обёртка под старую сигнатуру: метрику берёт у самого радио.
bool parseMeshCorePacket(uint8_t* data, int len) {
    MeshRxMeta m;
    m.origin = MESH_RX_RADIO;
    m.rssi = radio.getRSSI();
    m.snr = radio.getSNR();
    return parseMeshCorePacket(data, len, m);
}

// Вход для кадров, пришедших не из radio_rx (например, «вторые уши» — форвард узла по
// WiFi): дедуп общий с радио-путём (checkAndMarkSeen), свежий кадр разбирается как обычно.
bool meshRxFrame(uint8_t* data, int len, const MeshRxMeta& meta) {
    if (checkAndMarkSeen(data, len)) {
        duplicateCount++;
        Serial.printf("[DUP] skipped (total dups=%lu)\n", duplicateCount);
        return false;
    }
    return parseMeshCorePacket(data, len, meta);
}

