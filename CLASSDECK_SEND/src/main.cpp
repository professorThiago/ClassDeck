#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <strings.h>   // strcasecmp

#define nexSerial Serial2
#include <Nextion.h>   // ITEADLIB Nextion (via GitHub)

#include "config.h"

// ============ Protocolo (mesmo do hub) ============
enum class Device : uint8_t { AC1=0, AC2=1, PROJ=2 };
enum class Action : uint8_t { Off=0, SetOn=1, Power=2, Freeze=3 };
enum class MsgType: uint8_t { Cmd=1, Ack=2 };

struct EspNowHdr { uint8_t type; uint8_t seq; } __attribute__((packed));
struct EspNowCmd {
  EspNowHdr h; uint8_t device; uint8_t action; uint8_t mode; uint8_t temp; uint8_t fan;
} __attribute__((packed));
struct EspNowAck { EspNowHdr h; uint8_t ok; } __attribute__((packed));

struct Command {
  Device  device;
  Action  action;
  uint8_t mode;
  uint8_t temp;
  uint8_t fan;
};

// ============ Estado global ============
WiFiClient wifi;
PubSubClient mqtt(wifi);

char g_clientId[40] = {};        // "esp-ir-tx-xxxxxxxxxxxx"
volatile uint8_t g_seq = 0;

// pendência de ACK (um por vez p/ simplicidade)
struct Pending {
  bool waiting; uint8_t seq; uint8_t retries; uint32_t sentAt; EspNowCmd last;
  Pending():waiting(false),seq(0),retries(0),sentAt(0){ memset(&last,0,sizeof(last)); }
} g_pending;

// ============ Nextion ============
NexButton nx_ac1_on    (NX_PAGE, NX_ID_AC1_ON,     "bAC1On");
NexButton nx_ac1_off   (NX_PAGE, NX_ID_AC1_OFF,    "bAC1Off");
NexButton nx_proj_pwr  (NX_PAGE, NX_ID_PROJ_PWR,   "bProjPwr");
NexButton nx_proj_free (NX_PAGE, NX_ID_PROJ_FREEZ, "bProjFrz");
NexTouch* nex_listen_list[] = {
  &nx_ac1_on, &nx_ac1_off, &nx_proj_pwr, &nx_proj_free, nullptr
};

// ============ Prototipagem ============
void wifiConnect();
void mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int len);

uint8_t mapMode(const char* s);
uint8_t mapFan (const char* s);
Device  parseDevice(const char* s);
Action  parseAction(const char* s);

void publishStatus(const char* s);
void publishAck(uint8_t seq, uint8_t ok);
void publishEcho(const Command& c, uint8_t seq, const char* origin);

bool  sendEspNow(const Command& c);
void  onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void  onEspNowSent(const uint8_t* mac, esp_now_send_status_t status);
void  handleAckTimeout();

void handleNextion();
void nx_on_ac1_on(void*);
void nx_on_ac1_off(void*);
void nx_on_proj_pwr(void*);
void nx_on_proj_free(void*);

void setupButtons();
void handleButtons();

// ============ Setup / Loop ============
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println("\n[IR Transmitter - ESP-NOW + MQTT + Nextion]");

  // Nextion: inicialize a UART da HMI antes do nexInit()
  nexSerial.begin(NEXTION_BAUD);
  nexInit();
  nx_ac1_on.attachPop(nx_on_ac1_on,    &nx_ac1_on);
  nx_ac1_off.attachPop(nx_on_ac1_off,  &nx_ac1_off);
  nx_proj_pwr.attachPop(nx_on_proj_pwr,&nx_proj_pwr);
  nx_proj_free.attachPop(nx_on_proj_free,&nx_proj_free);

  setupButtons();

  // Wi-Fi
  wifiConnect();

  // MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  // ESP-NOW
  WiFi.mode(WIFI_STA); // necessário
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO: esp_now_init()");
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, HUB_MAC, 6);
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) {
    Serial.println("ERRO: esp_now_add_peer()");
  }

  // clientId = "esp-ir-tx-"+mac
  uint8_t mac[6]; WiFi.macAddress(mac);
  snprintf(g_clientId, sizeof(g_clientId), "esp-ir-tx-%02X%02X%02X%02X%02X%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  publishStatus("boot");
}

void loop() {
  if (!WiFi.isConnected()) wifiConnect();
  if (!mqtt.connected())   mqttConnect();
  mqtt.loop();

  nexLoop(nex_listen_list);   // eventos da IHM
  handleButtons();            // testes físicos
  handleAckTimeout();         // retries de ESP-NOW pendente
}

// ============ Wi-Fi / MQTT ============
void wifiConnect() {
  if (WiFi.isConnected()) return;
  Serial.printf("WiFi: conectando a %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<12000) { delay(200); Serial.print("."); }
  Serial.println();
  if (WiFi.isConnected()) {
    Serial.printf("WiFi OK: %s  RSSI %d dBm  IP %s\n",
                  WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi FAIL");
  }
}

void mqttConnect() {
  while (!mqtt.connected()) {
    Serial.print("MQTT: conectando ... ");
    if (mqtt.connect(g_clientId, MQTT_USER, MQTT_PASS,
                     TOPIC_STATUS, 1, true, "offline")) {
      Serial.println("OK");
      mqtt.publish(TOPIC_STATUS, "online", true);
      mqtt.subscribe(TOPIC_CMD, 1);
      Serial.printf("MQTT: inscrito em %s\n", TOPIC_CMD);
    } else {
      Serial.printf("falhou rc=%d; retry em 2s\n", mqtt.state());
      delay(2000);
    }
  }
}

// payload JSON -> Command -> envia via ESP-NOW
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  (void)topic;
  StaticJsonDocument<256> doc;  // ArduinoJson v6
  DeserializationError err = deserializeJson(doc, payload, len);
  if (err) { Serial.printf("MQTT JSON erro: %s\n", err.c_str()); return; }

  const char* origin = doc["origin"] | "";
  if (strcmp(origin, g_clientId)==0) return; // evita loop

  const char* devS   = doc["device"] | "ac1";
  const char* actS   = doc["action"] | "set_on";
  const char* modeS  = doc["mode"]   | "cool";
  uint8_t     temp   = doc["temp"]   | 24;
  const char* fanS   = doc["fan"]    | "auto";

  Command c;
  c.device = parseDevice(devS);
  c.action = parseAction(actS);
  c.mode   = mapMode(modeS);
  c.temp   = temp;
  c.fan    = mapFan(fanS);

  sendEspNow(c);
}

// ============ Mapeamentos ============
uint8_t mapMode(const char* s) {
  if (!strcasecmp(s,"auto")) return 0; // kFujitsuAcModeAuto
  if (!strcasecmp(s,"cool")) return 1; // kFujitsuAcModeCool
  if (!strcasecmp(s,"dry"))  return 2;
  if (!strcasecmp(s,"fan"))  return 3;
  if (!strcasecmp(s,"heat")) return 4;
  return 1;
}
uint8_t mapFan(const char* s) {
  if (!strcasecmp(s,"auto"))  return 0; // kFujitsuAcFanAuto
  if (!strcasecmp(s,"quiet")) return 1;
  if (!strcasecmp(s,"low"))   return 2;
  if (!strcasecmp(s,"med"))   return 3;
  if (!strcasecmp(s,"high"))  return 4;
  return 0;
}
Device parseDevice(const char* s) {
  if (!strcasecmp(s,"ac1"))  return Device::AC1;
  if (!strcasecmp(s,"ac2"))  return Device::AC2;
  return Device::PROJ;
}
Action parseAction(const char* s) {
  if (!strcasecmp(s,"off"))     return Action::Off;
  if (!strcasecmp(s,"set_on"))  return Action::SetOn;
  if (!strcasecmp(s,"power"))   return Action::Power;
  if (!strcasecmp(s,"freeze"))  return Action::Freeze;
  return Action::SetOn;
}

// ============ Publicações MQTT ============
void publishStatus(const char* s) {
  mqtt.publish(TOPIC_STATUS, s, true);
}
void publishAck(uint8_t seq, uint8_t ok) {
  StaticJsonDocument<96> doc;
  doc["seq"] = seq;
  doc["ok"]  = ok;
  char buf[96];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_ACK, buf, n);
}
void publishEcho(const Command& c, uint8_t seq, const char* origin) {
  StaticJsonDocument<192> doc;
  doc["origin"] = origin;
  doc["seq"]    = seq;
  doc["device"] = (c.device==Device::AC1? "ac1" : c.device==Device::AC2? "ac2" : "proj");
  switch (c.action) {
    case Action::Off:    doc["action"]="off"; break;
    case Action::SetOn:  doc["action"]="set_on"; break;
    case Action::Power:  doc["action"]="power"; break;
    case Action::Freeze: doc["action"]="freeze"; break;
  }
  doc["mode"] = c.mode;
  doc["temp"] = c.temp;
  doc["fan"]  = c.fan;
  char buf[192];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_ECHO, buf, n);
}

// ============ ESP-NOW ============
bool sendEspNow(const Command& c) {
  if (g_pending.waiting) {
    Serial.println("ESP-NOW: aguardando ACK anterior; comando ignorado");
    return false;
  }
  EspNowCmd cmd{};
  cmd.h.type = static_cast<uint8_t>(MsgType::Cmd);
  cmd.h.seq  = ++g_seq;
  cmd.device = static_cast<uint8_t>(c.device);
  cmd.action = static_cast<uint8_t>(c.action);
  cmd.mode   = c.mode;
  cmd.temp   = c.temp;
  cmd.fan    = c.fan;

  esp_err_t e = esp_now_send(HUB_MAC, reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
  if (e != ESP_OK) {
    Serial.printf("ESP-NOW send falhou (%d)\n", (int)e);
    return false;
  }

  // Guarda pendência
  g_pending.waiting = true;
  g_pending.seq     = cmd.h.seq;
  g_pending.retries = 0;
  g_pending.sentAt  = millis();
  g_pending.last    = cmd;

  Serial.printf("ESP-NOW -> seq=%u dev=%u act=%u\n", cmd.h.seq, cmd.device, cmd.action);
  return true;
}

void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("esp_now_send status=%d\n", status);
  }
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len < (int)sizeof(EspNowHdr)) return;
  const EspNowHdr* hdr = reinterpret_cast<const EspNowHdr*>(data);
  if (hdr->type != static_cast<uint8_t>(MsgType::Ack)) return;
  if (len < (int)sizeof(EspNowAck)) return;

  EspNowAck ack; memcpy(&ack, data, sizeof(ack));
  if (g_pending.waiting && ack.h.seq == g_pending.seq) {
    Serial.printf("ACK seq=%u ok=%u\n", ack.h.seq, ack.ok);
    g_pending.waiting = false;
    publishAck(ack.h.seq, ack.ok);
  }
}

void handleAckTimeout() {
  if (!g_pending.waiting) return;
  if (millis() - g_pending.sentAt < ESPNOW_ACK_TIMEOUT_MS) return;

  if (g_pending.retries >= ESPNOW_MAX_RETRY) {
    Serial.printf("ACK timeout seq=%u (max retries)\n", g_pending.seq);
    publishAck(g_pending.seq, 0);
    g_pending.waiting = false;
    return;
  }

  g_pending.retries++;
  g_pending.sentAt = millis();
  esp_err_t e = esp_now_send(HUB_MAC, reinterpret_cast<uint8_t*>(&g_pending.last), sizeof(g_pending.last));
  Serial.printf("Retry #%u seq=%u (%s)\n", g_pending.retries, g_pending.seq,
                e==ESP_OK? "sent":"err");
}

// ============ Nextion handlers ============
void handleNextion() {
  nexLoop(nex_listen_list);
}

void nx_on_ac1_on(void*) {
  Command c{Device::AC1, Action::SetOn, mapMode("cool"), 24, mapFan("quiet")};
  if (sendEspNow(c)) publishEcho(c, g_seq, g_clientId); // eco no MQTT quando vier da IHM
}
void nx_on_ac1_off(void*) {
  Command c{Device::AC1, Action::Off, 0, 0, 0};
  if (sendEspNow(c)) publishEcho(c, g_seq, g_clientId);
}
void nx_on_proj_pwr(void*) {
  Command c{Device::PROJ, Action::Power, 0, 0, 0};
  if (sendEspNow(c)) publishEcho(c, g_seq, g_clientId);
}
void nx_on_proj_free(void*) {
  Command c{Device::PROJ, Action::Freeze, 0, 0, 0};
  if (sendEspNow(c)) publishEcho(c, g_seq, g_clientId);
}

// ============ Botões físicos ============
struct Btn {
  uint8_t pin; bool last; uint32_t tEdge;
  Btn(): pin(0), last(true), tEdge(0) {}
  explicit Btn(uint8_t p): pin(p), last(true), tEdge(0) {}
};

Btn b_ac1_on     (PIN_BTN_AC1_ON);
Btn b_ac1_off    (PIN_BTN_AC1_OFF);
Btn b_proj_pwr   (PIN_BTN_PROJ_PWR);
Btn b_proj_freez (PIN_BTN_PROJ_FREEZ);

void setupButtons() {
  pinMode(b_ac1_on.pin,     INPUT_PULLUP);
  pinMode(b_ac1_off.pin,    INPUT_PULLUP);
  pinMode(b_proj_pwr.pin,   INPUT_PULLUP);
  pinMode(b_proj_freez.pin, INPUT_PULLUP);
}

static inline bool fell(bool now, bool &last, uint32_t &tEdge) {
  if (last && !now && millis()-tEdge > DEBOUNCE_MS) { last=now; tEdge=millis(); return true; }
  last = now; return false;
}

void handleButtons() {
  bool n;

  n = digitalRead(b_ac1_on.pin);
  if (fell(n, b_ac1_on.last, b_ac1_on.tEdge)) {
    Command c{Device::AC1, Action::SetOn, mapMode("cool"), 24, mapFan("quiet")};
    sendEspNow(c);
  }

  n = digitalRead(b_ac1_off.pin);
  if (fell(n, b_ac1_off.last, b_ac1_off.tEdge)) {
    Command c{Device::AC1, Action::Off, 0,0,0};
    sendEspNow(c);
  }

  n = digitalRead(b_proj_pwr.pin);
  if (fell(n, b_proj_pwr.last, b_proj_pwr.tEdge)) {
    Command c{Device::PROJ, Action::Power, 0,0,0};
    sendEspNow(c);
  }

  n = digitalRead(b_proj_freez.pin);
  if (fell(n, b_proj_freez.last, b_proj_freez.tEdge)) {
    Command c{Device::PROJ, Action::Freeze, 0,0,0};
    sendEspNow(c);
  }
}