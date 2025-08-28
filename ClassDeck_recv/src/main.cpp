#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Fujitsu.h>   // precisa vir antes de config.h (usa ARRAH2E)

#include "ir_codes.h"
#include "config.h"

// =================== PROTOCOLO ===================

enum class Device : uint8_t { AC1=0, AC2=1, PROJ=2 };
enum class Action : uint8_t { Off=0, SetOn=1, Power=2, Freeze=3 };
enum class MsgType: uint8_t { Cmd=1, Ack=2 };

struct EspNowHdr {
  uint8_t type;  // MsgType
  uint8_t seq;   // sequência p/ correlacionar ACK
} __attribute__((packed));

struct EspNowCmd {
  EspNowHdr h;   // {type=Cmd, seq}
  uint8_t device; // 0..2
  uint8_t action; // 0..3
  uint8_t mode;   // p/ AC (consts Fujitsu)
  uint8_t temp;   // 16..30
  uint8_t fan;    // p/ AC (consts Fujitsu)
} __attribute__((packed));

struct EspNowAck {
  EspNowHdr h;   // {type=Ack, seq}
  uint8_t ok;    // 1=recebido
} __attribute__((packed));

struct Command {
  Device  device;
  Action  action;
  uint8_t mode;
  uint8_t temp;
  uint8_t fan;
};

// =================== INSTÂNCIAS / FILA ===================

namespace {
  IRFujitsuAC ac1(IR_LED_AC1);
  IRFujitsuAC ac2(IR_LED_AC2);
  IRsend      irProj(IR_LED_PROJ);

  QueueHandle_t qCmd = nullptr;
}

// =================== PROTÓTIPOS ===================

void initIrHardware();
void setupEspNow();
bool ensurePeer(const uint8_t mac[6]);
void sendAckTo(const uint8_t to[6], uint8_t seq, uint8_t ok);
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);

void execAC(IRFujitsuAC& ac, Action a, uint8_t mode, uint8_t temp, uint8_t fan);
void execProj(Action a);

void taskIr(void*);
bool queueSend(const Command& c);

// =================== SETUP / LOOP ===================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);

  Serial.println("\n[IR Hub - ESP-NOW (simples)]");

  initIrHardware();

  qCmd = xQueueCreate(QUEUE_LEN, sizeof(Command));
  if (!qCmd) {
    Serial.println("ERRO: xQueueCreate falhou");
    ESP.restart();
  }

  xTaskCreatePinnedToCore(taskIr, "taskIR",
                          TASK_IR_STACK, nullptr,
                          TASK_IR_PRIO,  nullptr,
                          TASK_IR_CORE);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  setupEspNow();

  Serial.printf("ESP-NOW pronto no canal %u\n", ESPNOW_CHANNEL);
}

void loop() {
  // Nada aqui: RX no callback, execução na taskIr()
}

// =================== IMPLEMENTAÇÕES ===================

void initIrHardware() {
  irProj.begin();

  ac1.begin();
  ac1.setModel(static_cast<fujitsu_ac_remote_model_t>(FUJITSU_MODEL_AC1));
  ac1.setId(FUJITSU_ID_AC1);

  ac2.begin();
  ac2.setModel(static_cast<fujitsu_ac_remote_model_t>(FUJITSU_MODEL_AC2));
  ac2.setId(FUJITSU_ID_AC2);
}

void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO: esp_now_init()");
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);
}

bool ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false; // versão simples sem criptografia

  return esp_now_add_peer(&p) == ESP_OK;
}

void sendAckTo(const uint8_t to[6], uint8_t seq, uint8_t ok) {
  if (!ensurePeer(to)) {
    Serial.println("ACK: falha ao adicionar peer");
    return;
  }
  EspNowAck ack{};
  ack.h.type = static_cast<uint8_t>(MsgType::Ack);
  ack.h.seq  = seq;
  ack.ok     = ok;

  esp_err_t e = esp_now_send(to, reinterpret_cast<uint8_t*>(&ack), sizeof(ack));
  if (e != ESP_OK) {
    Serial.printf("ACK: esp_now_send falhou (%d)\n", (int)e);
  }
}

bool queueSend(const Command& c) {
  return xQueueSend(qCmd, &c, 0) == pdTRUE; // não bloquear (callback)
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < (int)sizeof(EspNowHdr)) return;

  const EspNowHdr* hdr = reinterpret_cast<const EspNowHdr*>(data);

  if (hdr->type == static_cast<uint8_t>(MsgType::Cmd)) {
    if (len < (int)sizeof(EspNowCmd)) {
      Serial.printf("CMD curto (%d < %u)\n", len, (unsigned)sizeof(EspNowCmd));
      return;
    }

    EspNowCmd ec;
    memcpy(&ec, data, sizeof(EspNowCmd));

    Command c{
      ec.device==0 ? Device::AC1 : (ec.device==1 ? Device::AC2 : Device::PROJ),
      static_cast<Action>(ec.action),
      ec.mode, ec.temp, ec.fan
    };

    if (!queueSend(c)) {
      Serial.println("Fila cheia — comando descartado");
      if (ACK_ON_RECEIVE) sendAckTo(mac, hdr->seq, /*ok=*/0);
      return;
    }

    if (ACK_ON_RECEIVE) {
      sendAckTo(mac, hdr->seq, /*ok=*/1); // ACK de recebido
    }

  } else if (hdr->type == static_cast<uint8_t>(MsgType::Ack)) {
    Serial.printf("ACK recebido (seq=%u)\n", hdr->seq);
  }
}

void execAC(IRFujitsuAC& ac, Action a, uint8_t mode, uint8_t temp, uint8_t fan) {
  if (a == Action::Off) {
    ac.setPower(false);
    ac.send();
    return;
  }
  // Set/Liga
  ac.setPower(true);
  ac.setMode(mode);
  ac.setTemp(temp);
  ac.setFanSpeed(fan);
  ac.setSwing(kFujitsuAcSwingOff);
  ac.send();
}

void execProj(Action a) {
  if (a == Action::Power) {
    irProj.sendRaw(RAW_POWER,  RAW_POWER_LEN,  IR_KHZ);
  } else if (a == Action::Freeze) {
    irProj.sendRaw(RAW_FREEZE, RAW_FREEZE_LEN, IR_KHZ);
  }
}

void taskIr(void*) {
  Command cmd;
  for (;;) {
    if (xQueueReceive(qCmd, &cmd, portMAX_DELAY) == pdTRUE) {
      Serial.printf("IR -> dev=%u act=%u\n",
        static_cast<unsigned>(cmd.device),
        static_cast<unsigned>(cmd.action));

      switch (cmd.device) {
        case Device::AC1:  execAC(ac1, cmd.action, cmd.mode, cmd.temp, cmd.fan); break;
        case Device::AC2:  execAC(ac2, cmd.action, cmd.mode, cmd.temp, cmd.fan); break;
        case Device::PROJ: execProj(cmd.action); break;
      }

      vTaskDelay(pdMS_TO_TICKS(IR_GAP_MS));
    }
  }
}
