#include "config.h"

// ======= REDE / MQTT =======
const char WIFI_SSID[]   = "Siot_9";
const char WIFI_PASS[]   = "info@134";

const char MQTT_HOST[]   = "172.16.9.20";
const uint16_t MQTT_PORT = 1883;
const char MQTT_USER[]   = "cai";
const char MQTT_PASS[]   = "Th1ago@Eletr0";

// Tópicos
const char TOPIC_CMD[]    = "docksala/ir/cmd";
const char TOPIC_ACK[]    = "docksala/ir/ack";
const char TOPIC_STATUS[] = "docksala/ir/sender/status";
const char TOPIC_ECHO[]   = "docksala/ir/echo";

// ======= ESP-NOW =======
const uint8_t HUB_MAC[6] = { 0x24,0x6F,0x28,0xAA,0xBB,0xCC };

const uint8_t  ESPNOW_CHANNEL         = 1;
const uint8_t  ESPNOW_MAX_RETRY       = 5;
const uint32_t ESPNOW_ACK_TIMEOUT_MS  = 150;

// ======= NEXTION =======
const uint32_t NEXTION_BAUD  = 9600;
const uint8_t  NX_PAGE       = 0;
const uint8_t  NX_ID_AC1_ON     = 1;
const uint8_t  NX_ID_AC1_OFF    = 2;
const uint8_t  NX_ID_PROJ_PWR   = 3;
const uint8_t  NX_ID_PROJ_FREEZ = 4;

// ======= BOTÕES (teste) =======
const uint8_t  PIN_BTN_AC1_ON     = 23;
const uint8_t  PIN_BTN_AC1_OFF    = 22;
const uint8_t  PIN_BTN_PROJ_PWR   = 21;
const uint8_t  PIN_BTN_PROJ_FREEZ = 19;
const uint32_t DEBOUNCE_MS        = 35;

// ======= GERAL =======
const uint32_t SERIAL_BAUD = 115200;
