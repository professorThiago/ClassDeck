#pragma once
#include <Arduino.h>
#include <stdint.h>

// ==================== REDE / MQTT ====================
extern const char WIFI_SSID[];
extern const char WIFI_PASS[];

extern const char MQTT_HOST[];
extern const uint16_t MQTT_PORT;
extern const char MQTT_USER[];
extern const char MQTT_PASS[];

// Tópicos
extern const char TOPIC_CMD[];
extern const char TOPIC_ACK[];
extern const char TOPIC_STATUS[];
extern const char TOPIC_ECHO[];  // publica ações vindas da IHM

// ==================== ESP-NOW ====================
// MAC do hub (receptor). Coloque o MAC STA do ESP hub.
extern const uint8_t HUB_MAC[6];

extern const uint8_t  ESPNOW_CHANNEL;         // igual ao do hub
extern const uint8_t  ESPNOW_MAX_RETRY;
extern const uint32_t ESPNOW_ACK_TIMEOUT_MS;

// ==================== NEXTION ====================
// Pinos padrão do Serial2 no ESP32: RX2=16, TX2=17 (OK p/ Nextion)
extern const uint32_t NEXTION_BAUD;
// IDs dos botões (exemplo)
extern const uint8_t NX_PAGE;
extern const uint8_t NX_ID_AC1_ON;
extern const uint8_t NX_ID_AC1_OFF;
extern const uint8_t NX_ID_PROJ_PWR;
extern const uint8_t NX_ID_PROJ_FREEZ;

// ==================== BOTÕES FÍSICOS (teste) ====================
extern const uint8_t  PIN_BTN_AC1_ON;
extern const uint8_t  PIN_BTN_AC1_OFF;
extern const uint8_t  PIN_BTN_PROJ_PWR;
extern const uint8_t  PIN_BTN_PROJ_FREEZ;
extern const uint32_t DEBOUNCE_MS;

// ==================== GERAL ====================
extern const uint32_t SERIAL_BAUD;
