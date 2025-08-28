#pragma once
#include <stdint.h>

// =================== PARÂMETROS GERAIS ===================

inline constexpr uint32_t SERIAL_BAUD      = 115200;

// IR
inline constexpr uint8_t  IR_LED_AC1       = 4;    // evite 6..11 no ESP32 WROOM
inline constexpr uint8_t  IR_LED_AC2       = 5;
inline constexpr uint8_t  IR_LED_PROJ      = 18;
inline constexpr uint16_t IR_KHZ           = 38;   // frequência típica de IR
inline constexpr uint32_t IR_GAP_MS        = 180;  // espaçamento entre envios IR

// Modelos/IDs dos ACs (biblioteca IRremoteESP8266 - Fujitsu)
inline constexpr uint16_t FUJITSU_MODEL_AC1 = ARRAH2E;
inline constexpr uint16_t FUJITSU_MODEL_AC2 = ARRAH2E;
inline constexpr uint8_t  FUJITSU_ID_AC1    = 0;
inline constexpr uint8_t  FUJITSU_ID_AC2    = 0;

// ESP-NOW
inline constexpr uint8_t  ESPNOW_CHANNEL   = 1;    // todos os nós devem usar o mesmo canal

// Fila / Task
inline constexpr uint8_t  QUEUE_LEN        = 12;
inline constexpr uint32_t TASK_IR_STACK    = 4096;
inline constexpr UBaseType_t TASK_IR_PRIO  = 1;
inline constexpr BaseType_t  TASK_IR_CORE  = 1;     // 0 ou 1

// ACK
// true  = envia ACK assim que RECEBE (menor latência)
// false = (não usado nesta versão simples)
inline constexpr bool      ACK_ON_RECEIVE  = true;
