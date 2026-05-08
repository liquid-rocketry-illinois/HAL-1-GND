//
// Created by dyrel on 3/7/2026.
//

#include "stm32c0xx_hal.h"
#include <cstdint>

#ifndef KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H
#define KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H

// CFG STARTING BITS
#define COMMAND_BYTE_WRITE_CFG_SAVE_FLASH 0xC0
#define COMMAND_BYTE_READ_CFG 0xC1
#define COMMAND_BYTE_WRITE_CFG_NOSAVE_FLASH 0xC2
#define COMMAND_BYTE_RESET_MODULE 0xCF


// General usage enums

typedef enum {
    TRANS,
    WOR,
    CONFIG,
    OFF
} EBYTE_MODE;

typedef enum
{
    E22_OK                = 0,
    E22_ERR_TIMEOUT       = -1,
    E22_ERR_UART          = -2,
    E22_ERR_BUSY          = -3,
    E22_ERR_INVALID_PARAM = -4,
    E22_ERR_NOT_INITIALIZED = -5,
    E22_ERR_DATA_VERIFICATION   = -6
} E22_Status;

/**
 * The general format for everything under here, describing the bits of registers,
 * follows this:
 *
 * <Register number>_<Bit in register>_<Function>
 *
 * This allows for easy reconstruction
 */


// REG0

typedef enum
{
    E22_UART_BAUD_1200    = 0b000 << 5,
    E22_UART_BAUD_2400    = 0b001 << 5,
    E22_UART_BAUD_4800    = 0b010 << 5,
    E22_UART_BAUD_9600    = 0b011 << 5,
    E22_UART_BAUD_19200   = 0b100 << 5,
    E22_UART_BAUD_38400   = 0b101 << 5,
    E22_UART_BAUD_57600   = 0b110 << 5,
    E22_UART_BAUD_115200  = 0b111 << 5
} R0_765_E22_UART_BAUD;

typedef enum
{
    MODE_8N1 = 0b00 << 3,
    MODE_8O1 = 0b01 << 3,
    MODE_8E1 = 0b10 << 3,
} R0_43_SERIAL_PORT_PARITY_BIT;

typedef enum
{
    E22_AIR_RATE_2_4K   = 0b010,  // 2.4 kbps
    E22_AIR_RATE_4_8K   = 0b011,  // 4.8 kbps
    E22_AIR_RATE_9_6K   = 0b100,  // 9.6 kbps
    E22_AIR_RATE_19_2K  = 0b101,   // 19.2 kbps
    E22_AIR_RATE_38_4K  = 0b110,  // 38.4 kbps
    E22_AIR_RATE_62_5K  = 0b111  // 62.5 kbps
} R0_210_E22_AIR_DATA_RATE;


// REG1

typedef enum
{
    BYTES_240 = 0b00 << 6,
    BYTES_128 = 0b01 << 6,
    BYTES_64  = 0b10 << 6,
    BYTES_32  = 0b11 << 6
} R1_76_SUB_PACKET_SETTING;

#define R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_ENABLE    0b1 << 5
#define R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE   0b0 << 5

// Bits 4 and 3 are reserved by the radio and are unusable

#define R1_2_SOFTWARE_MODE_SWITCHING_ON 0b1 << 2
#define R1_2_SOFTWARE_MODE_SWITCHING_OFF 0b0 << 2 // Always use this, M0 and M1 are already controlled via GPIO

typedef enum
{
    E22_TX_POWER_22DBM = 0b00,
    E22_TX_POWER_17DBM = 0b01,
    E22_TX_POWER_13DBM = 0b10,
    E22_TX_POWER_10DBM = 0b11
} R1_10_E22_TX_POWER;


// REG2
// CH902 = 902 MHz, CH903 = 903 MHz, etc....
typedef enum {
    CH902 = 52,
    CH903,
    CH904,
    CH905,
    CH906,
    CH907,
    CH908,
    CH909,
    CH910,
    CH911,
    CH912,
    CH913,
    CH914,
    CH915,
    CH916,
    CH917,
    CH918,
    CH919,
    CH920,
    CH921,
    CH922,
    CH923,
    CH924,
    CH925,
    CH926,
    CH927,
    CH928
} R2_E22Channel915;


// REG3

#define R3_7_RSSI_BYTE_ENABLE               0b0 << 7
#define R3_7_RSSI_BYTE_DISABLE              0b1 << 7
#define R3_6_TRANSFER_METHOD_TRANSPARENT    0b0 << 6
#define R3_6_TRANSFER_METHOD_FIXED_POINT    0b1 << 6
#define R3_5_REPEATER_OFF                   0b0 << 5
#define R3_5_REPEATER_ON                    0b1 << 5
#define R3_4_LBT_ENABLED                    0b1 << 4
#define R3_4_LBT_DISABLED                   0b0 << 4
#define R3_3_WOR_MODE_RECIEVER              0b0 << 3
#define R3_3_WOR_MODE_TRANSMIT              0b1 << 3

typedef enum
{
    _500_ms     = 0b000,
    _1000_ms    = 0b001,
    _1500_ms    = 0b010,
    _2000_ms    = 0b011,
    _2500_ms    = 0b100,
    _3000_ms    = 0b101,
    _3500_ms    = 0b110,
    _4000_ms    = 0b111,
} R3_210_WOR_CYCLE_TIME; // See datasheet pg 18 for cycle time calculation


// Packet structure

typedef struct
{
    uint8_t ADDH;
    uint8_t ADDL;
    uint8_t NETID;
    uint8_t REG0;
    uint8_t REG1;
    uint8_t REG2;
    uint8_t REG3;

    UART_HandleTypeDef *huart;

    GPIO_TypeDef *E22_M0_PORT;
    uint16_t E22_M0_PIN;

    GPIO_TypeDef *E22_M1_PORT;
    uint16_t E22_M1_PIN;

    GPIO_TypeDef *E22_AUX_PORT;
    uint16_t E22_AUX_PIN;
} config_e22_900t22s;

#endif //KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H