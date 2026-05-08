#ifndef EBYTE_E22_900T22S_H
#define EBYTE_E22_900T22S_H

#include <cstdint>
#include <cstddef>
#include "Ebyte_E22_900T22S_defs.h"

int8_t init_e22_900t22s(config_e22_900t22s *cfg);

// Transmit raw data in transparent mode
int8_t transmit_e22_900t22s(uint8_t *data, size_t length);

// Transmit using fixed addressing mode
int8_t transmit_fixed_e22_900t22s(
uint16_t address,
uint8_t channel,
uint8_t *data,
size_t length);

// Receive data into user buffer.
// expected_payload_len must be sizeof(the struct being received).
// The driver reads exactly (5 + expected_payload_len + 2) bytes with a
// baud-rate-derived timeout plus 100 ms margin for wireless decode latency.
// SYNC/CRC validation is left to the caller (decodeData in Telemetry.cpp).
// Returns total bytes written to buffer or negative error (-1 HAL error, -2 timeout/short).
int16_t recieve_e22_900t22s(uint8_t *buffer, uint16_t expected_payload_len);

// Check if data is available from the module
inline volatile bool e22_data_ready = false;
bool e22_available(void);

// Switch from trans, WOR, config, and off modes
void changeMode(EBYTE_MODE mode);
void changeOpFreq_e22_900t22s(R2_E22Channel915 channel);
R2_E22Channel915 getOpFreq_e22_900t22s(void);

// Address config
void setAddress_e22_900t22s(uint16_t address);
uint16_t getAddress_e22_900t22s(void);

// Parameters
void setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE rate);
void setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD baud);
void setTxPower_e22_900t22s(R1_10_E22_TX_POWER power);

// Read configuration from module
int8_t readConfig_e22_900t22s(config_e22_900t22s *cfg);

// Write configuration to module
int8_t writeConfig_e22_900t22s( const config_e22_900t22s *cfg,
                                bool save_to_flash);

// Wait until module finishes operation
int8_t waitAux_e22_900t22s(uint32_t timeout_ms);

// Check if module is busy
bool e22_isBusy(void);

// rst
void reset_e22_900t22s(void);

// State return
bool e22_initialized(void);

#endif
