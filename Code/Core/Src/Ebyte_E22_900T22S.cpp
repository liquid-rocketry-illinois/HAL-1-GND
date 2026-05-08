#include "Ebyte_E22_900T22S.h"
#include "stm32c0xx_hal.h"
#include <cstring>
#include "stm32c0xx_ll_usart.h"
#include "main.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_uart.h"

extern UART_HandleTypeDef huart1;

static config_e22_900t22s e22_cfg;
static bool initialized = false;

/* ================= AUX HANDLING ================= */

// Use aux to detect transmitted data via wireless, or if
// data won't go through UART, or if modules are still
// initializing/etc.+
static bool auxHigh(void)
{
    return HAL_GPIO_ReadPin(
        e22_cfg.E22_AUX_PORT,
        e22_cfg.E22_AUX_PIN
    ) == GPIO_PIN_SET;
}

// true if aux low -> means e22 is sending data.
// false if ready for input
bool e22_isBusy(void)
{
    return !auxHigh();
}

int8_t waitAux_e22_900t22s(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while(!auxHigh())
    {
        
        if((HAL_GetTick() - start) > timeout_ms) {
            return E22_ERR_TIMEOUT;
        }

        HAL_Delay(1);
    }
    return E22_OK;
}

/* ================= MODE CONTROL ================= */

void changeMode(EBYTE_MODE mode)
{
    // Module will finish its current task (e.g. transmission) even if
    // the mode is switched. No delay is needed
    switch(mode)
    {
        case TRANS:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case WOR:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case CONFIG:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_SET);
            break;

        case OFF:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_SET);
            break;
    }

    // Wait until the module has processed the mode switch
    waitAux_e22_900t22s(200);
}

/* ================= INITIALIZATION ================= */

int8_t init_e22_900t22s(config_e22_900t22s *cfg)
{
    if (cfg == NULL)
        return E22_ERR_INVALID_PARAM;

    /* copy desired configuration locally */
    e22_cfg = *cfg;

    /* CONFIG mode on the E22 always uses fixed 9600 baud regardless of REG0.
     * Force the MCU UART to 9600 now so config read/write succeeds even if
     * CubeMX regenerated usart.c at a different rate, or a prior init already
     * bumped it to 115200.  It will be raised to the target rate at the end. */
    e22_cfg.huart->Init.BaudRate = 9600;
    HAL_UART_Init(e22_cfg.huart);

    /* ensure mutex exists */
    // if (e22_mutex == NULL)
    //     e22_mutex = xSemaphoreCreateMutex();
    //
    // if (e22_mutex == NULL)
    //     return E22_ERR_BUSY;

    // active-low
    HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_SET);

    /* ensure radio is ready */
    waitAux_e22_900t22s(5000);

    /* switch to configuration mode */
    changeMode(CONFIG);
    // for safety
    HAL_Delay(500);
    waitAux_e22_900t22s(1000);

    /* read current radio configuration */
    config_e22_900t22s current_cfg = {};

    writeConfig_e22_900t22s(cfg, true);

    if (readConfig_e22_900t22s(&current_cfg) != E22_OK)
        return E22_ERR_UART;

    /* compare relevant fields */
    bool config_matches =
        (current_cfg.ADDH == cfg->ADDH)     &&
        (current_cfg.ADDL == cfg->ADDL)     &&
        (current_cfg.NETID == cfg->NETID)   &&
        (current_cfg.REG0 == cfg->REG0)     &&
        (current_cfg.REG1 == cfg->REG1)     &&
        (current_cfg.REG2 == cfg->REG2)     &&
        (current_cfg.REG3 == cfg->REG3)    ;

    /* only write if configuration differs */
    if (!config_matches)
    {
        if (writeConfig_e22_900t22s(cfg, true) != E22_OK)
            return E22_ERR_UART;

        if (waitAux_e22_900t22s(1000) != E22_OK)
            return E22_ERR_TIMEOUT;
    }

    if (readConfig_e22_900t22s(&current_cfg) != E22_OK)
        return E22_ERR_UART;
    config_matches =
        (current_cfg.ADDH == cfg->ADDH)     &&
        (current_cfg.ADDL == cfg->ADDL)     &&
        (current_cfg.NETID == cfg->NETID)   &&
        (current_cfg.REG0 == cfg->REG0)     &&
        (current_cfg.REG1 == cfg->REG1)     &&
        (current_cfg.REG2 == cfg->REG2)     &&
        (current_cfg.REG3 == cfg->REG3);
    if (!config_matches)
        return 1; // ensure that the config set in module is the config given to it

    /* Raise MCU UART to match the E22's configured TRANS-mode baud rate. */
    e22_cfg.huart->Init.BaudRate = 38400;
    HAL_UART_Init(e22_cfg.huart);

    /* return to normal transmit mode */
    changeMode(TRANS);

    if (waitAux_e22_900t22s(1000) != E22_OK)
        return E22_ERR_TIMEOUT;
    HAL_Delay(500); // Allow to initialize properly

    initialized = true;

    return E22_OK;
}

// Keep this
bool e22_initialized(void)
{
    return initialized;
}

/* ================= UART HELPERS ================= */

static int8_t uartWrite(uint8_t *data, uint16_t len)
{
    if(HAL_UART_Transmit(
        &huart1,
        data,
        len,
        5000) != HAL_OK)
        return E22_ERR_UART;

    return E22_OK;
}

static int8_t uartRead(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(
        &huart1, // e22_cfg.huart,
        data,
        len,
        5000); // Make a generous amount of time

    while (!auxHigh()) {} // wait for read to finish in case
    if(status != HAL_OK) return E22_ERR_UART;

    return E22_OK;
}

/* ================= CONFIGURATION ================= */

// save_to_flash overwrites the factory default settings.
// Set to TRUE when deploying finalized code!
int8_t writeConfig_e22_900t22s(
    const config_e22_900t22s *cfg,
    bool save_to_flash)
{
    uint8_t frame[10];

    // Head command byte
    frame[0] = save_to_flash ?
        COMMAND_BYTE_WRITE_CFG_SAVE_FLASH :
        COMMAND_BYTE_WRITE_CFG_NOSAVE_FLASH;
    frame[1] = 0x00;
    frame[2] = 0x07;

    // REG3 is for more advanced functions and is not included
    frame[3] = cfg->ADDH;
    frame[4] = cfg->ADDL;
    frame[5] = cfg->NETID;
    frame[6] = cfg->REG0;
    frame[7] = cfg->REG1;
    frame[8] = cfg->REG2;
    frame[9] = cfg->REG3;

    //xSemaphoreTake(e22_mutex, portMAX_DELAY);

    changeMode(CONFIG);
    // Ensure that config can be written
    HAL_Delay(400);

    // Construct and write config commands

    if(uartWrite(frame, 10) != E22_OK)
    {
        //xSemaphoreGive(e22_mutex);
        return E22_ERR_UART;
    }

    /* The E22 echoes back the written registers as confirmation:
     * [C1 00 07 ADDH ADDL NETID REG0 REG1 REG2 REG3] (10 bytes).
     * Must be consumed here — it sits in the MCU UART buffer regardless of
     * when changeMode(TRANS) is called, and shifts the next uartRead() by 1 byte. */
    uint8_t _echo[10];
    uartRead(_echo, 10);

    changeMode(TRANS);

    e22_cfg = *cfg;
    //xSemaphoreGive(e22_mutex);
    return E22_OK;
}

int8_t readConfig_e22_900t22s(config_e22_900t22s *cfg)
{
    uint8_t cmd[3];
    cmd[0] = COMMAND_BYTE_READ_CFG;
    cmd[1] = 0x00; // Start from ADDH
    cmd[2] = 0x07; // Read all necessary registers
    uint8_t resp[10] = {0};
    // resp: {c1, 00, 07, addh, addl, netid, reg0, reg1, reg2, reg3}

    //xSemaphoreTake(e22_mutex, portMAX_DELAY);

    changeMode(CONFIG);

    // Process mode switch in case aux pin logic is messed
    HAL_Delay(500);
    waitAux_e22_900t22s(1000); // shouldn't need this but just in case

    uartWrite(cmd,3);
    int8_t rslt = uartRead(resp,10);
    if(rslt != E22_OK)
    {
        //xSemaphoreGive(e22_mutex);
        return E22_ERR_UART;
    }

    // first three values are just repetition of sent data
    if (resp[0] != cmd[0] ||
        resp[1] != cmd[1] ||
        resp[2] != cmd[2])
        return E22_ERR_DATA_VERIFICATION;

    cfg->ADDH   = resp[3];
    cfg->ADDL   = resp[4];
    cfg->NETID  = resp[5];
    cfg->REG0   = resp[6];
    cfg->REG1   = resp[7];
    cfg->REG2   = resp[8];
    cfg->REG3   = resp[9];

    //xSemaphoreGive(e22_mutex);

    changeMode(TRANS);

    return E22_OK;
}

/* ================= DATA TRANSMISSION ================= */

int8_t transmit_e22_900t22s(uint8_t *data, size_t length)
{
    if(!initialized)
        return E22_ERR_NOT_INITIALIZED;

    int8_t status = E22_OK;

    //xSemaphoreTake(e22_mutex, portMAX_DELAY);

    waitAux_e22_900t22s(200);
    if (auxHigh()) // double check. dont want to skip sent bytes
        status = uartWrite(data, length);
    waitAux_e22_900t22s(200);

    //xSemaphoreGive(e22_mutex);

    return status;
}

/* ================= FIXED TRANSMISSION ================= */

int8_t transmit_fixed_e22_900t22s(
    uint16_t address,
    uint8_t channel,
    uint8_t *data,
    size_t length)
{
    if(!initialized)
        return E22_ERR_NOT_INITIALIZED;

    uint8_t frame[3 + length];

    frame[0] = (address >> 8);
    frame[1] = (address & 0xFF);
    frame[2] = channel;

    memcpy(&frame[3], data, length);

    return transmit_e22_900t22s(frame, length + 3);
}

/* ================= RECEIVE ================= */

bool e22_available()
{
    if(e22_data_ready)
    {
        e22_data_ready = false;
        return true;
    }
    return false;
}

/*
 * Single-phase receive sized to the exact expected packet.
 *
 * Reads exactly (5 + expected_payload_len + 2) bytes — the full frame:
 *   [SYNC1][SYNC2][len][seq_lo][seq_hi][payload x len][crc_lo][crc_hi]
 *
 * Timeout is calculated from the actual UART baud rate so it is tight
 * regardless of whether the baud is 9600 or 115200, with a 100 ms margin
 * to cover E22 wireless decode latency before UART output begins.
 *
 * SYNC validation and length/CRC checking are left to decodeData() in
 * Telemetry.cpp — this function only guarantees a full buffer fill.
 *
 * Returns total bytes placed in buffer or < 0 on error:
 *   -1  HAL_ERROR or HAL_BUSY
 *   -2  timeout with fewer than 7 bytes received (unusable)
 */
int16_t recieve_e22_900t22s(uint8_t *buffer, uint16_t expected_payload_len)
{
    const uint32_t baud     = e22_cfg.huart->Init.BaudRate;
    const uint16_t total    = 5u + expected_payload_len + 2u;   // hdr + payload + CRC

    /* Time to clock out the full frame at the configured baud, plus 100 ms
     * margin for the E22 to finish wireless decode before UART output begins. */
    /* 300 ms margin: gives the remote side time to finish its own TX, process
     * the received packet, and start sending a response before we time out.
     * At 9600 bps air rate a 96-byte telemetry packet takes ~80 ms over RF,
     * so the round-trip (GND TX → FC RX+process+TX → GND RX) is ~160 ms. */
    uint32_t timeout_ms = ((total * 10u * 1000u) / baud) + 300u;

    HAL_StatusTypeDef s = HAL_UART_Receive(e22_cfg.huart, buffer, total, timeout_ms);

    if (s == HAL_OK)
        return (int16_t)total;

    if (s == HAL_TIMEOUT) {
        uint16_t received = total - e22_cfg.huart->RxXferCount;
        if (received < 7u)
            return -2;   // not enough bytes for any valid packet
        return (int16_t)received;
    }

    /* HAL_ERROR or HAL_BUSY */
    return -1;
}

/* ================= ADDRESS ================= */

void setAddress_e22_900t22s(uint16_t address)
{
    e22_cfg.ADDH = (address >> 8);
    e22_cfg.ADDL = (address & 0xFF);

    writeConfig_e22_900t22s(&e22_cfg,false);
}

uint16_t getAddress_e22_900t22s(void)
{
    return (e22_cfg.ADDH << 8) | e22_cfg.ADDL;
}

/* ================= CHANNEL ================= */

void changeOpFreq_e22_900t22s(R2_E22Channel915 channel)
{
    e22_cfg.REG2 = channel;
    writeConfig_e22_900t22s(&e22_cfg,false);
}

R2_E22Channel915 getOpFreq_e22_900t22s(void)
{
    return (R2_E22Channel915)e22_cfg.REG2;
}

/* ================= PARAMETER SETTERS ================= */

void setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE rate)
{
    e22_cfg.REG0 &= ~0x07;
    e22_cfg.REG0 |= rate;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

void setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD baud)
{
    e22_cfg.REG0 &= ~(0b111 << 5);
    e22_cfg.REG0 |= baud;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

void setTxPower_e22_900t22s(R1_10_E22_TX_POWER power)
{
    e22_cfg.REG1 &= ~0x03;
    e22_cfg.REG1 |= power;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

/* ================= RESET ================= */

void reset_e22_900t22s(void)
{
    uint8_t cmd = COMMAND_BYTE_RESET_MODULE;

    changeMode(CONFIG);

    uartWrite(&cmd,1);

    waitAux_e22_900t22s(500);

    changeMode(TRANS);
}