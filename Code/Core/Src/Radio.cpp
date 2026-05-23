//
// Created by dyrel on 5/2/2026.
//

#include "Radio.h"
#include "Ebyte_E22_900T22S.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal.h"
#include "main.h"
#include <cstring>
#include "RADIO_DEFNS.h"
#include "RCP_Target.h"
#include "DataHandling.h"

extern UART_HandleTypeDef huart1;

Radio::Radio() {
    RXSize = sizeof(RXBuf);
    TXSize = sizeof(TXBuf);
}

telemetryData Radio::GetRXData() {
    return RX_Data;
}

float Radio::getRSSI()
{
    return RSSILocal;
}

int8_t Radio::Init() {
    config_e22_900t22s des_cfg = {};

    des_cfg.huart = &huart1;

    des_cfg.E22_AUX_PIN  = RADIO_AUX_Pin;
    des_cfg.E22_AUX_PORT = RADIO_AUX_GPIO_Port;
    des_cfg.E22_M0_PIN   = RADIO_M0_Pin;
    des_cfg.E22_M0_PORT  = RADIO_M0_GPIO_Port;
    des_cfg.E22_M1_PIN   = RADIO_M1_Pin;
    des_cfg.E22_M1_PORT  = RADIO_M1_GPIO_Port;

    des_cfg.ADDH = GND_RADIO_ADDRHIGH;
    des_cfg.ADDL = GND_RADIO_ADDRLOW;

    des_cfg.NETID = 0xE6; // const

    des_cfg.REG0 =   R0_765_E22_UART_BAUD::E22_UART_BAUD_38400
                   | R0_43_SERIAL_PORT_PARITY_BIT::MODE_8N1
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K; // must match HAL (Telemetry.cpp)

    des_cfg.REG1 =   R1_76_SUB_PACKET_SETTING::BYTES_240
                   | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE
                   | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                   | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    des_cfg.REG2 = CH915;

    des_cfg.REG3 =   R3_7_RSSI_BYTE_ENABLE
                   | R3_6_TRANSFER_METHOD_FIXED_POINT
                   | R3_5_REPEATER_OFF
                   | R3_4_LBT_DISABLED
                   | R3_3_WOR_MODE_RECIEVER
                   | R3_210_WOR_CYCLE_TIME::_2000_ms;

    int8_t status = init_e22_900t22s(&des_cfg);
    if(status != 0)
        return status;

    changeMode(TRANS);
    HAL_Delay(400);
    return 0;
}

extern GndStationData HALOutboundData;
// 1-second success-rate window → beep duration + period mapping:
//   <10%  success → silent
//   10–24%        → 10 ms on,  2000 ms period
//   25–49%        → 20 ms on,  1000 ms period
//   50–74%        → 50 ms on,   500 ms period
//   75–97%        → 100 ms on,  250 ms period
//   >97%          → constant on
static uint32_t bzr_window_start = 0;
static uint16_t bzr_total        = 0;
static uint16_t bzr_success      = 0;
static uint16_t bzr_beep_ms      = 0;      // 0 = silent, 0xFFFF = constant on
static uint32_t bzr_period_ms    = 2000;
static uint32_t bzr_pulse_start  = 0;
static bool     bzr_pulsing      = false;

int8_t Radio::Update(telemetryData* GNDLocalData) {
    if (e22_initialized()) {
        uint32_t now = HAL_GetTick();

        // ---- 1. Drive CommandByte from the command queue ----
        // Must happen before TransmitData so the correct byte goes out this cycle.
        // BYTE_ABORT bypasses this entirely — TransmitData injects it when e_stopped.
        if (_cmdRepeatRemaining > 0) {
            _cmdRepeatRemaining--;
        } else if (_cmdQCount > 0) {
            HALOutboundData.CommandByte = _cmdQueue[_cmdQHead];
            _cmdQHead = (_cmdQHead + 1) % CMD_QUEUE_DEPTH;
            _cmdQCount--;
            _cmdRepeatRemaining = CMD_TRANSMIT_REPEAT - 1; // already sending once now
        } else {
            HALOutboundData.CommandByte = BYTE_NO_CMD;
        }

        // ---- 2. Transmit to HAL first (GND is master, HAL is slave) ----
        // HAL blocks waiting to receive before it will send telemetry back.
        // If GND tries to receive before transmitting, both sides block on RX
        // simultaneously, both time out, and no data flows that cycle.
        TransmitData(HALOutboundData);

        // ---- 3. Start buzzer pulse before the blocking RX call ----
        // Pin goes HIGH at a known point; the RESET check runs right after
        // ReceiveData() returns to minimise extra on-time from UART blocking.
        if (bzr_beep_ms != 0xFFFF && bzr_beep_ms != 0) {
            if (!bzr_pulsing && now - bzr_pulse_start >= bzr_period_ms) {
                bzr_pulsing = true;
                bzr_pulse_start = now;
                HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
            }
        }

        // ---- 4. Receive HAL's telemetry response ----
        // HAL responds to any valid GND packet; being in RX mode immediately
        // after our own TX ensures we catch the response in the same cycle.
        // Round-trip at 9.6 kbps air rate ≈ 170 ms; RX timeout is 300 ms.
        int8_t rx_result = ReceiveData(RX_Data);
        now = HAL_GetTick(); // refresh after the blocking call

        // ---- 5. Buzzer stats — accumulate and recompute once per second ----
        bzr_total++;
        if (rx_result == 0) bzr_success++;

        if (now - bzr_window_start >= 1000u) {
            uint32_t succ_pct = bzr_total ? (bzr_success * 100u) / bzr_total : 0u;
            if      (succ_pct >  97u) { bzr_beep_ms = 0xFFFF; bzr_period_ms = 0;    }
            else if (succ_pct >= 75u) { bzr_beep_ms = 100;    bzr_period_ms = 250;  }
            else if (succ_pct >= 50u) { bzr_beep_ms = 50;     bzr_period_ms = 500;  }
            else if (succ_pct >= 25u) { bzr_beep_ms = 20;     bzr_period_ms = 750;  }
            else if (succ_pct >= 10u) { bzr_beep_ms = 10;     bzr_period_ms = 1000; }
            else                      { bzr_beep_ms = 0;      bzr_period_ms = 0;    }
            bzr_total = bzr_success = 0;
            bzr_window_start = now;
        }

        // drive buzzer pin
        if (bzr_beep_ms == 0xFFFF) {
            HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
        } else if (bzr_beep_ms == 0) {
            HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
        } else if (bzr_pulsing && now - bzr_pulse_start >= bzr_beep_ms) {
            bzr_pulsing = false;
            HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
        }

        // ---- 6. Non-blocking post-processing ----
        RSSILocal = getRSSIByte();
        Update_Local_Data();

        // ---- 7. RCP data streaming ----
        static uint32_t timeLastLogged = HAL_GetTick();

        if(RCP::getDataStreaming() && HAL_GetTick() - timeLastLogged > 10) {
            timeLastLogged = HAL_GetTick();
            RCP::sendThreeFloat(RCP_DEVCLASS_ACCELEROMETER, 0,
                        {RX_Data.mAccX, RX_Data.mAccY, RX_Data.mAccZ});

            RCP::sendThreeFloat(RCP_DEVCLASS_GYROSCOPE, 0,
                        {RX_Data.mGyrX, RX_Data.mGyrY, RX_Data.mGyrZ});

            RCP::sendThreeFloat(RCP_DEVCLASS_GPS, 0,
                        {RX_Data.latitude, RX_Data.longitude, RX_Data.altitude});

            RCP::sendTwoFloat(RCP_DEVCLASS_ALTITUDE, 0, {RX_Data.altitude, RX_Data.verticalVelocity});

            RCP::sendThreeFloat(RCP_DEVCLASS_RPY, 0,
                        {RX_Data.roll, RX_Data.pitch, RX_Data.yaw});

            RCP::sendFourFloat(RCP_DEVCLASS_QUATERNION, 0,
                        {RX_Data.Qw, RX_Data.Qx, RX_Data.Qy, RX_Data.Qz});

            RCP::sendTwoFloat(RCP_DEVCLASS_ANGLED_ACTUATOR, 0,
                        {RX_Data.servoPos1, RX_Data.servoPos2});

            // RX_Data.RSSI is int8_t (raw E22 byte); cast to float for RCP streaming.
            // Actual dBm value = -(256 - raw) / 2  (per E22 datasheet).
            RCP::sendTwoFloat(RCP_DEVCLASS_RADIO_STRENGTH, 0, {(float)RX_Data.RSSI, RSSILocal});

            // pyros in order of trigger
            RCP::forceSendSimpleActuatorState(0);
            RCP::forceSendSimpleActuatorState(1);
            RCP::forceSendSimpleActuatorState(2);

            // command byte
            RCP::forceSendSimpleActuatorState(3);
        }
        return 0;
    }

    return 1; // uninitialized
}

void Radio::EStop() {
    RCPDebug("ESTOP!");
    e_stopped = true;
}

bool Radio::enqueueCommand(uint8_t cmd) {
    if (_cmdQCount >= CMD_QUEUE_DEPTH)
    {
        RCPDebug("Command failed, queue full!");
        return false;
    }// queue full — caller should handle
    _cmdQueue[_cmdQTail] = cmd;
    _cmdQTail = (_cmdQTail + 1) % CMD_QUEUE_DEPTH;
    _cmdQCount++;
    return true;
}


// ======================= PRIVATE FUNCS ===========================

int8_t Radio::ReceiveData(telemetryData &dat) {
    int16_t len = recieve_e22_900t22s(RXBuf, sizeof(telemetryData));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(dat, static_cast<uint16_t>(len));
    if(status != 0)             return status;
    return 0;
}

int8_t Radio::TransmitData(GndStationData &dat) {
    if(e_stopped) {
        // Override CommandByte with BYTE_ABORT unconditionally.
        // The queue is bypassed once EStop() fires; BYTE_ABORT will be sent
        // on every subsequent Update() cycle for as long as the MCU runs,
        // which gives HAL the best chance of receiving it despite packet loss.
        dat.CommandByte = BYTE_ABORT;
        RCPDebug("Transmitting Abort Signal");
    }
    encodeAndSend(dat);
    return 0;
}

// Decode a received frame from RXBuf.
// buf_len is the number of bytes actually received (from recieve_e22_900t22s),
// so the sync search is bounded to valid data and cannot match stale buffer bytes.
template<typename T>
int8_t Radio::decodeData(T &payload, uint16_t buf_len)
{
    if (buf_len < 7u) return -1; // too short to contain any valid frame
    // buf_len >= 7 here, so buf_len - 1u >= 6u — no underflow risk.

    // Search only within the bytes we actually received.
    // Previously searched all TELEMETRY_MAX_PAYLOAD bytes, which allowed false
    // sync matches against stale buffer content from prior receives.
    const int16_t search_end = static_cast<int16_t>(buf_len - 1u);

    int16_t sync_idx = -1;
    for(int16_t i = 0; i < search_end; i++)
    {
        if(RXBuf[i] == TELEMETRY_SYNC1 && RXBuf[i + 1] == TELEMETRY_SYNC2)
        {
            sync_idx = i;
            break;
        }
    }

    if(sync_idx == -1)
        return -1;  // sync bytes not found in received data

    // Enough bytes after sync for the full 5-byte header?
    // [SYNC1][SYNC2][len][seq_lo][seq_hi] = 5 bytes before payload starts.
    if((uint16_t)(sync_idx + 5) >= buf_len)
        return -2;

    uint8_t payload_len = RXBuf[sync_idx + 2];

    if(payload_len != sizeof(T))
        return -3;  // length mismatch — wrong packet type or struct size skew

    // Full packet (header + payload + CRC) must fit inside received bytes.
    if((uint16_t)(sync_idx + 5 + payload_len + 2) > buf_len)
        return -4;

    uint16_t seq_rx = RXBuf[sync_idx + 3] | (RXBuf[sync_idx + 4] << 8);

    uint16_t crc_rx =
        RXBuf[sync_idx + 5 + payload_len] |
        (RXBuf[sync_idx + 6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(&RXBuf[sync_idx], 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;

    memcpy(&payload, &RXBuf[sync_idx + 5], sizeof(T));

    lastSeq = seq_rx;
    return 0;
}

// Similar encode func
template<typename T>
uint8_t Radio::encodeAndSend(const T &payload)
{
    static_assert(sizeof(T) <= TELEMETRY_MAX_PAYLOAD - 10, "payload exceeds TX buffer size");
    uint8_t payload_len = sizeof(T);

    // fixed mode header — module strips these before delivering to receiver
    TXBuf[0] = HAL1_RADIO_ADDRHIGH;
    TXBuf[1] = HAL1_RADIO_ADDRLOW;

    TXBuf[2] = CH915;

    // your packet starts at offset 3
    TXBuf[3] = TELEMETRY_SYNC1;
    TXBuf[4] = TELEMETRY_SYNC2;
    TXBuf[5] = payload_len;
    TXBuf[6] = lastSeq & 0xFF;
    TXBuf[7] = (lastSeq >> 8) & 0xFF;

    memcpy(&TXBuf[8], &payload, payload_len);

    uint16_t crc = Checksum(&TXBuf[3], 5 + payload_len);  // start at SYNC1
    TXBuf[8 + payload_len] = crc & 0xFF;
    TXBuf[9 + payload_len] = (crc >> 8) & 0xFF;

    // Not using transmit_fixed
    int8_t status = transmit_e22_900t22s(TXBuf, 10 + payload_len);
    if(status != E22_OK)
        return status;

    lastSeq++;
    return 0;
}

// CRC util
uint16_t Radio::Checksum(uint8_t *data, uint16_t length)
{
    uint16_t sum = 0;
    for(uint16_t i = 0; i < length; i++)
        sum += data[i];
    return sum;
}
