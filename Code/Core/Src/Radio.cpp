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
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K;

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

int8_t Radio::Update(telemetryData* GNDLocalData) {
    if (e22_initialized()) {
        // Simply receive data and apply tare logic
        if (ReceiveData(RX_Data)) {
            HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
        }
        else HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
        Update_Local_Data();
        // Transmit data with RCI/RCP processes
        TransmitData(HALOutboundData);

        // Now we can do the rest of RCP's stuff. We send the new RX data to be displayed by RCI.
        static uint32_t timeLastLogged = HAL_GetTick();

        if(RCP::getDataStreaming() && HAL_GetTick() - timeLastLogged > 10) {
            timeLastLogged = HAL_GetTick();
            RCP::sendThreeFloat(RCP_DEVCLASS_ACCELEROMETER, 0,
                        {RX_Data.mAccX, RX_Data.mAccY, RX_Data.mAccZ});

            RCP::sendThreeFloat(RCP_DEVCLASS_GYROSCOPE, 0,
                        {RX_Data.mGyrX, RX_Data.mGyrY, RX_Data.mGyrZ});

            RCP::sendThreeFloat(RCP_DEVCLASS_GPS, 0,
                        {RX_Data.latitude, RX_Data.longitude, RX_Data.altitude});

            RCP::sendOneFloat(RCP_DEVCLASS_ALTITUDE, 0, RX_Data.altitude);

            RCP::sendThreeFloat(RCP_DEVCLASS_RPY, 0,
                        {RX_Data.roll, RX_Data.pitch, RX_Data.yaw});

            RCP::sendFourFloat(RCP_DEVCLASS_QUATERNION, 0,
                        {RX_Data.Qw, RX_Data.Qx, RX_Data.Qy, RX_Data.Qz});

            RCP::sendTwoFloat(RCP_DEVCLASS_ANGLED_ACTUATOR, 0,
                        {RX_Data.servoPos1, RX_Data.servoPos2});

            RCP::sendOneFloat(RCP_DEVCLASS_RADIO_STRENGTH, 0, RX_Data.RSSI);

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
    e_stopped = true;
}


// ======================= PRIVATE FUNCS ===========================

int8_t Radio::ReceiveData(telemetryData &dat) {
    int16_t len = recieve_e22_900t22s(RXBuf, sizeof(telemetryData));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(dat);
    if(status != 0)             return (uint8_t)status;
    return 0;
}

int8_t Radio::TransmitData(const GndStationData &dat) {
    if(e_stopped) {
        GndStationData abortDat = dat;
        abortDat.CommandByte = BYTE_ABORT;
        encodeAndSend(abortDat);
    } else {
        encodeAndSend(dat);
    }
    return 0;
}

// template decode function that works for any data struct
// expect payload to be of gndstation data or toHAL data
template<typename T>
int8_t Radio::decodeData(T &payload)
{
    // search for SYNC1 followed by SYNC2
    int16_t sync_idx = -1;
    for(int16_t i = 0; i < (int16_t)(TELEMETRY_MAX_PAYLOAD - 1); i++)
    {
        if(RXBuf[i] == TELEMETRY_SYNC1 && RXBuf[i + 1] == TELEMETRY_SYNC2)
        {
            sync_idx = i;
            break;
        }
    }

    if(sync_idx == -1)
        return -1;  // sync bytes not found anywhere in buffer

    // ensure enough bytes remain after sync for the full header
    // [SYNC1][SYNC2][len][seq_lo][seq_hi] = 5 bytes minimum before payload
    if((sync_idx + 5) >= TELEMETRY_MAX_PAYLOAD)
        return -2;  // not enough room for header after sync

    uint8_t payload_len = RXBuf[sync_idx + 2];

    if(payload_len != sizeof(T))
        return -3;

    // ensure full packet fits in buffer
    // sync_idx + 5 header bytes + payload + 2 crc bytes
    if((sync_idx + 5 + payload_len + 2) > TELEMETRY_MAX_PAYLOAD)
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
