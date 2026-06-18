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
extern float CH_Global;
float RSSILocal = 0.0F;

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
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_4_8K; // must match HAL (Telemetry.cpp)

    des_cfg.REG1 =   R1_76_SUB_PACKET_SETTING::BYTES_240
                   | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_ENABLE
                   | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                   | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    des_cfg.REG2 = GLOBAL_RADIO_CHAN;

    des_cfg.REG3 =   R3_7_RSSI_BYTE_DISABLE
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

        // Allow buzzer to signal to end user
        if (bzr_beep_ms != 0xFFFF && bzr_beep_ms != 0) {
            if (!bzr_pulsing && now - bzr_pulse_start >= bzr_period_ms) {
                bzr_pulsing = true;
                bzr_pulse_start = now;
                HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
            }
        }

        // GND primarily acts as a receiver; receive first so that HAL's ACK byte
        // is visible before the transmit decision is made this cycle.
        int8_t rx_result = ReceiveData(RX_Data);
        now = HAL_GetTick(); // refresh tick after the blocking receive

        if (rx_result == 0) {
            _lastRxTick = now;
            _hasLinked  = true;
        }

        // Lost-link recovery: if link was established, then lost for LINK_LOST_TIMEOUT_MS
        // while on a non-default channel, revert to GLOBAL_RADIO_CHAN so both sides
        // can find each other again. Self-gated: once on default, condition is false;
        // ANNOUNCING/VERIFYING states also block re-entry.
        if (_hasLinked &&
            _channelState == ChannelState::STABLE &&
            _targetChannel != static_cast<R2_E22Channel915>(CH_Global) &&
            now - _lastRxTick >= LINK_LOST_TIMEOUT_MS) {
            RCPDebug("Link lost on non-default channel — reverting to default.");
            setChannel(static_cast<R2_E22Channel915>(static_cast<int>(CH_Global)));
        }

        // Channel-change state machine.
        // ANNOUNCING: broadcast pending channel on the OLD channel; switch GND radio
        //             only after the burst completes so HAL gets the announcement first.
        // VERIFYING:  GND switched; wait for HAL telemetry on new channel to confirm.
        //             Revert to old channel if nothing heard within the timeout.
        switch (_channelState) {
            case ChannelState::STABLE:
                break;
            case ChannelState::ANNOUNCING:
                if (!_burstActive) {
                    // Burst has finished — HAL has had 1 s of repeated announcements.
                    // Now switch GND's own radio and start verifying.
                    changeOpFreq_e22_900t22s(_pendingChannel);
                    _targetChannel    = _pendingChannel;
                    _channelState     = ChannelState::VERIFYING;
                    _channelStateTick = now;
                }
                break;
            case ChannelState::VERIFYING:
                if (rx_result == 0) {
                    // HAL responded on the new channel — both sides are in sync.
                    _channelState = ChannelState::STABLE;
                    RCPDebug("Channel change verified.");
                } else if (now - _channelStateTick >= CHANNEL_VERIFY_TIMEOUT_MS) {
                    // HAL went silent — it likely didn't receive the announcement.
                    // Revert GND back to the previous channel.
                    changeOpFreq_e22_900t22s(_prevChannel);
                    _targetChannel = _prevChannel;
                    _channelState  = ChannelState::STABLE;
                    RCPDebug("Channel change failed: no HAL response, reverted.");
                }
                break;
        }

        // During ANNOUNCING use the pending channel value so HAL knows where to go;
        // otherwise send the active channel (confirms to HAL what channel GND is on).
        HALOutboundData.channelByte = static_cast<uint8_t>(
            _channelState == ChannelState::ANNOUNCING ? _pendingChannel : _targetChannel
        );

        // Burst-transmit logic: GND stays silent unless a burst is active.
        // A burst is started by a data change or the 10-second heartbeat.
        // Once started, the same frozen snapshot is sent every cycle for 1 second
        // to maximise delivery without disrupting the FC any more than necessary.
        if (!_burstActive) {
            const bool dataChanged =
                (HALOutboundData.servoOffset1   != _lastSentData.servoOffset1)  ||
                (HALOutboundData.servoOffset2   != _lastSentData.servoOffset2)  ||
                (HALOutboundData.pyroActivation != _lastSentData.pyroActivation) ||
                (HALOutboundData.channelByte    != _lastSentData.channelByte);

            const bool periodicDue = (now - _lastPeriodicTick >= PERIODIC_INTERVAL_MS);

            if (dataChanged || periodicDue) {
                _burstActive    = true;
                _burstStartTick = now;
                _burstSnapshot  = HALOutboundData;
                _lastSentData   = HALOutboundData;
                if (periodicDue) _lastPeriodicTick = now;
            }
        }

        if (_burstActive) {
            if (now - _burstStartTick < BURST_DURATION_MS) {
                TransmitData(_burstSnapshot);
            } else {
                _burstActive = false;
            }
        }

        now = HAL_GetTick(); // refresh after any blocking transmit
        bzr_total++;
        if (rx_result < -2 || rx_result == 0) {
            bzr_success++;
            RCPDebug("Incoming packet detected!"); // notify that a packet containing sync bytes have been detected
        }

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

        // RX_Data.RSSI is int8_t (raw E22 byte); cast to float for RCP streaming.
        // Actual dBm value = -(256 - raw) / 2  (per E22 datasheet), or -raw / 2 (TODO: reference datasheet)
        static uint32_t lastRssiTick = 0;
        if (now - lastRssiTick >= 2000u) {
            RSSILocal = get_rssi_e22_900t22s();
            lastRssiTick = now;
        }
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

            RCP::sendOneFloat(RCP_DEVCLASS_ANGLED_ACTUATOR, 0, RX_Data.servoPos1);
            RCP::sendOneFloat(RCP_DEVCLASS_ANGLED_ACTUATOR, 1, RX_Data.servoPos2);

            RCP::sendOneFloat(RCP_DEVCLASS_RADIO_STRENGTH, 0, RX_Data.RSSI);
            RCP::sendOneFloat(RCP_DEVCLASS_RADIO_STRENGTH, 1, RSSILocal);

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

void Radio::setChannel(R2_E22Channel915 ch) {
    if (ch == _targetChannel && _channelState == ChannelState::STABLE) return;
    _prevChannel    = _targetChannel;
    _pendingChannel = ch;
    _channelState   = ChannelState::ANNOUNCING;
    // GND radio stays on _prevChannel until the announcement burst completes.
}


// ======================= PRIVATE FUNCS ===========================

int8_t Radio::ReceiveData(telemetryData &dat) {
    memset(&RXBuf, 0, sizeof(RXBuf));

    // +2 for the trailing SYNC2/SYNC1 pair appended by the sender after CRC.
    int16_t len = recieve_e22_900t22s(RXBuf, sizeof(telemetryData));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(dat, static_cast<uint16_t>(len));
    if(status != 0)             return status;
    return 0;
}

int8_t Radio::TransmitData(GndStationData &dat) {
    if(e_stopped) {
        // Override channelByte with BYTE_ABORT unconditionally (217 is outside
        // valid channel range 52–78, so HAL can distinguish it).
        dat.channelByte = BYTE_ABORT;
        RCPDebug("Transmitting Abort Signal");
    }
    encodeAndSend(dat);
    return 0;
}

int8_t Radio::WirelessConfig(config_e22_900t22s *cfg_d) {
    uint8_t frame[12];

    frame[0] = 0xCF;
    frame[1] = 0xCF;
    frame[2] = COMMAND_BYTE_WRITE_CFG_SAVE_FLASH;
    frame[3] = 0x00;
    frame[4] = 0x07;
    frame[5] = cfg_d->ADDH;
    frame[6] = cfg_d->ADDL;
    frame[7] = cfg_d->NETID;
    frame[8] = cfg_d->REG0;
    frame[9] = cfg_d->REG1;
    frame[10] = cfg_d->REG2;
    frame[11] = cfg_d->REG3;

    if (encodeAndSend(frame) != 0) {
        RCPDebug("Wireless Config Send Failed");
        return -1;
    }
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

    int16_t sync_idx = -1;
    for(int16_t i = 0; i < buf_len; i++)
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

    // Full packet (header + payload + CRC + trailing sync pair) must fit inside received bytes.
    // Layout after sync_idx: [S1][S2][len][seq_lo][seq_hi][payload x N][CRC_lo][CRC_hi][S2][S1]
    //                          0   1   2    3       4       5..4+N      5+N     6+N     7+N  8+N
    if((uint16_t)(sync_idx + 5 + payload_len + 4) > buf_len)
        return -4;

    uint16_t seq_rx = RXBuf[sync_idx + 3] | (RXBuf[sync_idx + 4] << 8);

    uint16_t crc_rx =
        RXBuf[sync_idx + 5 + payload_len] |
        (RXBuf[sync_idx + 6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(&RXBuf[sync_idx], 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;

    // Verify trailing sync bytes (mirror of header: SYNC2 then SYNC1)
    if(RXBuf[sync_idx + 7 + payload_len] != TELEMETRY_SYNC2 ||
       RXBuf[sync_idx + 8 + payload_len] != TELEMETRY_SYNC1)
        return -6;  // trailing sync mismatch — frame boundary corrupted

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
    TXBuf[2] = static_cast<uint8_t>(_targetChannel);

    // What is actually transmitted
    TXBuf[3] = TELEMETRY_SYNC1;
    TXBuf[4] = TELEMETRY_SYNC2;
    TXBuf[5] = payload_len;
    TXBuf[6] = lastSeq & 0xFF;
    TXBuf[7] = (lastSeq >> 8) & 0xFF;

    memcpy(&TXBuf[8], &payload, payload_len);

    uint16_t crc = Checksum(&TXBuf[3], 5 + payload_len);  // start at SYNC1
    TXBuf[8 + payload_len] = crc & 0xFF;
    TXBuf[9 + payload_len] = (crc >> 8) & 0xFF;
    TXBuf[10 + payload_len] = TELEMETRY_SYNC2;
    TXBuf[11 + payload_len] = TELEMETRY_SYNC1;

    // Not using transmit_fixed
    // +12: 3 routing bytes + SYNC1 + SYNC2 + len + seq_lo + seq_hi (8) + CRC_lo + CRC_hi + trailing SYNC2 + trailing SYNC1 (4)
    int8_t status = transmit_e22_900t22s(TXBuf, 12 + payload_len);
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
