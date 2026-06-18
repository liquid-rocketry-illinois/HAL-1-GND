//
// Created by dyrel on 5/2/2026.
//

#ifndef CODE_RADIO_H
#define CODE_RADIO_H

#include "Ebyte_E22_900T22S.h"
#include "Ebyte_E22_900T22S_defs.h"
#include "RADIO_DEFNS.h"

#define GLOBAL_RADIO_CHAN CH909

// ======================= DATA ==========================

typedef enum {
    PYROMAIN        = 121734683,
    PYRODROGUEBKP   = 402746912,
    PYRODROGUEMAIN  = 243656272
} pyroActivateKeys;

typedef struct
{
    float    servoOffset1      = 0.0f;  // S1 zero-point offset (degrees)
    float    servoOffset2      = 0.0f;  // S2 zero-point offset (degrees)
    uint32_t pyroActivation    = 0;
    uint8_t  channelByte       = GLOBAL_RADIO_CHAN;  // Operating channel (R2_E22Channel915 cast to uint8_t)
} GndStationData;

// ordering to reduce padding total size
typedef struct
{
    float altitude; // new
    float verticalVelocity;
    float longitude, latitude, GPSaltitude;
    float mAccX, mAccY, mAccZ; // imu stuff
    float mGyrX, mGyrY, mGyrZ; // imu stuff
    float Qx, Qy, Qz, Qw; // new (quaternions)
    float pitch, yaw, roll; // tait-bryan angles (new)
    float servoTarget1, servoTarget2; // simple float vals (new?) these are commands so gnd station -> hal
    float servoPos1, servoPos2; // motor encoder readings
    float temperature; // use averaged temperatures from BMP390L and BMI323 TMR
    uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72}; // (new)
    uint8_t CommandResponseByte; // (mainly for the radio ping command)
    int8_t RSSI;
    bool pyroMainDrogueFired   = false; // return status of pyro
    bool pyroBackupDrogueFired = false;
    bool pyroMainChuteFired    = false;
} telemetryData;


// ============================ CLASS =================================

class Radio {
public:
    Radio();
    int8_t Init();
    int8_t Update(telemetryData* GNDLocalData);
    void EStop();
    telemetryData GetRXData();
    float getRSSI();

    // Initiate a channel change. GND first announces the new channel on the current
    // channel, then switches its own radio, then verifies HAL responds on the new
    // channel. Reverts if HAL goes silent.
    void setChannel(R2_E22Channel915 ch);

private:
    config_e22_900t22s cfg;
    uint8_t TXBuf[240], RXBuf[240];
    int16_t lastSeq = 0;
    bool e_stopped = false;

    enum class ChannelState { STABLE, ANNOUNCING, VERIFYING };

    R2_E22Channel915 _targetChannel  = GLOBAL_RADIO_CHAN;  // channel GND radio is currently on
    R2_E22Channel915 _pendingChannel = GLOBAL_RADIO_CHAN;  // channel being switched to
    R2_E22Channel915 _prevChannel    = GLOBAL_RADIO_CHAN;  // for revert on verify timeout
    ChannelState     _channelState   = ChannelState::STABLE;
    uint32_t         _channelStateTick = 0;

    bool     _hasLinked  = false;  // true after first successful RX from HAL
    uint32_t _lastRxTick = 0;     // timestamp of last successful RX

    static constexpr uint32_t CHANNEL_VERIFY_TIMEOUT_MS = 5000u;
    static constexpr uint32_t LINK_LOST_TIMEOUT_MS      = 5000u;

    // RX data is data received from HAL (IMU, GPS, data from sensors, etc)
    telemetryData RX_Data = {};
    // TX Data is data sent to HAL (testing servos/pyros, commands)
    GndStationData TX_Data = {};

    uint16_t RXSize, TXSize;

    // Snapshot of the last transmitted outbound packet; used to detect changes.
    GndStationData _lastSentData = {};

    // Burst-transmit timing constants.
    static constexpr uint32_t BURST_DURATION_MS    = 1000u;   // transmit window length
    static constexpr uint32_t PERIODIC_INTERVAL_MS = 10000u;  // heartbeat period

    // Burst state: when triggered, the frozen snapshot is retransmitted for
    // BURST_DURATION_MS to maximise delivery in one concentrated window.
    bool           _burstActive      = false;
    uint32_t       _burstStartTick   = 0;
    uint32_t       _lastPeriodicTick = 0;
    GndStationData _burstSnapshot    = {};

    int8_t ReceiveData(telemetryData &gnd);
    int8_t TransmitData(GndStationData &data);
    int8_t WirelessConfig(config_e22_900t22s *cfg);
    static uint16_t Checksum(uint8_t *data, uint16_t length);
    template<typename T>
    int8_t decodeData(T &payload, uint16_t buf_len);
    template<typename T>
    uint8_t encodeAndSend(const T &payload);
};

#endif //CODE_RADIO_H
