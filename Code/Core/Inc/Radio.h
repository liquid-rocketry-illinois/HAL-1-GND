//
// Created by dyrel on 5/2/2026.
//

#ifndef CODE_RADIO_H
#define CODE_RADIO_H

#include "Ebyte_E22_900T22S.h"
#include "Ebyte_E22_900T22S_defs.h"
#include "RADIO_DEFNS.h"

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
    uint8_t  CommandByte; // Byte sent to HAL
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
    // @attention Must be int8_t to match HAL's telemetryData (Telemetry.h).
    // HAL stores the raw E22 RSSI byte here; convert with: dBm = -(256 - raw) / 2.
    // Using float here causes a 3-byte struct size difference that breaks decoding.
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

    /**
     * @brief Enqueue a command byte to be sent to HAL.
     *
     * Each enqueued command is transmitted CMD_TRANSMIT_REPEAT times in
     * consecutive packets to improve delivery reliability on the lossy RF
     * link. Commands are sent in FIFO order. Only one command is active at
     * a time; its byte occupies GndStationData::CommandByte while repeating,
     * then the next command is popped.
     *
     * @param cmd  One of the BYTE_* constants defined in RADIO_DEFNS.h.
     *             Do NOT enqueue BYTE_ABORT — call EStop() instead.
     * @return true if the command was added, false if the queue is full.
     */
    bool enqueueCommand(uint8_t cmd);

private:
    config_e22_900t22s cfg;
    uint8_t TXBuf[512], RXBuf[2048];
    int16_t lastSeq = 0;
    bool e_stopped = false;
    float RSSILocal = 0.0F;

    // RX data is data received from HAL (IMU, GPS, data from sensors, etc)
    telemetryData RX_Data = {};
    // TX Data is data sent to HAL (testing servos/pyros, commands)
    GndStationData TX_Data = {};

    uint16_t RXSize, TXSize;

    // ---- Command queue ----
    // Commands are enqueued via enqueueCommand() and consumed here so that
    // every command is guaranteed to be sent CMD_TRANSMIT_REPEAT times before
    // the next one is started, surviving occasional packet loss.
    static const uint8_t CMD_QUEUE_DEPTH = 8;
    uint8_t _cmdQueue[CMD_QUEUE_DEPTH] = {};
    uint8_t _cmdQHead  = 0;
    uint8_t _cmdQTail  = 0;
    uint8_t _cmdQCount = 0;
    uint8_t _cmdRepeatRemaining = 0; // how many more TX cycles for the active cmd

    int8_t ReceiveData(telemetryData &gnd);
    int8_t TransmitData(GndStationData &data);
    static uint16_t Checksum(uint8_t *data, uint16_t length);
    template<typename T>
    int8_t decodeData(T &payload, uint16_t buf_len);
    template<typename T>
    uint8_t encodeAndSend(const T &payload);
};

#endif //CODE_RADIO_H