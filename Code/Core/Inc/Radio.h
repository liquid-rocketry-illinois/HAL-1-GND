//
// Created by dyrel on 5/2/2026.
//

#ifndef CODE_RADIO_H
#define CODE_RADIO_H

#include "Ebyte_E22_900T22S.h"
#include "Ebyte_E22_900T22S_defs.h"

// ======================= DATA ==========================

typedef struct
{
    float    servoOffset1      = 0.0f;  // S1 zero-point offset (degrees)
    float    servoOffset2      = 0.0f;  // S2 zero-point offset (degrees)
    uint32_t pyroActivation[3] = {0, 0, 0};
    uint8_t  CommandByte; // Byte sent to HAL
} GndStationData;

// ordering to reduce padding total size
typedef struct
{
    float altitude; // new
    float longitude, latitude, GPSaltitude; // your implementation used four iirc
    float mAccX, mAccY, mAccZ; // imu stuff
    float mGyrX, mGyrY, mGyrZ; // imu stuff
    float Qx, Qy, Qz, Qw; // new (quaternions)
    float pitch, yaw, roll; // tait-bryan angles (new)
    float servoTarget1, servoTarget2; // simple float vals (new?) these are commands so gnd station -> hal
    float servoPos1, servoPos2; // motor encoder readings
    uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72}; // (new)
    uint8_t CommandResponseByte; // (mainly for the radio ping command)
    int8_t RSSI; // RSSI byte from radio, describes signal strength
    bool pyroMainDrogueFired   = false; // return status of pyro
    bool pyroBackupDrogueFired = false;
    bool pyroMainChuteFired    = false;
} telemetryData;


// ============================ CLASS =================================

class Radio {
public:
    Radio();
    int8_t Init();
    int8_t Update();
    void EStop();

private:
    config_e22_900t22s cfg;
    uint8_t TXBuf[512], RXBuf[2048];
    int16_t lastSeq = 0;

    // RX data is data received from HAL (IMU, GPS, data from sensors, etc)
    telemetryData RX_Data = {};

    // TX Data is data sent to HAL (testing servos/pyros, commands)
    GndStationData TX_Data = {};

    uint16_t RXSize, TXSize;

    int8_t ReceiveData(telemetryData &gnd);
    int8_t TransmitData(const GndStationData &data);
    static uint16_t Checksum(uint8_t *data, uint16_t length);
    template<typename T>
    int8_t decodeData(T &payload);
    template<typename T>
    uint8_t encodeAndSend(const T &payload);
};

#endif //CODE_RADIO_H