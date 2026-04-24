//
// Created by chris on 4/1/2026.
// Revised by Claude Code 2026-04-17.
//

#ifndef KINGFISHER_SW_MAXM10S_H
#define KINGFISHER_SW_MAXM10S_H

#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "TinyGPSPlus.h"

class MAXM10S {

public:
    // Return codes from update()
    enum UpdateStatus : uint8_t {
        GPS_NEW_DATA        = 0,  // at least one NMEA field updated this call
        GPS_READ_FAIL       = 1,  // I2C transaction failed
        GPS_NO_BYTES        = 2,  // module reports 0 bytes pending
        GPS_NO_NEW_SENTENCE = 3,  // bytes read but no complete sentence parsed
    };

    struct gpsData {
        // Position
        double  latitude   = 0.0;   // degrees, + = north
        double  longitude  = 0.0;   // degrees, + = east
        double  altitude   = 0.0;   // metres MSL (3D fix required)
        float   speedKmh   = 0.0f;  // ground speed

        // Fix quality
        uint8_t satellites = 0;     // satellites currently tracked
        float   hdop       = 99.9f; // horizontal dilution of precision (lower = better)
        bool    hasFix     = false; // true when location + altitude are valid

        // Validity flags (set independently; time often valid before position)
        bool isLocated   = false;
        bool isAltituded = false;
        bool isTimed     = false;

        // Time (UTC)
        uint8_t hour   = 0;
        uint8_t minute = 0;
        uint8_t second = 0;

        // Timestamp of last successful position fix (HAL_GetTick ms)
        uint32_t lastFixMs = 0;
    };

    MAXM10S();
    uint8_t Init(I2C_HandleTypeDef *hi2c);
    uint8_t update();
    gpsData getData() const;

private:
    I2C_HandleTypeDef *_hi2c;
    TinyGPSPlus        gpsParser;
    gpsData            gpsDataStruct;

    static constexpr uint16_t I2C_ADDRESS     = 0x84; // 7-bit 0x42 << 1
    static constexpr uint16_t REG_BYTES_AVAIL = 0xFD;
    static constexpr uint16_t REG_DATA_STREAM = 0xFF;
    static constexpr uint16_t RX_BUF_SIZE     = 1024;

    uint8_t  rxBuf[RX_BUF_SIZE];

    uint16_t getAvailableBytes();
    uint16_t readStream(uint16_t numBytes);
    bool     sendUBX(const uint8_t *msg, uint16_t len);
};

#endif //KINGFISHER_SW_MAXM10S_H
