//
// Created by chris on 4/1/2026.
// Revised by Claude Code 2026-04-17.
//

#include "MAXM10S.h"
#include "cmsis_os2.h"
#include "i2c.h"

MAXM10S::MAXM10S() : _hi2c(nullptr) {}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

bool MAXM10S::sendUBX(const uint8_t *msg, uint16_t len)
{
    return HAL_I2C_Master_Transmit(_hi2c, I2C_ADDRESS,
                                   const_cast<uint8_t*>(msg), len, 50) == HAL_OK;
}

uint16_t MAXM10S::getAvailableBytes()
{
    uint8_t buf[2] = {0};
    // 2-byte register read at 400kHz I2C takes <1ms; 20ms is a generous ceiling
    if (HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_BYTES_AVAIL,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 20) != HAL_OK)
        return 0;
    uint16_t n = (uint16_t)((buf[0] << 8) | buf[1]);
    return (n == 0xFFFF) ? 0 : n;  // 0xFFFF = I2C FIFO stall; treat as 0
}

uint16_t MAXM10S::readStream(uint16_t numBytes)
{
    if (numBytes == 0) return 0;
    if (numBytes > RX_BUF_SIZE) numBytes = RX_BUF_SIZE;
    // At 400kHz I2C, 1024 bytes takes ~23ms; 50ms is a safe ceiling
    if (HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_DATA_STREAM,
                         I2C_MEMADD_SIZE_8BIT, rxBuf, numBytes, 50) != HAL_OK)
        return 0;
    return numBytes;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

uint8_t MAXM10S::Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;

    // Release reset and wait for module boot (~300 ms minimum)
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
    osDelay(500);

    // UBX-CFG-NAV5: set dynamic model to Airborne <4g (dynModel=7).
    // Without this the default "portable" model rejects solutions at high
    // altitude and vertical velocity — GPS goes silent mid-flight.
    // Payload is exactly 36 bytes; checksum covers class+ID+length+payload.
    // CK_A=0x56, CK_B=0x1F (Fletcher-8, verified).
    static const uint8_t ubx_nav5[] = {
        0xB5, 0x62,              // sync chars
        0x06, 0x24,              // class=CFG, ID=NAV5
        0x24, 0x00,              // payload length = 36 (LE)
        // payload (36 bytes) ─────────────────────────────────────────────
        0x01, 0x00,              // [0-1]  mask: bit0 = apply dynModel only
        0x07,                    // [2]    dynModel  = 7 (Airborne <4g)
        0x03,                    // [3]    fixMode   = 3 (auto 2D/3D)
        0x00, 0x00, 0x00, 0x00, // [4-7]  fixedAlt
        0x10, 0x27, 0x00, 0x00, // [8-11] fixedAltVar
        0x05,                    // [12]   minElev (deg)
        0x00,                    // [13]   drLimit
        0xFA, 0x00,              // [14-15] pDop (×0.1)
        0xFA, 0x00,              // [16-17] tDop (×0.1)
        0x64, 0x00,              // [18-19] pAcc (m)
        0x2C, 0x01,              // [20-21] tAcc (ns)
        0x00,                    // [22]   staticHoldThresh
        0x3C,                    // [23]   dgnssTimeout
        0x00,                    // [24]   cnoThreshNumSVs
        0x00,                    // [25]   cnoThresh
        0x00, 0x00,              // [26-27] reserved1
        0x00, 0x00,              // [28-29] staticHoldMaxDist
        0x00,                    // [30]   utcStandard
        0x00, 0x00, 0x00, 0x00, 0x00, // [31-35] reserved2
        // checksum ──────────────────────────────────────────────────────
        0x56, 0x1F               // CK_A, CK_B
    };
    sendUBX(ubx_nav5, sizeof(ubx_nav5));

    return 0;
}

uint8_t MAXM10S::update()
{
    uint16_t available = getAvailableBytes();
    if (available == 0) return GPS_NO_BYTES;

    uint16_t bytesRead = readStream(available);
    if (bytesRead == 0) return GPS_READ_FAIL;

    bool newData = false;

    for (uint16_t i = 0; i < bytesRead; i++) {
        uint8_t b = rxBuf[i];
        if (b == 0xFF) continue;  // u-blox DDC filler byte — no data pending

        if (!gpsParser.encode(b)) continue;  // mid-sentence — keep feeding

        // A complete NMEA sentence was parsed. Update all fields unconditionally
        // so callers always see current satellite/quality state even without a fix.

        if (gpsParser.satellites.isValid())
            gpsDataStruct.satellites = (uint8_t)gpsParser.satellites.value();

        if (gpsParser.hdop.isValid())
            gpsDataStruct.hdop = (float)gpsParser.hdop.hdop();

        gpsDataStruct.isLocated   = gpsParser.location.isValid();
        gpsDataStruct.isAltituded = gpsParser.altitude.isValid();
        gpsDataStruct.isTimed     = gpsParser.time.isValid();
        gpsDataStruct.hasFix      = gpsDataStruct.isLocated && gpsDataStruct.isAltituded;

        if (gpsParser.location.isUpdated() && gpsDataStruct.isLocated) {
            gpsDataStruct.latitude  = gpsParser.location.lat();
            gpsDataStruct.longitude = gpsParser.location.lng();
            gpsDataStruct.lastFixMs = HAL_GetTick();
            newData = true;
        }

        if (gpsParser.altitude.isUpdated() && gpsDataStruct.isAltituded) {
            gpsDataStruct.altitude = gpsParser.altitude.meters();
            newData = true;
        }

        if (gpsParser.speed.isUpdated() && gpsParser.speed.isValid()) {
            gpsDataStruct.speedKmh = (float)gpsParser.speed.kmph();
            newData = true;
        }

        if (gpsParser.time.isUpdated() && gpsDataStruct.isTimed) {
            gpsDataStruct.hour   = gpsParser.time.hour();
            gpsDataStruct.minute = gpsParser.time.minute();
            gpsDataStruct.second = gpsParser.time.second();
            newData = true;
        }
    }

    return newData ? GPS_NEW_DATA : GPS_NO_NEW_SENTENCE;
}

MAXM10S::gpsData MAXM10S::getData() const
{
    return gpsDataStruct;
}
