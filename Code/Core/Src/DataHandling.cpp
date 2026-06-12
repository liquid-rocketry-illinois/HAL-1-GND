//
// Created by dyrel on 5/13/2026.
//

#include "DataHandling.h"
#include <cmath>

// stuff from other files
extern telemetryData LocalGNDData;
extern telemetryData LocalDataOffsets;
extern Radio mainDev;

// Send/Receive of the E22 module is done in Radio.cpp. This function applies any changes to the RCI read data.
void Update_Local_Data() {
    // Copy RX_Data but keep the RX_Data intact if we need it
    LocalGNDData = mainDev.GetRXData();

    // Taring
    LocalGNDData.mAccX -= LocalDataOffsets.mAccX;
    LocalGNDData.mAccY -= LocalDataOffsets.mAccY;
    LocalGNDData.mAccZ -= LocalDataOffsets.mAccZ;
    LocalGNDData.mGyrX -= LocalDataOffsets.mGyrX;
    LocalGNDData.mGyrY -= LocalDataOffsets.mGyrY;
    LocalGNDData.mGyrZ -= LocalDataOffsets.mGyrZ;
    LocalGNDData.altitude -= LocalDataOffsets.altitude;
    LocalGNDData.roll -= LocalDataOffsets.roll;
    LocalGNDData.pitch -= LocalDataOffsets.pitch;
    LocalGNDData.yaw -= LocalDataOffsets.yaw;

    // Wrap an angle (degrees) into [-180, 180] so that e.g. 359° == -1°.
    auto wrapAngle = [](float a) -> float {
        a = fmodf(a, 360.0f);
        if (a >  180.0f) a -= 360.0f;
        if (a < -180.0f) a += 360.0f;
        return a;
    };

    // If angle magnitude exceeds 30° for 5 consecutive frames, abort.
    // Debounce prevents a single corrupt/startup packet from latching EStop.
    static uint8_t estop_consec = 0;
    const float wp = wrapAngle(LocalGNDData.pitch);
    const float wy = wrapAngle(LocalGNDData.yaw);
    if (fabsf(wp) > 30.0f || fabsf(wy) > 30.0f) {
        if (++estop_consec >= 5) mainDev.EStop();
    } else {
        estop_consec = 0;
    }
}