//
// Created by dyrel on 5/13/2026.
//

#include "DataHandling.h"

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
}