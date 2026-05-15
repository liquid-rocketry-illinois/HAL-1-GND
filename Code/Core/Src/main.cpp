//
// Created by dyrel on 5/2/2026.
//

// For some reason, tinyusb header emits some warnings, surpress them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tusb.h"
#pragma GCC diagnostic pop

#include "main.h"

#include "Radio.h"
#include "RCP_Target.h"
#include "tusb.h"

extern void* EStop;

tusb_rhport_init_t TUSB_INIT_DATA = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};

Radio mainDev;

extern "C" void Init() {
    // Init tinyusb
    tud_rhport_init(BOARD_TUD_RHPORT, &TUSB_INIT_DATA);

    // Wait for the DCR line to go high (set by RCI)
    while(!(tud_cdc_get_line_state() & 0x01)) {
        tud_task_ext(5, false);
    }
    HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Init the various different parts of the code
    RCP::init();
    RCP::setReady(true);

    // Assign the E-STOP button functionality
    RCP::ESTOP_PROC = new Test::OneShot([]() { mainDev.EStop(); });
    HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    // Init the only other device in use for ground station yippee
    int8_t status = mainDev.Init();
    if (status == 0) {
        HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(BZR_GPIO_Port, BZR_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
    }
    // breakpoint point
}


/* ===================== MAIN LOOP ===================== */

// Buffer to store received data for RCP
LRI::RingBuf<uint8_t, 1024> inbuffer;

// Extra temporary buffer to read from tinyusb
uint8_t tempIn[64];

// Unique structure for receiving/sending HAL data. By using our own copy, we can tare/filter the values
// and still be able to compare it to HAL's raw data
telemetryData LocalGNDData = {};
GndStationData HALOutboundData = {};

// stores data for taring and offset. sensors
telemetryData LocalDataOffsets = {};

extern "C" void Run() {
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
      };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    while (1) {
        // Make sure to call the tinyusb processing function so it can do its thing. This is why
        // nothing in the code can block
        tud_task_ext(1, false);

        // Read bytes from tinyusb input
        uint32_t read = tud_cdc_read(tempIn, sizeof(tempIn) / sizeof(uint8_t));

        // Push the bytes into the buffer
        for(uint32_t i = 0; i < read; i++) {
            inbuffer.push(tempIn[i]);
        }

        // Make sure to flush output so tinyusb actually sends the data over the wire
        tud_cdc_write_flush();

        // Update both parts.
        RCP::yield();
        int8_t status = mainDev.Update(&LocalGNDData);
    }
}

// ============== RCP Required functions ==============
void RCP::write(const void* data, uint8_t length) { tud_cdc_write(data, length); }

uint8_t RCP::readAvail() { return inbuffer.size(); }

uint8_t RCP::read() {
    uint8_t val;
    inbuffer.pop(val);
    return val;
}

uint32_t RCP::systime() { return HAL_GetTick(); }

// ============= Callback Implementations =============

RCP_SimpleActuatorState RCP::readSimpleActuator(uint8_t id) {
    switch (id) {
        //
        case 0:
            return (LocalGNDData.pyroMainDrogueFired == 1) ? RCP_SIMPLE_ACTUATOR_ON : RCP_SIMPLE_ACTUATOR_OFF;
        case 1:
            return (LocalGNDData.pyroBackupDrogueFired == 1) ? RCP_SIMPLE_ACTUATOR_ON : RCP_SIMPLE_ACTUATOR_OFF;
        case 2:
            return (LocalGNDData.pyroMainChuteFired == 1) ? RCP_SIMPLE_ACTUATOR_ON : RCP_SIMPLE_ACTUATOR_OFF;
        case 3:
            // This is the command byte stuff
            switch (LocalGNDData.CommandResponseByte) {
            // Sensor and status issues start at 0 going up
                case 0:
                    RCPDebug("All Nominal");
                case 1:
                    RCPDebug("Radio Connection Issue!");
                case 2:
                    RCPDebug("Servo tolerance Issue!");
                // More error codes may be added to account for other things

            // Controls and CONOPs issues start at 255 going down
                case 255:
                    RCPDebug("Main Drogue Triggered");
                case 254:
                    RCPDebug("Backup Drogue Triggered");
                case 253:
                    RCPDebug("Main Chute Triggered");
                case 252:
                    RCPDebug("Burnout. Starting Roll-CTRL");
                case 251:
                    RCPDebug("Roll CTRL Deactivated!");
                case 250:
                    RCPDebug("WARNING Horizontal drift notable!");

                default:
                    RCPDebug("Unknown Command Response!");
            }
            return RCP_SIMPLE_ACTUATOR_TOGGLE; // Keep track of continuous updating if debug msg is the same
        default:
            return RCP_SIMPLE_ACTUATOR_OFF;
    }
}

RCP_SimpleActuatorState RCP::simpleActuatorWrite_CLBK(uint8_t id, RCP_SimpleActuatorState state) {
    switch (id) {
        case 0:
            if (state == RCP_SIMPLE_ACTUATOR_ON) HALOutboundData.pyroActivation = PYRODROGUEMAIN;
        case 1:
            if (state == RCP_SIMPLE_ACTUATOR_ON) HALOutboundData.pyroActivation = PYRODROGUEBKP;
        case 2:
            if (state == RCP_SIMPLE_ACTUATOR_ON) HALOutboundData.pyroActivation = PYROMAIN;

        // For writing to HAL, only the ABORT signal is needed. Everything else is handled automatically
        // including the cmd-ack. Therefore case 3 for the last state isn't needed. Instead we'll use
        // it for deflection testing
        case 3:
            // Using 150 just because
            if (state == RCP_SIMPLE_ACTUATOR_ON) HALOutboundData.CommandByte = 150;
        default:
            HALOutboundData.pyroActivation = 0;
            HALOutboundData.CommandByte = 0;
    }
    return RCP_SIMPLE_ACTUATOR_OFF;
}

// Callback for reading from sensor device. Automatic data streaming is handled inside the various singletons
RCP::Floats4 RCP::readSensor(RCP_DeviceClass devclass, uint8_t id) {
    Floats4 floats;

    // Switch on the device class, then load the floats structure with the appropriate data
    switch(devclass) {
        case RCP_DEVCLASS_ANGLED_ACTUATOR:
            floats.vals[0] = LocalGNDData.servoPos1;
            floats.vals[1] = LocalGNDData.servoPos2;
            break;

        case RCP_DEVCLASS_ALTITUDE:
            floats.vals[0] = LocalGNDData.altitude;
            break;

        case RCP_DEVCLASS_GPS:
            floats.vals[0] = LocalGNDData.latitude;
            floats.vals[1] = LocalGNDData.longitude;
            floats.vals[2] = LocalGNDData.GPSaltitude;
            break;

        case RCP_DEVCLASS_GYROSCOPE:
            floats.vals[0] = LocalGNDData.mGyrX;
            floats.vals[1] = LocalGNDData.mGyrY;
            floats.vals[2] = LocalGNDData.mGyrZ;
            break;

        case RCP_DEVCLASS_QUATERNION:
            floats.vals[0] = LocalGNDData.Qw;
            floats.vals[1] = LocalGNDData.Qx;
            floats.vals[2] = LocalGNDData.Qy;
            floats.vals[3] = LocalGNDData.Qz;
            break;

        case RCP_DEVCLASS_RPY:
            floats.vals[0] = LocalGNDData.roll;
            floats.vals[1] = LocalGNDData.pitch;
            floats.vals[2] = LocalGNDData.yaw;
            break;

        case RCP_DEVCLASS_RADIO_STRENGTH:
            floats.vals[0] = LocalGNDData.RSSI;
            break;

        case RCP_DEVCLASS_TEMPERATURE:
            floats.vals[0] = LocalGNDData.temperature;
            break;

        default:
            break;
    }

    return floats;
}

// Tare local values so HAL can just send all the raw data
void RCP::writeSensorTare(RCP_DeviceClass devclass, uint8_t id, [[maybe_unused]] uint8_t dataChannel, float tareVal) {
    switch(devclass) {
        // Servo motors have a built in zero state but with this,
        // we can set a specific value to be zero.
        // This controls a rocket-side item, this data is sent to HAL...
        case RCP_DEVCLASS_ANGLED_ACTUATOR:
            if (id == 0) HALOutboundData.servoOffset1 = tareVal;
            if (id == 1) HALOutboundData.servoOffset2 = tareVal;
            break;

            // NOTE: accelerometer and gyroscope are not zeroed. we need what the sensor detects

            // ...but for most the change only needs to be local.
        case RCP_DEVCLASS_ALTITUDE:
            LocalDataOffsets.altitude   = LocalGNDData.altitude;
            break;

        case RCP_DEVCLASS_RPY: // this is positional, not a rate
            LocalDataOffsets.roll       = LocalGNDData.roll;
            LocalDataOffsets.pitch      = LocalGNDData.pitch;
            LocalDataOffsets.yaw        = LocalGNDData.yaw;
            break;

        default:
            break;
    }
}