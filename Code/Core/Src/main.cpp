//
// Created by dyrel on 5/2/2026.
//

// For some reason, tinyusb header emits some warnings, surpress them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tusb.h"
#pragma GCC diagnostic pop

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

    // Init the various different parts of the code
    RCP::init();
    RCP::setReady(true);

    // Assign the E-STOP button functionality
    RCP::ESTOP_PROC = new Test::OneShot([]() { mainDev.EStop(); });

    // Init the only other device in use for ground station yippee
    mainDev.Init();
}


/* ===================== MAIN LOOP ===================== */

// Buffer to store received data for RCP
LRI::RingBuf<uint8_t, 1024> inbuffer;

// Extra temporary buffer to read from tinyusb
uint8_t tempIn[64];

// Unique structure for receiving HAL data. By using our own copy, we can tare/filter the values
// and still be able to compare it to HAL's raw data
telemetryData LocalGNDData;

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
        int8_t status = mainDev.Update();
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

// Tare local values so HAL can just send all the raw data
void RCP::writeSensorTare(RCP_DeviceClass devclass, uint8_t id, [[maybe_unused]] uint8_t dataChannel, float tareVal) {
    switch(devclass) {
        // Servo motors have a built in zero state but with this,
        // we can set a specific value to be zero
        case RCP_DEVCLASS_MOTOR:

            break;

        case RCP_DEVCLASS_ACCELEROMETER:

            break;

        case RCP_DEVCLASS_GYROSCOPE:

            break;

        case RCP_DEVCLASS_ALTITUDE:

            break;

        case RCP_DEVCLASS_RPY: // this is positional, not a rate

            break;

        default:

            break;
    }
}