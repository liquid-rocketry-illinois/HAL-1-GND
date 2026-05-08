#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "tusb_option.h"

// Hardware configuration options
#define CFG_TUSB_MCU                        OPT_MCU_STM32C0
#define CFG_TUSB_OS                         OPT_OS_NONE
#define BOARD_TUD_RHPORT                    0
#define BOARD_DEVICE_RHPORT_SPEED           OPT_MODE_FULL_SPEED

// Device configuration
#define CFG_TUD_ENABLED                     1
#define CFG_TUD_MAX_SPEED                   BOARD_DEVICE_RHPORT_SPEED
#define CFG_TUSB_MEM_ALIGN                  __attribute__ ((aligned(4)))
#define USB_VID                             0xCAFE
#define USB_PID                             0x0001
#define USB_BCD                             0x0200
// #define CFG_TUD_ENDPPOINT_MAX               8

#define CFG_TUD_ENDPOINT0_SIZE              64

#define CFG_TUD_CDC                         1
#define CFG_TUD_MSC                         0
#define CFG_TUD_HID                         0
#define CFG_TUD_MIDI                        0
#define CFG_TUD_VENDOR                      0

#define CFG_TUD_CDC_RX_BUFSIZE              1024
#define CFG_TUD_CDC_TX_BUFSIZE              1024
#define CFG_TUD_CDC_EP_BUFSIZE              64
// #define CFG_TUD_MSC_EP_BUFSIZE              512

#define CFG_TUSB_DEBUG_PRINTF printf
#define CFG_TUSB_DEBUG 0
// #define TU_LOG printf

#endif //TUSB_CONFIG_H
