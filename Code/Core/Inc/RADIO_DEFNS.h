//
// Created by dyrel on 5/7/2026.
//

#ifndef CODE_RADIO_DEFNS_H
#define CODE_RADIO_DEFNS_H

// GND's own radio address — must match what HAL has for GND_RADIO_ADDRLOW/HIGH
// in its Telemetry.h (0x4A:0x4A), which is what HAL puts in the fixed-point
// packet header when sending telemetry back to us.
#define GND_RADIO_ADDRLOW  0x4A
#define GND_RADIO_ADDRHIGH 0x4A

// HAL-side syncing variables.
/** @attention make sure these are identical on both HAL and GND sides!!!!
 */
#define TELEMETRY_SYNC1         0xAA
#define TELEMETRY_SYNC2         0x55
#define TELEMETRY_MAX_PAYLOAD 240 // set to maximum payload size
#define HAL1_RADIO_NETID 0xE6
#define HAL1_RADIO_ADDRLOW 0x06
#define HAL1_RADIO_ADDRHIGH 0x07

// ERROR CODES
#define E22_NO_DATA             2
#define E22_RECEIVE_ERR         3
#define E22_BAD_LENGTH          4

// ===== SPECIAL CHANNEL BYTE VALUES =====
// channelByte normally carries the R2_E22Channel915 channel value (52–78).
// BYTE_ABORT is out of that range so HAL can distinguish an e-stop from a channel.
#define BYTE_ABORT           217    // E-Stop: HAL halts all active operations immediately
                                    //   (HAL: SHUTDOWN_KEEPALIVE = 217)

#endif //CODE_RADIO_DEFNS_H