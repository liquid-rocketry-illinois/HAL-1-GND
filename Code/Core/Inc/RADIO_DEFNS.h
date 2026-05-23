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

// ===== CMD BYTES: GND → HAL =====
// Values must match HAL-side definitions in Telemetry.h / constants.h.
// Value 0 means "no command" (default/idle — HAL ignores it).

#define BYTE_NO_CMD          0      // No command / idle
#define BYTE_HANDSHAKE       0xA1u  // Ping: HAL responds with BYTE_HANDSHAKE_ACK
#define BYTE_REQUEST_DATA    0xC3u  // Request a telemetry packet from HAL
#define BYTE_DEFLECT_TEST    150    // Command HAL to run a fin/servo deflection test
                                    //   (HAL: DEFLECT_TEST = 150 in Telemetry.h)
#define BYTE_SERVO_TARE      0xD4u  // Tell HAL to apply servoOffset1/2 as new zero pts
                                    //   (HAL: SERVO_OFFSET_CMD_BYTE = 0xD4)
#define BYTE_ABORT           217    // E-Stop: HAL halts all active operations immediately
                                    //   (HAL: SHUTDOWN_KEEPALIVE = 217)

// ===== CMD RESPONSE BYTES: HAL → GND (CommandResponseByte) =====
#define BYTE_HANDSHAKE_ACK   0xB2u  // HAL acknowledges BYTE_HANDSHAKE
                                    //   (HAL: HANDSHAKE_FC_BYTE = 0xB2)

// How many consecutive radio packets each command is repeated in.
// Tuned against measured packet-error rate on this link (~9.6 kbps air rate).
#define CMD_TRANSMIT_REPEAT  3

#endif //CODE_RADIO_DEFNS_H