//
// Created by dyrel on 5/7/2026.
//

#ifndef CODE_RADIO_DEFNS_H
#define CODE_RADIO_DEFNS_H

#define GND_RADIO_ADDRLOW 0x4A
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

// CMD BYTES (chosen relatively arbitrarily)
#define BYTE_ABORT 217
#define BYTE_PING_RADIO 5
#define BYTE_DEFLECT_TEST 12
#define BYTE_PYRO_APOGEE 79
#define BYTE_APOGEE_BKP 83
#define BYTE_DROGUE 97

#endif //CODE_RADIO_DEFNS_H