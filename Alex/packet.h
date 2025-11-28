#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

#define MAX_STR_LEN   32
// This packet has 1 + 1 + 2 + 32 + 16 * 4 = 100 bytes
typedef struct
{

	char packetType; // Type of packet. Consult the TPacketType enum in constants.h for details
	char command; // Command to be executed by Alex when sent by Pi to Arduino, or Alex’s status when sent by Arduino to Pi.
	char dummy[2]; // Padding to make up 4 bytes
	char data[MAX_STR_LEN]; // String data
	uint32_t params[16];
//Array of 16 uint32_t integers 
// for the Pi to send command parameters to the Arduino, 
// or for the Arduino to send back telemetry data to the Pi
} TPacket;

#endif
