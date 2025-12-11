#include "ch32fun.h"
#include "iSLER.h"
#include <stdio.h>

#define ACCESS_ADDRESS 0x8E89BED6 // the "BED6" address for BLE advertisements

#define PRINT_STRUCT_BYTES(struct_ptr, format) do { \
	const u8* bytes = (const u8*)(struct_ptr); \
	for (size_t i = 0; i < sizeof(*(struct_ptr)); i++) { \
		printf(format " ", bytes[i]); \
	} \
	printf("\n"); \
} while(0)


#define BLE_AD_MAC(mac) \
	(mac & 0xFF), (mac>>8) & 0xFF, \
	(mac>>16) & 0xFF, (mac>>24) & 0xFF, \
	(mac>>32) & 0xFF, (mac>>40) & 0xFF

#define BLE_AD_FLAGS(flags) 0x02, 0x01, flags

#define PHY_MODE				PHY_1M
#define MAX_PAYLOAD_LEN		 20

#ifndef PACKED
#define PACKED __attribute__( ( packed ) )
#endif

typedef struct PACKED {
	u16 preamble;		  // Sync pattern (0xAABB)
	u16 control_bits;	  // control bits
	u16 msgCode;		   // message integrity check
	u8 dest[6];			// destination

	u8 group_id;
	u8 data_len;		  			 // length
	u8 payload[MAX_PAYLOAD_LEN];	// max payload length
} MESS_DataFrame_t;

typedef struct PACKED {
	u8 mac[6];
	u8 field_adv_flags[3];
	u8 name_len;
	u8 ad_type_local_name;
	u8 name[20];
	u8 data_len;
	u8 field_sev_data[3];
	MESS_DataFrame_t dataFrame;
} iSLER_frame_t;

typedef struct PACKED {
	u8 command;
	u16 value1;
	u16 value2;
	u16 value3;
	u16 value4;
	u16 value5;
	u16 value6;
	u16 value7;
	u16 value8;
} remote_command_t;


// BLE advertisements are sent on channels 37, 38 and 39
u8 adv_channels[] = {37, 38, 39};
// u8 adv_channels[] = {37};

iSLER_frame_t frame = {
	.mac = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66},
	.field_adv_flags = {0x02, 0x01, 0x06},
	.name_len = 21,	 // name length is only 20 + local name byte
	.ad_type_local_name = 0x09,
	.name = { 'b','e', 'e', '-', '5', '5', '5' },
	.data_len = sizeof(MESS_DataFrame_t) + 3,
	.field_sev_data = {0xFF, 0xD7, 0x07},
	.dataFrame = {
		.preamble = 0xA1A2,
		.control_bits = 0xB1B2,
		.msgCode = 0xC1C2,
		.dest = {0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6},
		.group_id = 0x55,
		.data_len = sizeof(remote_command_t)
	}
};

void MESS_advertise(remote_command_t *cmd) {
	memcpy(&(frame.dataFrame.payload), cmd, sizeof(remote_command_t));

	// printf("Frame: ");
	// PRINT_STRUCT_BYTES(&frame, "%02X");
	// printf("\n");

	for(int c = 0; c < sizeof(adv_channels); c++) {
		Frame_TX(ACCESS_ADDRESS, (u8*)&frame, sizeof(frame), adv_channels[c], PHY_MODE);
	}
}

remote_command_t* chMess_rx_handler() {
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	u8 *frame = (u8*)LLE_BUF;
	iSLER_frame_t* rx_frame = (iSLER_frame_t*)(frame + 2);
	u8 target_mac[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

	if (memcmp(rx_frame->mac, target_mac, 6) == 0) {
		// first 8 bytes contains: [RSSI x 1Byte] [len x 1Byte] [MAC x 6Bytes]
		// The first two bytes of the frame are metadata with RSSI and length
		// printf("RSSI:%d len:%d MAC:", frame[0], frame[1]);
		// PRINT_ARRAY(rx_frame->mac, "%02X");
		// printf("Raw Data: ");
		// PRINT_ARRAY_WITH_SIZE(frame, frame[1], "%02X");
		
		remote_command_t *cmd = (remote_command_t*)rx_frame->dataFrame.payload;
		// printf("Command: %02X Value1: %08X Value2: %08X\n", 
		// cmd->command, cmd->value1, cmd->value2);
		return cmd;
	}

	return NULL;
}
