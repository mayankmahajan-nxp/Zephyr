/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "u_blox_protocol.h"

static void u_blox_create_frame(uint8_t ubx_frame[],
			       uint8_t *ubx_frame_size, uint8_t message_class,
			       uint8_t message_id, uint8_t payload[], uint16_t payload_size)
{
	uint8_t frame_length_without_payload = 8;

	uint8_t ckA = 0;
	uint8_t ckB = 0;

	*ubx_frame_size = frame_length_without_payload + payload_size;

	ubx_frame[0] = 0xb5;
	ubx_frame[1] = 0x62;
	ubx_frame[2] = message_class;
	ubx_frame[3] = message_id;
	ubx_frame[4] = (payload_size & 0xff);
	ubx_frame[5] = (payload_size >> 8);
	memcpy(&ubx_frame[6], payload, payload_size);

	for (unsigned int i = 2; i < (*ubx_frame_size - 2); i++) {
		ckA += ubx_frame[i];
		ckB += ckA;
	}

	ubx_frame[*ubx_frame_size - 2] = ckA;
	ubx_frame[*ubx_frame_size - 1] = ckB;
}

void u_blox_get_cfg_prt(uint8_t ubx_frame[],
				uint8_t *ubx_frame_size, uint8_t port_id, uint32_t baudrate)
{
	uint8_t payload_size = 20;
	uint8_t payload[payload_size];

	payload[0] = port_id;
	payload[1] = 0x0;

	payload[2] = 0x0;
	payload[3] = 0x0;

	/* Mode = 0x8D0 */
	payload[4] = 0xD0;
	payload[5] = 0x08;
	payload[6] = 0x0;
	payload[7] = 0x0;

	/* Baud Rate*/
	payload[8] = baudrate;
	payload[9] = baudrate >> 8;
	payload[10] = baudrate >> 16;
	payload[11] = baudrate >> 24;

	/* in_proto_mask = All proto enable */
	payload[12] = 0x7;
	payload[13] = 0x0;

	/* Out Proto Mask = All proto enable */
	payload[14] = 0x23;
	payload[15] = 0x0;

	/* flags */
	payload[16] = 0x0;
	payload[17] = 0x0;

	payload[18] = 0x0;
	payload[19] = 0x0;

	u_blox_create_frame(ubx_frame, ubx_frame_size, 0x06, 0x00, payload, payload_size);
}

void u_blox_get_cfg_rst(uint8_t ubx_frame[],
				uint8_t *ubx_frame_size, uint8_t reset_mode)
{
	uint8_t payload_size = 4;
	uint8_t payload[payload_size];

	/* navBbrMask. */
	payload[0] = 0x00;
	payload[1] = 0x00;

	/* resetMode. */
	payload[2] = reset_mode;

	/* reserved1. */
	payload[3] = 0x00;

	u_blox_create_frame(ubx_frame, ubx_frame_size, 0x06, 0x04, payload, payload_size);
}
