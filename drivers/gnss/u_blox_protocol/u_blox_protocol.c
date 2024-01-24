/*
 * Copyright 2024 NXP
 * Copyright (c) 2022 Abel Sensors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "u_blox_protocol.h"

const uint32_t u_blox_baudrate[U_BLOX_BAUDRATE_COUNT] = {
	4800,
	9600,
	19200,
	38400,
	57600,
	115200,
	230400,
	460800,
};

static void u_blox_create_frame(uint8_t ubx_frame[], uint16_t *ubx_frame_size,
				uint8_t message_class, uint8_t message_id,
				uint8_t payload[], uint16_t payload_size)
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

void u_blox_get_cfg_prt_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size, enum port_number port_id)
{
	uint16_t payload_size = 1;
	uint8_t payload[payload_size];

	payload[0] = port_id;

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_PRT, payload,
			    payload_size);
}

void u_blox_get_cfg_prt_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, enum port_number port_id,
			    uint32_t baudrate)
{
	uint16_t payload_size = 20;
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

	/* Baud Rate */
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

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_PRT, payload,
			    payload_size);
}

void u_blox_get_cfg_rst_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t reset_mode)
{
	uint16_t payload_size = 4;
	uint8_t payload[payload_size];

	/* navBbrMask. */
	payload[0] = 0x00;
	payload[1] = 0x00;

	/* resetMode. */
	payload[2] = reset_mode;

	/* reserved1. */
	payload[3] = 0x00;

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_RST, payload,
			    payload_size);
}
void u_blox_get_cfg_nav5_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size)
{
	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5, NULL, 0);
}

void u_blox_get_cfg_nav5_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, enum gnss_mode g_mode,
			     enum fix_mode f_mode, int32_t fixed_alt, uint32_t fixed_alt_var,
			     int8_t min_elev, uint16_t p_dop, uint16_t t_dop, uint16_t p_acc,
			     uint16_t t_acc, uint8_t static_hold_thresh, uint8_t dgnss_timeout,
			     uint8_t cno_thresh_num_svs, uint8_t cno_thresh,
			     uint16_t static_hold_max_dist, enum utc_standard utc_strd)
{
	uint16_t payload_size = 36;
	uint8_t payload[payload_size];
	int8_t fixed_alt_le[4] = { 0 };
	uint8_t fixed_alt_var_le[4] = { 0 };
	uint8_t p_dop_le[2] = { 0 };
	uint8_t t_dop_le[2] = { 0 };
	uint8_t p_acc_le[2] = { 0 };
	uint8_t t_acc_le[2] = { 0 };
	uint8_t static_hold_max_dist_le[2] = { 0 };

	TO_LITTLE_ENDIAN(fixed_alt, fixed_alt_le);
	TO_LITTLE_ENDIAN(fixed_alt_var, fixed_alt_var_le);
	TO_LITTLE_ENDIAN(p_dop, p_dop_le);
	TO_LITTLE_ENDIAN(t_dop, t_dop_le);
	TO_LITTLE_ENDIAN(p_acc, p_acc_le);
	TO_LITTLE_ENDIAN(t_acc, t_acc_le);
	TO_LITTLE_ENDIAN(static_hold_max_dist, static_hold_max_dist_le);

	payload[0] = 0xff;
	payload[1] = 0x05;
	payload[2] = g_mode;
	payload[3] = f_mode;
	payload[4] = fixed_alt_le[0];
	payload[5] = fixed_alt_le[1];
	payload[6] = fixed_alt_le[2];
	payload[7] = fixed_alt_le[3];
	payload[8] = fixed_alt_var_le[0];
	payload[9] = fixed_alt_var_le[1];
	payload[10] = fixed_alt_var_le[2];
	payload[11] = fixed_alt_var_le[3];
	payload[12] = min_elev;
	payload[14] = p_dop_le[0];
	payload[15] = p_dop_le[1];
	payload[16] = t_dop_le[0];
	payload[17] = t_dop_le[1];
	payload[18] = p_acc_le[0];
	payload[19] = p_acc_le[1];
	payload[20] = t_acc_le[0];
	payload[21] = t_acc_le[1];
	payload[22] = static_hold_thresh;
	payload[23] = dgnss_timeout;
	payload[24] = cno_thresh_num_svs;
	payload[25] = cno_thresh;
	payload[28] = static_hold_max_dist_le[0];
	payload[29] = static_hold_max_dist_le[1];
	payload[30] = utc_strd;

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5, payload,
			    payload_size);
}

void u_blox_get_cfg_gnss_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size)
{
	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS, NULL, 0);
}

void u_blox_get_cfg_gnss_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t msg_ver,
			     uint8_t num_trk_ch_use, uint8_t *config_gnss, uint16_t config_size)
{
	uint16_t payload_size = (4 + (config_size));
	uint8_t payload[payload_size];

	payload[0] = msg_ver;
	payload[2] = num_trk_ch_use;
	payload[3] = config_size / 8;

	memcpy(payload + 4, config_gnss, config_size);

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS, payload,
			    payload_size);
}

void u_blox_get_cfg_msg_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t msg_id,
			    uint8_t rate)
{
	uint16_t payload_size = 3;
	uint8_t payload[payload_size];

	payload[0] = UBX_CLASS_NMEA;
	payload[1] = msg_id;
	payload[2] = rate;

	u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_MSG, payload,
			    payload_size);
}

