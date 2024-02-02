/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Based on the file "drivers/gnss/ublox-neo-m8/ublox_neo_m8.c" from pull request #46447
 * (https://github.com/zephyrproject-rtos/zephyr/pull/46447).
 */

#include "gnss_u_blox_protocol.h"

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

static int u_blox_create_frame(uint8_t *ubx_frame, uint16_t ubx_frame_size,
				uint8_t message_class, uint8_t message_id,
				uint16_t payload_size)
{
	uint8_t ckA = 0;
	uint8_t ckB = 0;

	uint16_t ubx_frame_len = U_BLOX_MESSAGE_SIZE_WITHOUT_PAYLOAD + payload_size;

	printk("ubx frame size = %d, ubx frame len = %d.\n", ubx_frame_size, ubx_frame_len);
	if (ubx_frame_size < ubx_frame_len) {
		return -1;
	}

	ubx_frame[0] = 0xb5;
	ubx_frame[1] = 0x62;
	ubx_frame[2] = message_class;
	ubx_frame[3] = message_id;
	ubx_frame[4] = (payload_size & 0xff);
	ubx_frame[5] = (payload_size >> 8);
	// memcpy(&ubx_frame[6], payload, payload_size);

	for (unsigned int i = 2; i < (ubx_frame_len - 2); i++) {
		ckA += ubx_frame[i];
		ckB += ckA;
	}

	ubx_frame[ubx_frame_len - 2] = ckA;
	ubx_frame[ubx_frame_len - 1] = ckB;

	return ubx_frame_len;
}

int u_blox_cfg_prt_get(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_prt_get_data *data)
{
	uint16_t payload_size = 1;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	payload[0] = data->port_id;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_PRT,
			    payload_size);
}

void u_blox_cfg_prt_set_data_default(struct u_blox_cfg_prt_set_data *data) {
	data->port_id = UBX_PORT_NUMBER_UART;
	data->tx_ready_pin_conf = 0U;
	data->port_mode = UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8 | UBX_CFG_PRT_PORT_MODE_PARITY_NONE |
			  UBX_CFG_PRT_PORT_MODE_STOP_BITS_1;
	data->baudrate = u_blox_baudrate[3];
	data->in_proto_mask = UBX_CFG_PRT_IN_PROTO_UBX | UBX_CFG_PRT_IN_PROTO_NMEA |
			      UBX_CFG_PRT_IN_PROTO_RTCM;
	data->out_proto_mask = UBX_CFG_PRT_OUT_PROTO_UBX | UBX_CFG_PRT_OUT_PROTO_NMEA |
			       UBX_CFG_PRT_OUT_PROTO_RTCM3;
	data->flags = 0U;
}

int u_blox_cfg_prt_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_prt_set_data *data)
{
	uint16_t payload_size = 20;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	uint32_t port_mode = UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8 |
			     UBX_CFG_PRT_PORT_MODE_PARITY_NONE |
			     UBX_CFG_PRT_PORT_MODE_STOP_BITS_1;

	/* Port identifier number */
	payload[0] = data->port_id;

	/* Reserved0 */
	payload[1] = 0x00;

	/* TX ready PIN conﬁguration */
	payload[2] = data->tx_ready_pin_conf;
	payload[3] = data->tx_ready_pin_conf >> 8;

	/* Port mode */
	memcpy(payload + 4, (uint8_t *) &port_mode, 4);

	/* Baud Rate */
	payload[8] = data->baudrate;
	payload[9] = data->baudrate >> 8;
	payload[10] = data->baudrate >> 16;
	payload[11] = data->baudrate >> 24;
	for (int i = 0; i < 4; ++i) {
		printk("%d %d\n", payload[8 + i], data->baudrate >> (8 * i));
	}

	/* In Proto Mask = All proto enable */
	payload[12] = data->in_proto_mask;
	payload[13] = data->in_proto_mask >> 8;

	/* Out Proto Mask = All proto enable */
	payload[14] = data->out_proto_mask;
	payload[15] = data->out_proto_mask >> 8;

	/* Flags */
	payload[16] = data->flags;
	payload[17] = data->flags >> 8;

	/* Reserved1 */
	payload[18] = 0x00;
	payload[19] = 0x00;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_PRT,
			    payload_size);
}

int u_blox_cfg_rst_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, uint8_t reset_mode)
{
	uint16_t payload_size = 4;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	/* navBbrMask (0x0000 for Hot start). */
	payload[0] = 0x00;
	payload[1] = 0x00;

	/* resetMode. */
	payload[2] = reset_mode;

	/* reserved1. */
	payload[3] = 0x00;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_RST,
			    payload_size);
}

int u_blox_cfg_nav5_get(uint8_t *ubx_frame, uint16_t ubx_frame_size)
{
	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5, 0);
}

int u_blox_cfg_nav5_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, enum ubx_dynamic_model dyn_model,
			     enum ubx_fix_mode f_mode, int32_t fixed_alt, uint32_t fixed_alt_var,
			     int8_t min_elev, uint16_t p_dop, uint16_t t_dop, uint16_t p_acc,
			     uint16_t t_acc, uint8_t static_hold_thresh, uint8_t dgnss_timeout,
			     uint8_t cno_thresh_num_svs, uint8_t cno_thresh,
			     uint16_t static_hold_max_dist, enum ubx_utc_standard utc_strd)
{
	uint16_t payload_size = 36;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

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

	/* Parameters bitmask (0xFF05 for selecting all parameters) */
	payload[0] = 0xFF;
	payload[1] = 0x05;

	payload[2] = dyn_model;
	payload[3] = f_mode;
	memcpy(payload + 4, fixed_alt_le, 4);
	memcpy(payload + 8, fixed_alt_var_le, 4);
	payload[12] = min_elev;
	memcpy(payload + 14, p_dop_le, 2);
	memcpy(payload + 16, t_dop_le, 2);
	memcpy(payload + 18, p_acc_le, 2);
	memcpy(payload + 20, t_acc_le, 2);
	payload[22] = static_hold_thresh;
	payload[23] = dgnss_timeout;
	payload[24] = cno_thresh_num_svs;
	payload[25] = cno_thresh;
	memcpy(payload + 28, static_hold_max_dist_le, 2);
	payload[30] = utc_strd;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5,
			    payload_size);
}

int u_blox_cfg_gnss_get(uint8_t *ubx_frame, uint16_t ubx_frame_size)
{
	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS, 0);
}

int u_blox_cfg_gnss_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, uint8_t msg_ver,
			     uint8_t num_trk_ch_use, uint8_t *config_gnss, uint16_t config_size)
{
	uint16_t payload_size = (4 + (config_size));
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	payload[0] = msg_ver;
	payload[2] = num_trk_ch_use;
	payload[3] = config_size / 8;

	memcpy(payload + 4, config_gnss, config_size);

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS,
			    payload_size);
}

int u_blox_cfg_msg_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, uint8_t msg_id,
			    uint8_t rate)
{
	uint16_t payload_size = 3;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	payload[0] = UBX_CLASS_NMEA;
	payload[1] = msg_id;
	payload[2] = rate;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_MSG,
			    payload_size);
}
