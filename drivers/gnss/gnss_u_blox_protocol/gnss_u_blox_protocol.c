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

void u_blox_cfg_prt_get_data_default(struct u_blox_cfg_prt_get_data *data) {
	data->port_id = UBX_PORT_NUMBER_UART;
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
	data->tx_ready_pin_conf = 0x0000;
	data->port_mode = UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8 | UBX_CFG_PRT_PORT_MODE_PARITY_NONE |
			  UBX_CFG_PRT_PORT_MODE_STOP_BITS_1;
	data->baudrate = u_blox_baudrate[3];
	data->in_proto_mask = UBX_CFG_PRT_IN_PROTO_UBX | UBX_CFG_PRT_IN_PROTO_NMEA |
			      UBX_CFG_PRT_IN_PROTO_RTCM;
	data->out_proto_mask = UBX_CFG_PRT_OUT_PROTO_UBX | UBX_CFG_PRT_OUT_PROTO_NMEA |
			       UBX_CFG_PRT_OUT_PROTO_RTCM3;
	data->flags = 0x0000;
}

int u_blox_cfg_prt_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_prt_set_data *data)
{
	uint16_t payload_size = 20;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	/* Port identifier number */
	payload[0] = data->port_id;

	/* Reserved0 */
	payload[1] = 0x00;

	/* TX ready PIN conﬁguration */
	memcpy(payload + 2, &(data->tx_ready_pin_conf), 2);

	/* Port mode */
	memcpy(payload + 4, &(data->port_mode), 4);

	/* Baud Rate */
	memcpy(payload + 8, &(data->baudrate), 4);

	/* In Proto Mask = All proto enable */
	memcpy(payload + 12, &(data->in_proto_mask), 2);

	/* Out Proto Mask = All proto enable */
	memcpy(payload + 14, &(data->out_proto_mask), 2);

	/* Flags */
	memcpy(payload + 16, &(data->flags), 2);

	/* Reserved1 */
	payload[18] = 0x00;
	payload[19] = 0x00;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_PRT, payload_size);
}

void u_blox_cfg_rst_set_data_default(struct u_blox_cfg_rst_set_data *data) {
	data->nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	data->reset_mode = UBX_CFG_RST_RESET_MODE_CONTROLLED_SOFT_RESET;
}

int u_blox_cfg_rst_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_rst_set_data *data)
{
	uint16_t payload_size = 4;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	/* navBbrMask. */
	memcpy(payload, &(data->nav_bbr_mask), 2);

	/* resetMode. */
	payload[2] = data->reset_mode;

	/* reserved1. */
	payload[3] = 0x00;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_RST,
			    payload_size);
}

int u_blox_cfg_nav5_get(uint8_t *ubx_frame, uint16_t ubx_frame_size)
{
	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5, 0);
}

void u_blox_cfg_nav5_set_data_default(struct u_blox_cfg_nav5_set_data *data) {
	data->mask = 0x05FF;
	data->dyn_model = UBX_DYN_MODEL_Portable;

	data->fix_mode = UBX_FIX_AutoFix;

	data->fixed_alt = 0;
	data->fixed_alt_var = 1;

	data->min_elev = 5;
	data->dr_limit = 3;

	data->p_dop = 100;
	data->t_dop = 100;
	data->p_acc = 100;
	data->t_acc = 350;

	data->static_hold_threshold = 0;
	data->dgnss_timeout = 60;
	data->cno_threshold_num_svs = 0;
	data->cno_threshold = 0;

	data->static_hold_dist_threshold = 0;
	data->utc_standard = UBX_UTC_AutoUTC;
}

int u_blox_cfg_nav5_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_nav5_set_data *data)
{
	uint16_t payload_size = 36;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	memcpy(payload, &(data->mask), 2);

	memcpy(payload + 2, &(data->dyn_model), 1);
	memcpy(payload + 3, &(data->fix_mode), 1);

	memcpy(payload + 4, &(data->fixed_alt), 4);
	memcpy(payload + 8, &(data->fixed_alt_var), 4);
	payload[12] = data->min_elev;
	memcpy(payload + 14, &(data->p_dop), 2);
	memcpy(payload + 16, &(data->t_dop), 2);
	memcpy(payload + 18, &(data->p_acc), 2);
	memcpy(payload + 20, &(data->t_acc), 2);
	payload[22] = data->static_hold_threshold;
	payload[23] = data->dgnss_timeout;
	payload[24] = data->cno_threshold_num_svs;
	payload[25] = data->cno_threshold;
	memcpy(payload + 28, &(data->static_hold_dist_threshold), 2);
	payload[30] = data->utc_standard;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_NAV5,
			    payload_size);
}

int u_blox_cfg_gnss_get(uint8_t *ubx_frame, uint16_t ubx_frame_size)
{
	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS, 0);
}

static struct u_blox_cfg_gnss_set_data_config_block u_blox_cfg_gnss_set_data_config_block_default = {
	.gnssId = 0x00,
	.num_res_trk_ch = 0x08,
	.max_num_trk_ch = 0x16,
	.reserved0 = 0x00,
	.flags = 0x00000000,
};

void u_blox_cfg_gnss_set_data_default(struct u_blox_cfg_gnss_set_data *data, struct u_blox_cfg_gnss_set_data_config_block *cfg_blocks, uint8_t num_cfg_blocks)
{
	data->msg_ver = 0x00;
	data->num_trk_ch_hw = 0x20;
	data->num_trk_ch_use = 0x20;
	data->num_config_blocks = num_cfg_blocks;
	data->config_blocks = cfg_blocks;

	for (int i = 0; i < num_cfg_blocks; ++i) {
		data->config_blocks[i] = u_blox_cfg_gnss_set_data_config_block_default;
	}
}

int u_blox_cfg_gnss_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_gnss_set_data *data)
{
	uint16_t payload_size = (4 + (data->num_config_blocks * sizeof(struct u_blox_cfg_gnss_set_data_config_block)));
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	payload[0] = data->msg_ver;
	payload[1] = data->num_trk_ch_hw;
	payload[2] = data->num_trk_ch_use;
	payload[3] = data->num_config_blocks;

	memcpy(payload + 4, data->config_blocks, data->num_config_blocks * sizeof(struct u_blox_cfg_gnss_set_data_config_block));

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_GNSS,
			    payload_size);
}

void u_blox_cfg_msg_set_data_default(struct u_blox_cfg_msg_set_data *data)
{
	data->message_class = UBX_CLASS_NMEA;
	data->message_id = UBX_NMEA_GGA;
	data->rate = 1;
}

int u_blox_cfg_msg_set(uint8_t *ubx_frame, uint16_t ubx_frame_size, struct u_blox_cfg_msg_set_data *data)
{
	uint16_t payload_size = 3;
	uint8_t *payload = ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE;

	payload[0] = data->message_class;
	payload[1] = data->message_id;
	payload[2] = data->rate;

	return u_blox_create_frame(ubx_frame, ubx_frame_size, UBX_CLASS_CFG, UBX_CFG_MSG,
			    payload_size);
}
