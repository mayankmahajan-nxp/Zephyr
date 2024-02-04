/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Based on the file "drivers/gnss/ublox-neo-m8/ublox_neo_m8.c" from pull request #46447
 * (https://github.com/zephyrproject-rtos/zephyr/pull/46447).
 */

#include "gnss_u_blox_protocol.h"

static int u_blox_cfg_prt_get(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_prt_get_data *const data);
static int u_blox_cfg_prt_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_prt_set_data *const data);
static int u_blox_cfg_rst_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_rst_set_data *const data);
static int u_blox_cfg_nav5_set(uint8_t *payload, uint16_t payload_size,
			       const struct u_blox_cfg_nav5_set_data *const data);
static int u_blox_cfg_gnss_set(uint8_t *payload, uint16_t payload_size,
			       const struct u_blox_cfg_gnss_set_data *const data);
static int u_blox_cfg_msg_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_msg_set_data *const data);

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

int u_blox_create_payload(uint8_t *payload, uint16_t payload_size,
			  uint8_t message_class, uint8_t message_id,
			  const void *const data)
{
	switch (message_class) {
	case UBX_CLASS_CFG:
		switch (message_id) {
		case UBX_CFG_PRT:
			switch (payload_size) {
			case UBX_CFG_PRT_GET_PAYLOAD_SIZE: return u_blox_cfg_prt_get(payload, payload_size, data);
			case UBX_CFG_PRT_SET_PAYLOAD_SIZE: return u_blox_cfg_prt_set(payload, payload_size, data);
			default: return 0;
			}
		case UBX_CFG_RST:
			switch (payload_size) {
			case UBX_CFG_RST_SET_PAYLOAD_SIZE: return u_blox_cfg_rst_set(payload, payload_size, data);
			default: return 0;
			}
		case UBX_CFG_NAV5:
			switch (payload_size) {
			case UBX_CFG_NAV5_SET_PAYLOAD_SIZE: return u_blox_cfg_nav5_set(payload, payload_size, data);
			default: return 0;
			}
		case UBX_CFG_GNSS:
			switch (payload_size) {
			case 0: return 0;
			default: return u_blox_cfg_gnss_set(payload, payload_size, data);
			}
		case UBX_CFG_MSG:
			switch (payload_size) {
			case UBX_CFG_MSG_SET_PAYLOAD_SIZE: return u_blox_cfg_msg_set(payload, payload_size, data);
			default: return 0;
			}
		default: return -1;
		}
	default: return -1;
	}
}

int u_blox_create_frame(uint8_t *ubx_frame, uint16_t ubx_frame_size,
			uint8_t message_class, uint8_t message_id,
			const void *const data, uint16_t payload_size)
{
	int payload_len;
	uint8_t ckA = 0;
	uint8_t ckB = 0;

	if (ubx_frame_size < U_BLOX_MESSAGE_SIZE_WITHOUT_PAYLOAD) {
		return -1;
	} else if (ubx_frame_size - U_BLOX_MESSAGE_SIZE_WITHOUT_PAYLOAD < payload_size) {
		return -1;
	}

	ubx_frame[0] = U_BLOX_PREAMBLE_SYNC_CHAR_1;
	ubx_frame[1] = U_BLOX_PREAMBLE_SYNC_CHAR_2;
	ubx_frame[2] = message_class;
	ubx_frame[3] = message_id;

	payload_len = u_blox_create_payload(ubx_frame + U_BLOX_MESSAGE_HEADER_SIZE, payload_size,
					    message_class, message_id, data);

	if (payload_len < 0) {
		return -1;
	} else if (payload_size < payload_len) {
		return -1;
	}

	ubx_frame[4] = (payload_len & 0xff);
	ubx_frame[5] = (payload_len >> 8);

	uint16_t ubx_frame_len = payload_len + U_BLOX_MESSAGE_SIZE_WITHOUT_PAYLOAD;

	for (unsigned int i = 2; i < (ubx_frame_len - 2); i++) {
		ckA += ubx_frame[i];
		ckB += ckA;
	}

	ubx_frame[ubx_frame_len - 2] = ckA;
	ubx_frame[ubx_frame_len - 1] = ckB;

	return ubx_frame_len;
}

void u_blox_cfg_prt_get_data_default(struct u_blox_cfg_prt_get_data *const data) {
	data->port_id = UBX_PORT_NUMBER_UART;
}

static int u_blox_cfg_prt_get(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_prt_get_data *const data)
{
	uint16_t payload_len = UBX_CFG_PRT_GET_PAYLOAD_SIZE;

	if (payload_size < payload_len) {
		return -1;
	}

	payload[0] = data->port_id;

	return payload_len;
}

void u_blox_cfg_prt_set_data_default(struct u_blox_cfg_prt_set_data *const data) {
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

static int u_blox_cfg_prt_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_prt_set_data *const data)
{
	uint16_t payload_len = UBX_CFG_PRT_SET_PAYLOAD_SIZE;

	if (payload_size < payload_len) {
		return -1;
	}

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

	return payload_len;
}

void u_blox_cfg_rst_set_data_default(struct u_blox_cfg_rst_set_data *const data) {
	data->nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	data->reset_mode = UBX_CFG_RST_RESET_MODE_CONTROLLED_SOFT_RESET;
}

static int u_blox_cfg_rst_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_rst_set_data *const data)
{
	uint16_t payload_len = UBX_CFG_RST_SET_PAYLOAD_SIZE;

	if (payload_size < payload_len) {
		return -1;
	}

	/* navBbrMask. */
	memcpy(payload, &(data->nav_bbr_mask), 2);

	/* resetMode. */
	payload[2] = data->reset_mode;

	/* reserved1. */
	payload[3] = 0x00;

	return payload_len;
}

void u_blox_cfg_nav5_set_data_default(struct u_blox_cfg_nav5_set_data *const data) {
	data->mask = 0x05FF;
	data->dyn_model = UBX_DYN_MODEL_PORTABLE;

	data->fix_mode = UBX_FIX_AUTO_FIX;

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

static int u_blox_cfg_nav5_set(uint8_t *payload, uint16_t payload_size,
			       const struct u_blox_cfg_nav5_set_data *const data)
{
	uint16_t payload_len = UBX_CFG_NAV5_SET_PAYLOAD_SIZE;

	if (payload_size < payload_len) {
		return -1;
	}

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

	return payload_len;
}

static struct u_blox_cfg_gnss_set_data_config_block u_blox_cfg_gnss_set_data_config_block_default =
{
	.gnssId = 0x00,
	.num_res_trk_ch = 0x08,
	.max_num_trk_ch = 0x16,
	.reserved0 = 0x00,
	.flags = 0x00000000,
};

void u_blox_cfg_gnss_set_data_default(struct u_blox_cfg_gnss_set_data *data,
				      struct u_blox_cfg_gnss_set_data_config_block *cfg_blocks,
				      uint8_t num_cfg_blocks)
{
	data->msg_ver = 0x00;
	data->num_trk_ch_hw = 0x31;
	data->num_trk_ch_use = 0x31;
	data->num_config_blocks = num_cfg_blocks;
	data->config_blocks = cfg_blocks;

	for (int i = 0; i < num_cfg_blocks; ++i) {
		data->config_blocks[i] = u_blox_cfg_gnss_set_data_config_block_default;
	}
}

static int u_blox_cfg_gnss_set(uint8_t *payload, uint16_t payload_size,
			       const struct u_blox_cfg_gnss_set_data *const data)
{
	uint16_t payload_len = UBX_CFG_GNSS_SET_PAYLOAD_INIT_SIZE +
			       (data->num_config_blocks * UBX_CFG_GNSS_SET_PAYLOAD_CFG_BLOCK_SIZE);

	if (payload_size < payload_len) {
		return -1;
	}

	payload[0] = data->msg_ver;
	payload[1] = data->num_trk_ch_hw;
	payload[2] = data->num_trk_ch_use;
	payload[3] = data->num_config_blocks;

	memcpy(payload + UBX_CFG_GNSS_SET_PAYLOAD_INIT_SIZE, data->config_blocks,
	       data->num_config_blocks * UBX_CFG_GNSS_SET_PAYLOAD_CFG_BLOCK_SIZE);

	return payload_len;
}

void u_blox_cfg_msg_set_data_default(struct u_blox_cfg_msg_set_data *const data)
{
	data->message_class = UBX_CLASS_NMEA;
	data->message_id = UBX_NMEA_GGA;
	data->rate = 1;
}

static int u_blox_cfg_msg_set(uint8_t *payload, uint16_t payload_size,
			      const struct u_blox_cfg_msg_set_data *const data)
{
	uint16_t payload_len = UBX_CFG_MSG_SET_PAYLOAD_SIZE;

	if (payload_size < payload_len) {
		return -1;
	}

	payload[0] = data->message_class;
	payload[1] = data->message_id;
	payload[2] = data->rate;

	return payload_len;
}
