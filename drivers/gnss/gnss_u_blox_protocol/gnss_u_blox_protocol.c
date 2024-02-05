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

int u_blox_validate_frame(uint16_t ubx_frame_size, uint8_t message_class, uint8_t message_id,
			  uint16_t payload_size)
{
	if (ubx_frame_size > U_BLOX_FRM_SZ_MAX ||
	    ubx_frame_size < U_BLOX_FRM_SZ_WITHOUT_PAYLOAD ||
	    ubx_frame_size < U_BLOX_FRM_SZ_WITHOUT_PAYLOAD + payload_size) {
		return -1;
	}

	if (payload_size == 0) {
		return 0;
	}

	uint16_t payload_size_expected;
	switch (message_class) {
	case UBX_CLASS_CFG:
		switch (message_id) {
		case UBX_CFG_PRT:
			if (payload_size == UBX_CFG_PRT_POLL_PAYLOAD_SZ ||
			    payload_size == UBX_CFG_PRT_SET_PAYLOAD_SZ) {
				return 0;
			} else {
				return -1;
			}
			break;
		case UBX_CFG_RST:
			payload_size_expected = UBX_CFG_RST_PAYLOAD_SZ;
			break;
		case UBX_CFG_NAV5:
			payload_size_expected = UBX_CFG_NAV5_PAYLOAD_SZ;
			break;
		case UBX_CFG_GNSS:
			if ((payload_size - UBX_CFG_GNSS_PAYLOAD_INIT_SZ) %
			    UBX_CFG_GNSS_PAYLOAD_CFG_BLK_SZ == 0) {
				return 0;
			} else {
				return -1;
			}
			break;
		case UBX_CFG_MSG:
			payload_size_expected = UBX_CFG_MSG_PAYLOAD_SZ;
			break;
		default: return -1;
		}
		break;
	default: return -1;
	}

	if (payload_size == payload_size_expected) {
		return 0;
	} else {
		return -1;
	}
}

int u_blox_create_frame(uint8_t *ubx_frame, uint16_t ubx_frame_size,
			uint8_t message_class, uint8_t message_id,
			const void *const data, uint16_t payload_size)
{
	if (u_blox_validate_frame(ubx_frame_size, message_class, message_id, payload_size)) {
		return -1;
	}

	ubx_frame[U_BLOX_PREAMBLE_SYNC_CHAR_1_IDX] = U_BLOX_PREAMBLE_SYNC_CHAR_1;
	ubx_frame[U_BLOX_PREAMBLE_SYNC_CHAR_2_IDX] = U_BLOX_PREAMBLE_SYNC_CHAR_2;
	ubx_frame[U_BLOX_FRM_MSG_CLASS_IDX] = message_class;
	ubx_frame[U_BLOX_FRM_MSG_ID_IDX] = message_id;
	memcpy(ubx_frame + U_BLOX_FRM_PAYLOAD_SZ_L_IDX, (uint8_t *) &payload_size,
	       sizeof(payload_size));

	memcpy(ubx_frame + U_BLOX_FRM_PAYLOAD_IDX, (uint8_t *) data, payload_size);

	uint16_t ubx_frame_len = payload_size + U_BLOX_FRM_SZ_WITHOUT_PAYLOAD;

	uint8_t ckA = 0, ckB = 0;
	for (unsigned int i = U_BLOX_CHECKSUM_START_IDX;
	     i < (ubx_frame_len - U_BLOX_CHECKSUM_STOP_IDX_FROM_END); i++) {
		ckA += ubx_frame[i];
		ckB += ckA;
	}

	ubx_frame[ubx_frame_len - U_BLOX_CHECKSUM_A_IDX_FROM_END] = ckA;
	ubx_frame[ubx_frame_len - U_BLOX_CHECKSUM_B_IDX_FROM_END] = ckB;

	return ubx_frame_len;
}

void u_blox_cfg_prt_poll_data_default(struct u_blox_cfg_prt_poll_data *const data) {
	data->port_id = UBX_PORT_NUMBER_UART;
}

void u_blox_cfg_prt_set_data_default(struct u_blox_cfg_prt_set_data *const data) {
	data->port_id = UBX_PORT_NUMBER_UART;
	data->reserved0 = 0x00;
	data->tx_ready_pin_conf = 0x0000;
	data->port_mode = UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8 | UBX_CFG_PRT_PORT_MODE_PARITY_NONE |
			  UBX_CFG_PRT_PORT_MODE_STOP_BITS_1;
	data->baudrate = u_blox_baudrate[3];
	data->in_proto_mask = UBX_CFG_PRT_IN_PROTO_UBX | UBX_CFG_PRT_IN_PROTO_NMEA |
			      UBX_CFG_PRT_IN_PROTO_RTCM;
	data->out_proto_mask = UBX_CFG_PRT_OUT_PROTO_UBX | UBX_CFG_PRT_OUT_PROTO_NMEA |
			       UBX_CFG_PRT_OUT_PROTO_RTCM3;
	data->flags = 0x0000;
	data->reserved1 = 0x0000;
}

void u_blox_cfg_rst_data_default(struct u_blox_cfg_rst_data *const data) {
	data->nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	data->reset_mode = UBX_CFG_RST_RESET_MODE_CONTROLLED_SOFT_RESET;
	data->reserved0 = 0x00;
}

void u_blox_cfg_nav5_data_default(struct u_blox_cfg_nav5_data *const data) {
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

	data->reserved0 = 0x0000;

	data->static_hold_dist_threshold = 0;
	data->utc_standard = UBX_UTC_AutoUTC;
}

static struct u_blox_cfg_gnss_data_config_block u_blox_cfg_gnss_data_config_block_default =
{
	.gnssId = UBX_GNSS_ID_GPS,
	.num_res_trk_ch = 0x08,
	.max_num_trk_ch = 0x16,
	.reserved0 = U_BLOX_CFG_GNSS_DATA_RESERVED0,
	.flags = U_BLOX_CFG_GNSS_DATA_CNF_BLK_FLAG_ENABLE |
		 U_BLOX_CFG_GNSS_DATA_CNF_BLK_FLAG_SGN_CNF_MASK_GPS_L1C_A,
};

void u_blox_cfg_gnss_data_default(struct u_blox_cfg_gnss_data *data)
{
	data->msg_ver = U_BLOX_CFG_GNSS_DATA_MSG_VER;
	data->num_trk_ch_hw = U_BLOX_CFG_GNSS_DATA_NUM_TRK_CH_HW_DEFAULT;
	data->num_trk_ch_use = U_BLOX_CFG_GNSS_DATA_NUM_TRK_CH_USE_DEFAULT;

	for (int i = 0; i < data->num_config_blocks; ++i) {
		data->config_blocks[i] = u_blox_cfg_gnss_data_config_block_default;
	}
}

void u_blox_cfg_msg_data_default(struct u_blox_cfg_msg_data *const data)
{
	data->message_class = UBX_CLASS_NMEA;
	data->message_id = UBX_NMEA_GGA;
	data->rate = U_BLOX_CFG_MSG_DATA_RATE_DEFAULT;
}
