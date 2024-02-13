/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss/gnss_u_blox_protocol.h>

const uint32_t ubx_baudrate[UBX_BAUDRATE_COUNT] = {
	4800,
	9600,
	19200,
	38400,
	57600,
	115200,
	230400,
	460800,
};

static int ubx_validate_frame(uint16_t ubx_frame_size, uint8_t message_class, uint8_t message_id,
			      uint16_t payload_size)
{
	if (ubx_frame_size > UBX_FRM_SZ_MAX ||
	    ubx_frame_size < UBX_FRM_SZ_WITHOUT_PAYLOAD ||
	    ubx_frame_size < UBX_FRM_SZ_WITHOUT_PAYLOAD + payload_size) {
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

int ubx_create_frame(uint8_t *ubx_frame, uint16_t ubx_frame_size, uint8_t message_class,
		     uint8_t message_id, const void *const data, uint16_t payload_size)
{
	if (ubx_validate_frame(ubx_frame_size, message_class, message_id, payload_size)) {
		return -1;
	}

	struct ubx_frame_t *frame = (struct ubx_frame_t *) ubx_frame;

	frame->preamble_sync_char_1 = UBX_PREAMBLE_SYNC_CHAR_1;
	frame->preamble_sync_char_2 = UBX_PREAMBLE_SYNC_CHAR_2;
	frame->message_class = message_class;
	frame->message_id = message_id;
	frame->payload_size_low = payload_size & 0xFF;
	frame->payload_size_high = payload_size >> 8;

	memcpy(frame->payload_and_checksum, (uint8_t *) data, payload_size);

	uint16_t ubx_frame_len = payload_size + UBX_FRM_SZ_WITHOUT_PAYLOAD;

	uint8_t ckA = 0, ckB = 0;
	for (unsigned int i = UBX_CHECKSUM_START_IDX;
	     i < (ubx_frame_len - UBX_CHECKSUM_STOP_IDX_FROM_END); i++) {
		ckA += ubx_frame[i];
		ckB += ckA;
	}

	frame->payload_and_checksum[payload_size] = ckA;
	frame->payload_and_checksum[payload_size + 1] = ckB;

	return ubx_frame_len;
}

void ubx_cfg_prt_poll_data_default(struct ubx_cfg_prt_poll_data *const data) {
	data->port_id = UBX_PORT_NUMBER_UART;
}

void ubx_cfg_prt_set_data_default(struct ubx_cfg_prt_set_data *const data) {
	data->port_id = UBX_PORT_NUMBER_UART;
	data->reserved0 = UBX_CFG_PRT_RESERVED0;
	data->tx_ready_pin_conf = UBX_CFG_PRT_TX_READY_PIN_CONF_POL_HIGH;
	data->port_mode = UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8 | UBX_CFG_PRT_PORT_MODE_PARITY_NONE |
			  UBX_CFG_PRT_PORT_MODE_STOP_BITS_1;
	data->baudrate = ubx_baudrate[3];
	data->in_proto_mask = UBX_CFG_PRT_IN_PROTO_UBX | UBX_CFG_PRT_IN_PROTO_NMEA |
			      UBX_CFG_PRT_IN_PROTO_RTCM;
	data->out_proto_mask = UBX_CFG_PRT_OUT_PROTO_UBX | UBX_CFG_PRT_OUT_PROTO_NMEA |
			       UBX_CFG_PRT_OUT_PROTO_RTCM3;
	data->flags = UBX_CFG_PRT_FLAGS_DEFAULT;
	data->reserved1 = UBX_CFG_PRT_RESERVED1;
}

void ubx_cfg_rst_data_default(struct ubx_cfg_rst_data *const data) {
	data->nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	data->reset_mode = UBX_CFG_RST_RESET_MODE_CONTROLLED_SOFT_RESET;
	data->reserved0 = UBX_CFG_RST_RESERVED0;
}

void ubx_cfg_nav5_data_default(struct ubx_cfg_nav5_data *const data) {
	data->mask = UBX_CFG_NAV5_MASK_ALL;
	data->dyn_model = UBX_DYN_MODEL_PORTABLE;

	data->fix_mode = UBX_FIX_AUTO_FIX;

	data->fixed_alt = UBX_CFG_NAV5_FIXED_ALT_DEFAULT;
	data->fixed_alt_var = UBX_CFG_NAV5_FIXED_ALT_VAR_DEFAULT;

	data->min_elev = UBX_CFG_NAV5_MIN_ELEV_DEFAULT;
	data->dr_limit = UBX_CFG_NAV5_DR_LIMIT_DEFAULT;

	data->p_dop = UBX_CFG_NAV5_P_DOP_DEFAULT;
	data->t_dop = UBX_CFG_NAV5_T_DOP_DEFAULT;
	data->p_acc = UBX_CFG_NAV5_P_ACC_DEFAULT;
	data->t_acc = UBX_CFG_NAV5_T_ACC_DEFAULT;

	data->static_hold_threshold = UBX_CFG_NAV5_STATIC_HOLD_THRESHOLD_DEFAULT;
	data->dgnss_timeout = UBX_CFG_NAV5_DGNSS_TIMEOUT_DEFAULT;
	data->cno_threshold_num_svs = UBX_CFG_NAV5_CNO_THRESHOLD_NUM_SVS_DEFAULT;
	data->cno_threshold = UBX_CFG_NAV5_CNO_THRESHOLD_DEFAULT;

	data->reserved0 = UBX_CFG_NAV5_RESERVED0;

	data->static_hold_dist_threshold = UBX_CFG_NAV5_STATIC_HOLD_DIST_THRESHOLD;
	data->utc_standard = UBX_CFG_NAV5_UTC_STANDARD_DEFAULT;
}

static struct ubx_cfg_gnss_data_config_block ubx_cfg_gnss_data_config_block_default = {
	.gnss_id = UBX_GNSS_ID_GPS,
	.num_res_trk_ch = 0x00,
	.max_num_trk_ch = 0x00,
	.reserved0 = UBX_CFG_GNSS_RESERVED0,
	.flags = UBX_CFG_GNSS_FLAG_ENABLE | UBX_CFG_GNSS_FLAG_SGN_CNF_GPS_L1C_A,
};

void ubx_cfg_gnss_data_default(struct ubx_cfg_gnss_data *data)
{
	data->msg_ver = UBX_CFG_GNSS_MSG_VER;
	data->num_trk_ch_hw = UBX_CFG_GNSS_NUM_TRK_CH_HW_DEFAULT;
	data->num_trk_ch_use = UBX_CFG_GNSS_NUM_TRK_CH_USE_DEFAULT;

	for (int i = 0; i < data->num_config_blocks; ++i) {
		data->config_blocks[i] = ubx_cfg_gnss_data_config_block_default;
	}
}

void ubx_cfg_msg_data_default(struct ubx_cfg_msg_data *const data)
{
	data->message_class = UBX_CLASS_NMEA;
	data->message_id = UBX_NMEA_GGA;
	data->rate = UBX_CFG_MSG_RATE_DEFAULT;
}

uint16_t ubx_get_payload_size(uint8_t *ubx_frame)
{
	struct ubx_frame_t *frame = (struct ubx_frame_t *) ubx_frame;

	return (uint16_t) ((frame->payload_size_low & 0xFF) | (frame->payload_size_high << 8));
}
