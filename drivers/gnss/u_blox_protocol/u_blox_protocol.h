/*
 * Copyright 2024 NXP
 * Copyright (c) 2022 Abel Sensors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include "u_blox_protocol_defines.h"

#ifndef ZEPHYR_U_BLOX_PROTOCOL_
#define ZEPHYR_U_BLOX_PROTOCOL_

#define U_BLOX_BAUDRATE_COUNT	8

#define U_BLOX_MESSAGE_LEN_MAX	264
#define U_BLOX_PAYLOAD_LEN_MAX	256

#define U_BLOX_CFG_PRT_WAIT_MS	4000
#define U_BLOX_CFG_RST_WAIT_MS	8000

extern const uint32_t u_blox_baudrate[U_BLOX_BAUDRATE_COUNT];

#define TO_LITTLE_ENDIAN(data, b)			\
	for (int j = 0; j < sizeof(data); j++) {	\
		b[j] = (data >> (j * 8)) & 0xFF;	\
	}

void u_blox_get_cfg_prt_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size,
			    enum port_number port_id);
void u_blox_get_cfg_prt_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size,
			    enum port_number port_id, uint32_t baudrate);
void u_blox_get_cfg_rst_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t reset_mode);
void u_blox_get_cfg_nav5_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size);
void u_blox_get_cfg_nav5_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, enum gnss_mode g_mode,
			 enum fix_mode f_mode, int32_t fixed_alt, uint32_t fixed_alt_var,
			 int8_t min_elev, uint16_t p_dop, uint16_t t_dop, uint16_t p_acc,
			 uint16_t t_acc, uint8_t static_hold_thresh, uint8_t dgnss_timeout,
			 uint8_t cno_thresh_num_svs, uint8_t cno_thresh,
			 uint16_t static_hold_max_dist, enum utc_standard utc_strd);
void u_blox_get_cfg_gnss_get(uint8_t ubx_frame[], uint16_t *ubx_frame_size);
void u_blox_get_cfg_gnss_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t msg_ver,
			     uint8_t num_trk_ch_use, uint8_t *config, uint16_t config_size);
void u_blox_get_cfg_msg_set(uint8_t ubx_frame[], uint16_t *ubx_frame_size, uint8_t msg_id,
			    uint8_t rate);

#endif /* ZEPHYR_U_BLOX_PROTOCOL_ */
