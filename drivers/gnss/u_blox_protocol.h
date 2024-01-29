/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#ifndef ZEPHYR_U_BLOX_PROTOCOL_
#define ZEPHYR_U_BLOX_PROTOCOL_

#define U_BLOX_M10_BAUDRATE_COUNT	8

extern const uint32_t u_blox_m10_baudrate[U_BLOX_M10_BAUDRATE_COUNT];

void u_blox_get_cfg_prt(uint8_t ubx_frame[], uint8_t *ubx_frame_size, uint8_t port_id,
			uint32_t baudrate);
void u_blox_get_cfg_rst(uint8_t ubx_frame[], uint8_t *ubx_frame_size, uint8_t reset_mode);

#endif /* ZEPHYR_U_BLOX_PROTOCOL_ */
