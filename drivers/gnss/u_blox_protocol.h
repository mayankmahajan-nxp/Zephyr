/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

static void u_blox_create_frame(uint8_t ubx_frame[],
			       uint8_t *ubx_frame_size, uint8_t message_class,
			       uint8_t message_id, uint8_t payload[], uint16_t payload_size);

static void u_blox_get_cfg_prt(uint8_t ubx_frame[],
				uint8_t *ubx_frame_size, uint8_t port_id, uint32_t baudrate);
static void u_blox_get_cfg_rst(uint8_t ubx_frame[],
				uint8_t *ubx_frame_size, uint8_t reset_mode);
