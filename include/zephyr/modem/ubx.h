/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/modem/pipe.h>

#ifndef ZEPHYR_MODEM_UBX_
#define ZEPHYR_MODEM_UBX_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modem UBX
 * @defgroup modem_ubx Modem UBX
 * @ingroup modem
 * @{
 */

struct modem_ubx_frame {
	uint8_t *ubx_frame;
	uint8_t ubx_frame_size;
};
struct modem_ubx {
	/* User data passed with match callbacks */
	void *user_data;

	atomic_t state;

	/* Buffers used for processing partial frames */
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint8_t *transmit_buf;
	uint16_t transmit_buf_size;

	/* Wrapped UBX frames are sent and received through this pipe */
	struct modem_pipe *pipe;

	/* Ring buffer used for transmitting partial UBX frame */
	struct ring_buf transmit_rb;

	struct k_fifo tx_pkt_fifo;

	/* Work */
	struct k_work send_work;
	struct k_work process_work;
};

struct modem_ubx_config {
	/** Free to use user data passed with modem match callbacks */
	void *user_data;
	/** Receive buffer used to store parsed arguments */
	uint8_t *receive_buf;
	/** Size of receive buffer should be longest line + longest match */
	uint16_t receive_buf_size;
	/** Delay from receive ready event to pipe receive occurs */
	k_timeout_t process_timeout;
};

/**
 * @brief Attach pipe to instance and connect
 *
 * @param ubx Modem UBX instance
 * @param pipe Pipe to attach to modem UBX instance
 */
int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe);

/**
 * @brief Release pipe from instance
 *
 * @param ubx Modem UBX instance
 */
void modem_ubx_release(struct modem_ubx *ubx);

/**
 * @brief Initialize modem pipe ubx instance
 * @param ubx Ubx instance
 * @param config Configuration which shall be applied to Ubx instance
 * @note Ubx instance must be attached to pipe
 */
int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config);

int modem_ubx_transmit(struct modem_ubx *ubx, const struct modem_ubx_frame *frame);
int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODEM_UBX_ */
