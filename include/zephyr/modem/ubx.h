/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/modem/pipe.h>

#ifndef ZEPHYR_MODEM_UBX_
#define ZEPHYR_MODEM_UBX_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modem Ubx
 * @defgroup modem_ubx Modem Ubx
 * @ingroup modem
 * @{
 */

#define UBX_PREAMBLE_SYNC_CHAR_1	0xB5
#define UBX_PREAMBLE_SYNC_CHAR_2	0x62

#define UBX_FRM_MSG_CLASS_IDX		2
#define UBX_FRM_MSG_ID_IDX		3

#define UBX_FRM_MSG_CLASS_ACK		5
#define UBX_FRM_MSG_ID_ACK		1

#define UBX_FRM_HEADER_SIZE		6
#define UBX_FRM_FOOTER_SIZE		2
#define UBX_FRM_SIZE_WITHOUT_PAYLOAD	UBX_FRM_HEADER_SIZE + UBX_FRM_FOOTER_SIZE
#define UBX_FRM_PAYLOAD_SIZE_L_IDX	4
#define UBX_FRM_PAYLOAD_SIZE_H_IDX	5

#define MODEM_UBX_RETRY_DEFAULT		10

struct modem_ubx_script {
	uint8_t *ubx_frame;
	uint16_t ubx_frame_size;
	uint16_t retry_count;
};

struct modem_ubx {
	void *user_data;

	atomic_t state;

	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint8_t *transmit_buf;
	uint16_t transmit_buf_size;

	uint8_t *work_buf;
	uint16_t work_buf_size;
	uint16_t work_buf_len;
	uint8_t *ubx_response_buf;
	uint16_t ubx_response_buf_size;
	uint16_t ubx_response_buf_len;

	struct modem_pipe *pipe;

	struct k_work send_work;
	struct k_work process_work;
	struct k_sem script_stopped_sem;
	struct k_sem script_running_sem;
	k_timeout_t process_timeout;
};

struct modem_ubx_config {
	void *user_data;

	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint8_t *work_buf;
	uint16_t work_buf_size;
	uint8_t *ubx_response_buf;
	uint16_t ubx_response_buf_size;

	k_timeout_t process_timeout;
};

/**
 * @brief Attach pipe to Modem Ubx
 *
 * @param ubx Modem Ubx instance
 * @param pipe Pipe instance to attach Modem Ubx instance to
 * @returns 0 if successful
 * @returns negative errno code if failure
 * @note Modem Ubx instance is enabled if successful
 */
int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe);

/**
 * @brief Release pipe from Modem Ubx instance
 *
 * @param ubx Modem Ubx instance
 */
void modem_ubx_release(struct modem_ubx *ubx);

/**
 * @brief Initialize Modem Ubx instance
 * @param ubx Modem Ubx instance
 * @param config Configuration which shall be applied to the Modem Ubx instance
 * @note Modem Ubx instance must be attached to a pipe instance
 */
int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config);

int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *frame);
int modem_ubx_run_script_async(struct modem_ubx *ubx, const struct modem_ubx_script *frame);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODEM_UBX_ */
