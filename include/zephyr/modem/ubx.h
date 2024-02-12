/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/modem/pipe.h>

#include <zephyr/drivers/gnss/gnss_u_blox_protocol.h>

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

#define MODEM_UBX_RETRY_DEFAULT		10

struct modem_ubx_script {
	uint8_t *ubx_frame;
	uint16_t ubx_frame_size;
	uint16_t retry_count;
	k_timeout_t script_timeout;
};

struct modem_ubx {
	void *user_data;

	atomic_t state;

	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	bool received_ubx_preamble_sync_chars;
	bool received_ubx_get_frame_response;

	uint8_t *transfer_buf;
	uint16_t transfer_buf_len;

	uint8_t *work_buf;
	uint16_t work_buf_size;
	uint16_t work_buf_len;

	struct modem_pipe *pipe;

	struct k_work send_work;
	struct k_work process_work;
	struct k_sem script_stopped_sem;
	struct k_sem script_running_sem;
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

/**
 * @brief Writes the ubx frame in the script and reads it's response
 * @details For each ubx frame sent, the device responds with a UBX-ACK message (class = 0x05).
 * 	In case a "get" or "poll" ubx frame (used to get configuration of the device) is written,
 * 	the device first responds with exact same ubx frame whose payload contains the desired
 * 	configuration; then sends the UBX-ACK message.
 * 	Ex: if "set" variant of UBX-CFG-GNSS was sent, then the device responds with UBX-ACK only.
 * 	Ex: if "get" variant of UBX-CFG-GNSS was sent, then the device responds with UBX-CFG-GNSS
 * 	then sends a UBX-ACK message.
 *
 * 	The message id of the UBX-ACK received determines whether the device acknowledged or not
 * 	acknowledged the ubx frame that we wrote to it.
 *
 * 	This function writes the ubx frame in the script and reads it's response from the device.
 * 	If a "get" response was received, then it's written back to the ubx_frame of the script,
 * 	so that the caller could retrieve the information that it required.
 * @param ubx Modem Ubx instance
 * @param script Script (containing the ubx frame) to be executed
 * @note The length of ubx frame in the script should not exceed UBX_FRM_SZ_MAX
 * @note Modem Ubx instance must be attached to a pipe instance
 * @returns 0 if device acknowledged via UBX-ACK and no "get" response was received
 * @returns positive integer denoting the length of "get" response that was received
 * @returns negative errno code if failure
 */
int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODEM_UBX_ */
