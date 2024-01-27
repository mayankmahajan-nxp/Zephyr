/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/modem/pipe.h>

#ifndef ZEPHYR_MODEM_UBX_
#define ZEPHYR_MODEM_UBX_

#ifdef __cplusplus
extern "C" {
#endif

struct modem_ubx;

enum modem_ubx_script_result {
	MODEM_UBX_SCRIPT_RESULT_SUCCESS,
	MODEM_UBX_SCRIPT_RESULT_ABORT,
	MODEM_UBX_SCRIPT_RESULT_TIMEOUT
};


/**
 * @brief Callback called when script ubx is received
 *
 * @param ubx Pointer to ubx instance instance
 * @param result Result of script execution
 * @param user_data Free to use user data set during modem_ubx_init()
 */
typedef void (*modem_ubx_script_callback)(struct modem_ubx *ubx,
					   enum modem_ubx_script_result result, void *user_data);

enum modem_ubx_script_send_state {
	/* No data to send */
	MODEM_UBX_SCRIPT_SEND_STATE_IDLE,
	/* Sending request */
	MODEM_UBX_SCRIPT_SEND_STATE_REQUEST,
	/* Sending delimiter */
	MODEM_UBX_SCRIPT_SEND_STATE_DELIMITER,
};

struct modem_ubx_script {
	uint8_t *ubx_frame;
	uint8_t ubx_frame_size;
};

/**
 * @brief Ubx instance internal context
 * @warning Do not modify any members of this struct directly
 */
struct modem_ubx {
	/* Pipe used to send and receive data */
	struct modem_pipe *pipe;

	/* User data passed with match callbacks */
	void *user_data;

	/* Receive buffer */
	uint8_t *receive_buf;
	uint16_t receive_buf_size;
	uint16_t receive_buf_len;

	/* Work buffer */
	uint8_t work_buf[32];
	uint16_t work_buf_len;

	/* Script execution */
	const struct modem_ubx_script *script;
	const struct modem_ubx_script *pending_script;
	struct k_work script_run_work;
	struct k_work_delayable script_timeout_work;
	struct k_work script_abort_work;
	atomic_t script_state;
	enum modem_ubx_script_result script_result;
	struct k_sem script_stopped_sem;

	/* Script sending */
	uint16_t script_send_request_pos;
	uint16_t script_send_delimiter_pos;
	struct k_work_delayable script_send_work;
	struct k_work_delayable script_send_timeout_work;

	/* Process received data */
	struct k_work_delayable process_work;
	k_timeout_t process_timeout;
};

/**
 * @brief Ubx configuration
 */
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
 * @brief Initialize modem pipe ubx instance
 * @param ubx Ubx instance
 * @param config Configuration which shall be applied to Ubx instance
 * @note Ubx instance must be attached to pipe
 */
int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config);

/**
 * @brief Attach modem ubx instance to pipe
 * @param ubx Ubx instance
 * @param pipe Pipe instance to attach Ubx instance to
 * @returns 0 if successful
 * @returns negative errno code if failure
 * @note Ubx instance is enabled if successful
 */
int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe);

/**
 * @brief Run script asynchronously
 * @param ubx Ubx instance
 * @param script Script to run
 * @returns 0 if script successfully started
 * @returns -EBUSY if a script is currently running
 * @returns -EPERM if modem pipe is not attached
 * @returns -EINVAL if arguments or script is invalid
 * @note Script runs asynchronously until complete or aborted.
 */
int modem_ubx_run_script_async(struct modem_ubx *ubx, const struct modem_ubx_script *script);

/**
 * @brief Run script
 * @param ubx Ubx instance
 * @param script Script to run
 * @returns 0 if successful
 * @returns -EBUSY if a script is currently running
 * @returns -EPERM if modem pipe is not attached
 * @returns -EINVAL if arguments or script is invalid
 * @note Script runs until complete or aborted.
 */
int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script);

/**
 * @brief Abort script
 * @param ubx Ubx instance
 */
void modem_ubx_script_abort(struct modem_ubx *ubx);

/**
 * @brief Release pipe from ubx instance
 * @param ubx Ubx instance
 */
void modem_ubx_release(struct modem_ubx *ubx);

int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_script *script);
int modem_ubx_transmit(struct modem_ubx *ubx, const struct modem_ubx_script *script);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODEM_UBX_ */
