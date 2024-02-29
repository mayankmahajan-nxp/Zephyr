/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/modem/pipe.h>

// #include <zephyr/drivers/gnss/gnss_u_blox_protocol.h>

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

#define UBX_FRM_HEADER_SZ			6
#define UBX_FRM_FOOTER_SZ			2
#define UBX_FRM_SZ_WITHOUT_PAYLOAD		UBX_FRM_HEADER_SZ + UBX_FRM_FOOTER_SZ
#define UBX_FRM_SZ(payload_size)		payload_size + UBX_FRM_SZ_WITHOUT_PAYLOAD

#define UBX_PREAMBLE_SYNC_CHAR_1		0xB5
#define UBX_PREAMBLE_SYNC_CHAR_2		0x62

#define UBX_PREAMBLE_SYNC_CHAR_1_IDX		0
#define UBX_PREAMBLE_SYNC_CHAR_2_IDX		1
#define UBX_FRM_MSG_CLASS_IDX			2
#define UBX_FRM_MSG_ID_IDX			3
#define UBX_FRM_PAYLOAD_SZ_L_IDX		4
#define UBX_FRM_PAYLOAD_SZ_H_IDX		5
#define UBX_FRM_PAYLOAD_IDX			6

#define UBX_CHECKSUM_START_IDX			2
#define UBX_CHECKSUM_STOP_IDX_FROM_END		2
#define UBX_CHECKSUM_A_IDX_FROM_END		2
#define UBX_CHECKSUM_B_IDX_FROM_END		1

#define UBX_FRM_SZ_MAX				264
#define UBX_PAYLOAD_SZ_MAX			256
#define UBX_FRM_HEADER_SZ			6
#define UBX_FRM_FOOTER_SZ			2
#define UBX_FRM_SZ_WO_PAYLOAD			UBX_FRM_HEADER_SZ + UBX_FRM_FOOTER_SZ

#define UBX_MSG_CLASS_ACK		0x05
#define UBX_ACK_MSG_ID_ACK		0x01
#define UBX_ACK_MSG_ID_NAK		0x00

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
 * @details For each ubx frame sent, the device responds with a UBX-ACK frame (class = 0x05).
 *	In case a "get" or "poll" ubx frame (used to get configuration of the device) is sent,
 *	the device responds with two back-to-back frames - first with the exact same ubx frame
 *	whose payload contains the desired configuration; then sends the UBX-ACK frame.
 *	In case a "set" ubx frame (used to set configuration of the device) is sent, the device
 *	only responds with a UBX-ACK frame.
 *	Ex: if "set" variant of UBX-CFG-GNSS was sent, then the device responds with UBX-ACK only.
 *	Ex: if "get" variant of UBX-CFG-GNSS was sent, then the device responds with UBX-CFG-GNSS
 *	then sends a UBX-ACK message.
 *
 *	The message id of the UBX-ACK received determines whether the device acknowledged or not
 *	acknowledged the ubx frame that was sent to it.
 *
 *	This function writes the ubx frame in the script and reads it's response from the device.
 *	If a "get" response was received, then it's written back to the ubx_frame of the script,
 *	so that the caller could retrieve the information that it required.
 * @param ubx Modem Ubx instance
 * @param script Script (containing the ubx frame) to be executed
 * @note The length of ubx frame in the script should not exceed UBX_FRM_SZ_MAX
 * @note Modem Ubx instance must be attached to a pipe instance
 * @returns 0 if device acknowledged via UBX-ACK and no "get" response was received
 * @returns positive integer denoting the length of "get" response that was received
 * @returns negative errno code if failure
 */
int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODEM_UBX_ */
