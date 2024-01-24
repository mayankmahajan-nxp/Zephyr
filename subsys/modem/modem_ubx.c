/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/modem/ubx.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

#define MODEM_UBX_STATE_ATTACHED_BIT		(0)

static void modem_ubx_reset_received_ubx_preamble_sync_chars(struct modem_ubx *ubx)
{
	ubx->ubx_preamble_sync_chars_received = false;
}

static void modem_ubx_reset_parser(struct modem_ubx *ubx)
{
	(void) modem_ubx_reset_received_ubx_preamble_sync_chars(ubx);
}

static int modem_ubx_get_payload_length(struct ubx_frame_t *frame)
{
	uint16_t payload_len = frame->payload_size_high;

	payload_len = payload_len << 8;

	return payload_len | frame->payload_size_low;
}

static int modem_ubx_get_frame_length(struct ubx_frame_t *frame)
{
	return modem_ubx_get_payload_length(frame) + UBX_FRM_SZ_WITHOUT_PAYLOAD;
}

static bool modem_ubx_match_frame_type(struct ubx_frame_t *frame_1, struct ubx_frame_t *frame_2)
{
	if (frame_1->message_class == frame_2->message_class
	    && frame_1->message_id == frame_2->message_id) {
		return true;
	} else {
		return false;
	}
}

static bool modem_ubx_match_frame_full(struct ubx_frame_t *frame_1, struct ubx_frame_t *frame_2)
{
	if (modem_ubx_get_frame_length(frame_1) != modem_ubx_get_frame_length(frame_2)) {
		return false;
	}

	if (memcmp(frame_1, frame_2, modem_ubx_get_frame_length(frame_1)) == 0) {
		return true;
	} else {
		return false;
	}
}

static void modem_ubx_script_init(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	ubx->request = script->request;
	ubx->response = script->response;
	ubx->match = script->match;
}

int modem_ubx_run_script_helper(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	k_sem_reset(&ubx->script_stopped_sem);

	(void) modem_ubx_reset_parser(ubx);

	k_work_submit(&ubx->send_work);

	if (ubx->match == NULL) {
		return 0;
	}

	ret = k_sem_take(&ubx->script_stopped_sem, script->timeout);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret, attempt;

	if (modem_ubx_get_frame_length(script->request) > UBX_FRM_SZ_MAX) {
		return -EFBIG;
	}

	if (atomic_test_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return -EPERM;
	}

	ret = k_sem_take(&ubx->script_running_sem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	(void) modem_ubx_script_init(ubx, script);

	for (attempt = 0; attempt < script->retry_count; ++attempt) {
		ret = modem_ubx_run_script_helper(ubx, script);
		if (ret > -1) {
			LOG_INF("Successfully executed script on attempt: %d.", attempt);
			break;
		} else if (ret == -EPERM) {
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("Failed to execute script successfully. Attempts: %d.", attempt);
		goto unlock;
	}

unlock:
	k_sem_give(&ubx->script_running_sem);

	return ret;
}

static void modem_ubx_pipe_callback(struct modem_pipe *pipe, enum modem_pipe_event event,
				    void *user_data)
{
	struct modem_ubx *ubx = (struct modem_ubx *)user_data;

	if (event == MODEM_PIPE_EVENT_RECEIVE_READY) {
		k_work_submit(&ubx->process_work);
	}
}

static void modem_ubx_send_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, send_work);
	int ret, tx_frame_len;

	tx_frame_len = modem_ubx_get_frame_length(ubx->request);
	ret = modem_pipe_transmit(ubx->pipe, (const uint8_t *) ubx->request, tx_frame_len);
	if (ret < tx_frame_len) {
		LOG_ERR("Ubx frame transmission failed. Returned %d.", ret);
		return;
	}
}

static int modem_ubx_process_received_ubx_frame(struct modem_ubx *ubx)
{
	int ret = -1;
	struct ubx_frame_t *received = (struct ubx_frame_t *) ubx->work_buf;

	if (modem_ubx_match_frame_full(received, ubx->match) == true) {
		k_sem_give(&ubx->script_stopped_sem);
		ret = 0; /* Frame matched successfully. Terminate the script. */
	} else if (modem_ubx_match_frame_type(received, ubx->request) == true) {
		memcpy(ubx->response, ubx->work_buf, ubx->work_buf_len);
		ret = -1; /* Response receveid successfully. Script not ended. */
	}

	(void) modem_ubx_reset_parser(ubx);

	return ret;
}

static int modem_ubx_process_received_byte(struct modem_ubx *ubx, uint8_t byte)
{
	static uint8_t prev_byte;
	static uint16_t rx_ubx_frame_len;

	if (ubx->ubx_preamble_sync_chars_received == false) {
		if (prev_byte == UBX_PREAMBLE_SYNC_CHAR_1 && byte == UBX_PREAMBLE_SYNC_CHAR_2) {
			ubx->ubx_preamble_sync_chars_received = true;
			ubx->work_buf[0] = UBX_PREAMBLE_SYNC_CHAR_1;
			ubx->work_buf[1] = UBX_PREAMBLE_SYNC_CHAR_2;
			ubx->work_buf_len = 2;
		}
	} else {
		ubx->work_buf[ubx->work_buf_len] = byte;
		++ubx->work_buf_len;

		if (ubx->work_buf_len == UBX_FRM_HEADER_SZ) {
			uint16_t rx_ubx_payload_len = ubx->work_buf[UBX_FRM_PAYLOAD_SZ_H_IDX];

			rx_ubx_payload_len = ubx->work_buf[UBX_FRM_PAYLOAD_SZ_H_IDX] << 8;
			rx_ubx_payload_len |= ubx->work_buf[UBX_FRM_PAYLOAD_SZ_L_IDX];

			rx_ubx_frame_len = rx_ubx_payload_len + UBX_FRM_SZ_WITHOUT_PAYLOAD;
		}

		if (ubx->work_buf_len == rx_ubx_frame_len) {
			return modem_ubx_process_received_ubx_frame(ubx);
		}
	}

	prev_byte = byte;

	return -1;
}

static void modem_ubx_process_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, process_work);
	int ret;

	ret = modem_pipe_receive(ubx->pipe, ubx->receive_buf, ubx->receive_buf_size);
	if (ret < 1) {
		return;
	}

	for (int i = 0; i < ret; i++) {
		ret = modem_ubx_process_received_byte(ubx, ubx->receive_buf[i]);
		if (ret == 0) { /* Frame matched successfully. Terminate the script. */
			break;
		}
	}

	k_work_submit(&ubx->process_work);
}

int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe)
{
	if (atomic_test_and_set_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == true) {
		return 0;
	}

	ubx->pipe = pipe;
	(void) modem_pipe_attach(ubx->pipe, modem_ubx_pipe_callback, ubx);
	k_sem_give(&ubx->script_running_sem);

	return 0;
}

void modem_ubx_release(struct modem_ubx *ubx)
{
	struct k_work_sync sync;

	if (atomic_test_and_clear_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return;
	}

	(void) modem_pipe_release(ubx->pipe);
	k_work_cancel_sync(&ubx->send_work, &sync);
	k_work_cancel_sync(&ubx->process_work, &sync);
	k_sem_reset(&ubx->script_stopped_sem);
	k_sem_reset(&ubx->script_running_sem);
	ubx->work_buf_len = 0;
	(void) modem_ubx_reset_parser(ubx);
	ubx->pipe = NULL;
}

int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config)
{
	__ASSERT_NO_MSG(ubx != NULL);
	__ASSERT_NO_MSG(config != NULL);
	__ASSERT_NO_MSG(config->receive_buf != NULL);
	__ASSERT_NO_MSG(config->receive_buf_size > 0);
	__ASSERT_NO_MSG(config->work_buf != NULL);
	__ASSERT_NO_MSG(config->work_buf_size > 0);

	memset(ubx, 0x00, sizeof(*ubx));
	ubx->user_data = config->user_data;

	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->work_buf = config->work_buf;
	ubx->work_buf_size = config->work_buf_size;

	ubx->pipe = NULL;

	k_work_init(&ubx->send_work, modem_ubx_send_handler);
	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);
	k_sem_init(&ubx->script_running_sem, 1, 1);

	return 0;
}
