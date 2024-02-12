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

void modem_ubx_reset_received_ubx_preamble_sync_chars(struct modem_ubx *ubx)
{
	ubx->received_ubx_preamble_sync_chars = false;
}

void modem_ubx_reset_parser(struct modem_ubx *ubx)
{
	modem_ubx_reset_received_ubx_preamble_sync_chars(ubx);
	ubx->received_ubx_get_frame_response = false;
}

int modem_ubx_run_script_helper(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	k_sem_reset(&ubx->script_stopped_sem);

	/* Initialize transfer buffer to the ubx frame in the script. */
	ubx->transfer_buf = script->ubx_frame;
	ubx->transfer_buf_len = script->ubx_frame_size;

	modem_ubx_reset_parser(ubx);

	k_work_submit(&ubx->send_work);

	ret = k_sem_take(&ubx->script_stopped_sem, script->script_timeout);

	if (ret < 0) {
		return ret;
	} else {
		if (ubx->work_buf[UBX_FRM_MSG_ID_IDX] == UBX_ACK_ACK) {
			/* The response of the ubx frame written was acknowledged. */
			if (ubx->received_ubx_get_frame_response == true) {
				/* The response of a "get" ubx frame was received. */
				return ubx->transfer_buf_len;
			} else {
				return 0;
			}
		} else { /* The response of the ubx frame written was not acknowledged. */
			return -1;
		}
	}
}

int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	if (script->ubx_frame_size > UBX_FRM_SZ_MAX) {
		return -EFBIG;
	}

	if (atomic_test_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return -EPERM;
	}

	ret = k_sem_take(&ubx->script_running_sem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	for (int attempt = 0; attempt < script->retry_count; ++attempt) {
		ret = modem_ubx_run_script_helper(ubx, script);
		if (ret > -1) {
			LOG_INF("Successfully executed script on attempt: %d.", attempt);
			break;
		} else if (ret == -EPERM) {
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("Failed to execute script successfully.");
		goto out;
	}

out:
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
	int ret;

	ret = modem_pipe_transmit(ubx->pipe, ubx->transfer_buf, ubx->transfer_buf_len);
	if (ret < ubx->transfer_buf_len) {
		LOG_ERR("Ubx frame transmission failed. Returned %d.", ret);
		return;
	}
}

static int modem_ubx_process_received_ubx_frame(struct modem_ubx *ubx)
{
	// TODO: check validity of ubx frame received and return accordingly.

	if (ubx->work_buf[UBX_FRM_MSG_CLASS_IDX] == UBX_CLASS_ACK) {
		k_sem_give(&ubx->script_stopped_sem);

		return 0;
	} else {
		ubx->received_ubx_get_frame_response = true;

		memcpy(ubx->transfer_buf, ubx->work_buf, ubx->work_buf_len);
		ubx->transfer_buf_len = ubx->work_buf_len;

		modem_ubx_reset_received_ubx_preamble_sync_chars(ubx);

		return -1;
	}
}

static int modem_ubx_process_received_byte(struct modem_ubx *ubx, uint8_t byte)
{
	static uint8_t byte_prev;
	static uint16_t received_ubx_frame_len;

	if (ubx->received_ubx_preamble_sync_chars == false) {
		if (byte_prev == UBX_PREAMBLE_SYNC_CHAR_1 && byte == UBX_PREAMBLE_SYNC_CHAR_2) {
			ubx->received_ubx_preamble_sync_chars = true;
			ubx->work_buf[0] = UBX_PREAMBLE_SYNC_CHAR_1;
			ubx->work_buf[1] = UBX_PREAMBLE_SYNC_CHAR_2;
			ubx->work_buf_len = 2;
		}
	} else {
		ubx->work_buf[ubx->work_buf_len] = byte;
		++ubx->work_buf_len;

		if (ubx->work_buf_len == UBX_FRM_HEADER_SZ) {
			received_ubx_frame_len = (ubx->work_buf[UBX_FRM_PAYLOAD_SZ_L_IDX]
						 | ubx->work_buf[UBX_FRM_PAYLOAD_SZ_H_IDX] << 8)
						 + UBX_FRM_SZ_WITHOUT_PAYLOAD;
		}

		if (ubx->work_buf_len == received_ubx_frame_len) {
			return modem_ubx_process_received_ubx_frame(ubx);
		}
	}

	byte_prev = byte;

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
		if (ret == 0) {
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
	modem_pipe_attach(ubx->pipe, modem_ubx_pipe_callback, ubx);
	k_sem_give(&ubx->script_running_sem);

	return 0;
}

void modem_ubx_release(struct modem_ubx *ubx)
{
	struct k_work_sync sync;

	if (atomic_test_and_clear_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return;
	}

	modem_pipe_release(ubx->pipe);
	k_work_cancel_sync(&ubx->send_work, &sync);
	k_work_cancel_sync(&ubx->process_work, &sync);
	k_sem_reset(&ubx->script_stopped_sem);
	k_sem_reset(&ubx->script_running_sem);
	ubx->work_buf_len = 0;
	modem_ubx_reset_parser(ubx);
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
