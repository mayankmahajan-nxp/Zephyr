/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Referred from the files:
 * "zephyr/subsys/modem/modem_chat.c" and "zephyr/subsys/modem/modem_ppp.c".
 */

#include <zephyr/modem/ubx.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

#define MODEM_UBX_STATE_ATTACHED_BIT		(0)

static bool received_ubx_preamble_sync_chars;
// static bool received_ubx_frame_get_response;
// static bool received_ubx_frame_preamble_sync_char_2;

void modem_ubx_reset_parser(struct modem_ubx *ubx)
{
	received_ubx_preamble_sync_chars = false;
	// received_ubx_frame_get_response = false;
	// received_ubx_frame_preamble_sync_char_1 = false;
	// received_ubx_frame_preamble_sync_char_2 = false;
	ubx->work_buf_len = 0;
	ubx->supplementary_buf_len = 0;
}

int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	k_sem_reset(&ubx->script_stopped_sem);

	ubx->transmit_buf = script->ubx_frame;
	ubx->transmit_buf_size = *script->ubx_frame_size;

	modem_ubx_reset_parser(ubx);
	// received_ubx_frame_preamble_sync_char_1 = false;
	// received_ubx_frame_preamble_sync_char_2 = false;
	// ubx->work_buf_len = 0;
	// ubx->supplementary_buf_len = 0;

	k_work_submit(&ubx->send_work);

	ret = k_sem_take(&ubx->script_stopped_sem, ubx->process_timeout);

	if (ubx->supplementary_buf != 0) {
		memcpy(script->ubx_frame, ubx->supplementary_buf, ubx->supplementary_buf_len);
		*script->ubx_frame_size = ubx->supplementary_buf_len;
	}

	if (ret < 0) {
		return ret;
	} else if (ubx->work_buf[UBX_ACK_IDX] == 0) {
		return -1;
	} else {
		return 0;
	}
}

int modem_ubx_transmit(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	if (atomic_test_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return -EPERM;
	}

	ret = k_sem_take(&ubx->script_running_sem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < script->retry_count; ++i) {
		ret = modem_ubx_transmit_async(ubx, script);
		if (ret == 0) {
			LOG_INF("success on attempt: %d.", i);
			break;
		}
	}

	if (ret < 0) {
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

	ret = modem_pipe_transmit(ubx->pipe, ubx->transmit_buf, ubx->transmit_buf_size);
	if (ret < ubx->transmit_buf_size) {
		LOG_ERR("modem_pipe_transmit failed. returned %d.", ret);
		return;
	}
}

static int modem_ubx_process_received_ubx_frame(struct modem_ubx *ubx)
{
	if (ubx->work_buf[UBX_FRM_CLASS_IDX] == UBX_FRM_CLASS_ACK) {
		k_sem_give(&ubx->script_stopped_sem);
		return 0;
	}

	memcpy(ubx->supplementary_buf, ubx->work_buf, ubx->work_buf_len);
	ubx->supplementary_buf_len = ubx->work_buf_len;

	// received_ubx_frame_preamble_sync_char_1 = false;
	// received_ubx_frame_preamble_sync_char_2 = false;
	received_ubx_preamble_sync_chars = false;

	return -1;
}

static int modem_ubx_process_received_byte(struct modem_ubx *ubx, uint8_t byte)
{
	static uint8_t byte_prev;
	static uint16_t received_ubx_frame_len;

	if (received_ubx_preamble_sync_chars == false) {
		if (byte_prev == UBX_PREAMBLE_SYNC_CHAR_1 && byte == UBX_PREAMBLE_SYNC_CHAR_2) {
			received_ubx_preamble_sync_chars = true;
			ubx->work_buf[0] = UBX_PREAMBLE_SYNC_CHAR_1;
			ubx->work_buf[1] = UBX_PREAMBLE_SYNC_CHAR_2;
			ubx->work_buf_len = 2;
		}
	} else {
		ubx->work_buf[ubx->work_buf_len] = byte;
		++ubx->work_buf_len;

		if (ubx->work_buf_len == UBX_FRM_HEADER_LEN) {
			received_ubx_frame_len = (ubx->work_buf[UBX_FRM_PAYLOAD_LEN_L_IDX] |
						 ubx->work_buf[UBX_FRM_PAYLOAD_LEN_H_IDX] << 8) +
						 UBX_FRM_LEN_WITHOUT_PAYLOAD;
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
	ubx->supplementary_buf_len = 0;
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
	__ASSERT_NO_MSG(config->supplementary_buf != NULL);
	__ASSERT_NO_MSG(config->supplementary_buf_size > 0);

	memset(ubx, 0x00, sizeof(*ubx));
	ubx->user_data = config->user_data;

	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->work_buf = config->work_buf;
	ubx->work_buf_size = config->work_buf_size;
	ubx->supplementary_buf = config->supplementary_buf;
	ubx->supplementary_buf_size = config->supplementary_buf_size;

	ubx->pipe = NULL;

	k_work_init(&ubx->send_work, modem_ubx_send_handler);
	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);
	k_sem_init(&ubx->script_running_sem, 1, 1);
	ubx->process_timeout = config->process_timeout;

	return 0;
}
