/*
 * Copyright (c) 2022 Trackunit Corporation
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/modem/ubx.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

#define MODEM_UBX_STATE_ATTACHED_BIT		(0)
// #define MODEM_UBX_SCRIPT_STATE_RUNNING_BIT	(0)

static bool received_ubx_ack_start_1 = false;
static bool received_ubx_ack_start_2 = false;

int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;
	// bool script_is_running;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	k_sem_reset(&ubx->script_stopped_sem);

	ubx->transmit_buf = script->ubx_frame;
	ubx->transmit_buf_size = *script->ubx_frame_size;

	received_ubx_ack_start_1 = false; // temp.
	received_ubx_ack_start_2 = false; // temp.
	ubx->work_buf_len = 0; // temp.
	ubx->supplementary_buf_len = 0; // temp.

	// script_is_running =
	// 	atomic_test_and_set_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	// if (script_is_running == true) {
	// 	return -EBUSY;
	// }

	k_work_submit(&ubx->send_work);

	ret = k_sem_take(&ubx->script_stopped_sem, ubx->process_timeout);

	if (ubx->supplementary_buf != 0) {
		memcpy(script->ubx_frame, ubx->supplementary_buf, ubx->supplementary_buf_len);
		*script->ubx_frame_size = ubx->supplementary_buf_len;
	}

	if (ret < 0) {
		return ret;
	} else if (ubx->work_buf[3] == 0) {
		return -1;
	} else {
		return 0;
	}
}

int modem_ubx_transmit(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	ret = k_sem_take(&ubx->script_running_sem, K_FOREVER); // temp. could add a reasonable timeout here.
	if (ret < 0) {
		return ret;
	}

	if (atomic_test_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
		return -EPERM;
	}

	for (int i = 0; i < script->retry_count; ++i) {
		ret = modem_ubx_transmit_async(ubx, script);
		if (ret == 0) {
			printk("success on attempt: %d.\n", i);
			break;
		}
	}
	if (ret < 0) {
		goto out;
	}

	// return ubx->script_result == MODEM_UBX_SCRIPT_RESULT_SUCCESS ? 0 : -EAGAIN;
	// return 0;

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
		LOG_ERR("modem_pipe_transmit failed %d.", ret);
		return;
	}
}

static uint16_t received_ubx_msg_len = 0;

static int modem_ubx_process_received_ubx(struct modem_ubx *ubx)
{
	if (ubx->work_buf[2] == 5) {
		k_sem_give(&ubx->script_stopped_sem);
		return 0;
	} else {
		memcpy(ubx->supplementary_buf, ubx->work_buf, ubx->work_buf_len);
		ubx->supplementary_buf_len = ubx->work_buf_len;

		received_ubx_ack_start_1 = false;
		received_ubx_ack_start_2 = false;

		return -1;
	}
}

static int modem_ubx_process_received_byte(struct modem_ubx *ubx, uint8_t byte)
{
	if (!received_ubx_ack_start_1 || !received_ubx_ack_start_2) {
		if (byte == 0xB5) {
			received_ubx_ack_start_1 = true;
		}
		if (byte == 0x62 && received_ubx_ack_start_1) {
			received_ubx_ack_start_2 = true;
			ubx->work_buf[0] = 0xB5;
			ubx->work_buf[1] = 0x62;
			ubx->work_buf_len = 2;
		}
	} else {
		if (received_ubx_ack_start_1 && received_ubx_ack_start_2) {
			ubx->work_buf[ubx->work_buf_len] = byte;
			++ubx->work_buf_len;
		}

		if (ubx->work_buf_len == 6) {
			received_ubx_msg_len = (ubx->work_buf[4] | ubx->work_buf[5] << 8) + 8;
		}

		if (ubx->work_buf_len == received_ubx_msg_len) {
			return modem_ubx_process_received_ubx(ubx);
		}
	}

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

	// atomic_set(&ubx->state, 0);
	k_work_init(&ubx->send_work, modem_ubx_send_handler);
	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);
	k_sem_init(&ubx->script_running_sem, 1, 1);
	ubx->process_timeout = config->process_timeout;

	return 0;
}
