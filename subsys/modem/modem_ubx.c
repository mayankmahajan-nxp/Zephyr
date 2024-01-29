/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <zephyr/net/ppp.h>
// #include <zephyr/sys/crc.h>
#include <zephyr/modem/ubx.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

bool received_ubx_ack_full = false;
static bool received_ubx_ack_start = false;

int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_frame *frame)
{
	// bool script_is_running;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	// script_is_running =
	// 	atomic_test_and_set_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	// if (script_is_running == true) {
	// 	return -EBUSY;
	// }

	// ubx->pending_script = script;
	k_work_submit(&ubx->send_work);
	return 0;
}

int modem_ubx_transmit(struct modem_ubx *ubx, const struct modem_ubx_frame *frame)
{
	int ret;

	k_sem_reset(&ubx->script_stopped_sem);

	ubx->transmit_buf = frame->ubx_frame;
	ubx->transmit_buf_size = frame->ubx_frame_size;

	received_ubx_ack_start = false; // temp.
	ubx->work_buf_len = 0; // temp.

	ret = modem_ubx_transmit_async(ubx, frame);
	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&ubx->script_stopped_sem, ubx->process_timeout);
	if (ret < 0) {
		return ret;
	} else if (ubx->work_buf[3] == 0) {
		return -1;
	} else {
		return 0;
	}

	// return ubx->script_result == MODEM_UBX_SCRIPT_RESULT_SUCCESS ? 0 : -EAGAIN;
	// return 0;
}

static void modem_ubx_process_received_byte(struct modem_ubx *ubx, uint8_t byte)
{
}

static void modem_ubx_pipe_callback(struct modem_pipe *pipe, enum modem_pipe_event event,
				    void *user_data)
{
	struct modem_ubx *ubx = (struct modem_ubx *)user_data;
	// printk("[%d] ", event);

	if (event == MODEM_PIPE_EVENT_RECEIVE_READY) {
		k_work_submit(&ubx->process_work);
	}
}

static void modem_ubx_send_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, send_work);
	uint8_t byte;
	int ret;

	ret = modem_pipe_transmit(ubx->pipe, ubx->transmit_buf, ubx->transmit_buf_size);
	if (ret < 1) {
		return;
	}
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
		// modem_ubx_process_received_byte(ubx, ubx->receive_buf[i]);

		if (ubx->receive_buf[i] == 0xB5) {
			received_ubx_ack_start = true;
		}

		if (received_ubx_ack_start) {
			ubx->work_buf[ubx->work_buf_len] = ubx->receive_buf[i];
			++ubx->work_buf_len;
		}

		if (ubx->work_buf_len == 10) {
			received_ubx_ack_full = true;
			// for (int i = 0; i < ubx->work_buf_len; ++i)
			// 	printk("%x ", ubx->work_buf[i]);
			k_sem_give(&ubx->script_stopped_sem);
			break;
		}
	}

	k_work_submit(&ubx->process_work);
}

int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe)
{
	// if (atomic_test_and_set_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == true) {
	// 	return 0;
	// }

	modem_pipe_attach(pipe, modem_ubx_pipe_callback, ubx);
	ubx->pipe = pipe;
	return 0;
}

void modem_ubx_release(struct modem_ubx *ubx)
{
	struct k_work_sync sync;

	// if (atomic_test_and_clear_bit(&ubx->state, MODEM_UBX_STATE_ATTACHED_BIT) == false) {
	// 	return;
	// }

	modem_pipe_release(ubx->pipe);
	k_work_cancel_sync(&ubx->send_work, &sync);
	k_work_cancel_sync(&ubx->process_work, &sync);
	k_sem_reset(&ubx->script_stopped_sem);
	ubx->pipe = NULL;
}

int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config)
{
	__ASSERT_NO_MSG(ubx != NULL);
	__ASSERT_NO_MSG(config != NULL);
	__ASSERT_NO_MSG(config->receive_buf != NULL);
	__ASSERT_NO_MSG(config->receive_buf_size > 0);

	ubx->pipe = NULL;
	ubx->user_data = config->user_data;
	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->work_buf = config->work_buf;
	ubx->work_buf_size = config->work_buf_size;
	ubx->process_timeout = config->process_timeout;

	atomic_set(&ubx->state, 0);
	k_work_init(&ubx->send_work, modem_ubx_send_handler);
	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);

	return 0;
}
