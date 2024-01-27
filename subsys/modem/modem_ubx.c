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

int modem_ubx_transmit_async(struct modem_ubx *ubx, const struct modem_ubx_frame *frame)
{
	bool script_is_running;

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

	// k_sem_reset(&ubx->script_stopped_sem);

	ubx->transmit_buf = frame->ubx_frame;
	ubx->transmit_buf_size = frame->ubx_frame_size;

	ret = modem_ubx_transmit_async(ubx, frame);
	if (ret < 0) {
		return ret;
	}

	// ret = k_sem_take(&ubx->script_stopped_sem, K_FOREVER);
	// if (ret < 0) {
	// 	return ret;
	// }

	// return ubx->script_result == MODEM_UBX_SCRIPT_RESULT_SUCCESS ? 0 : -EAGAIN;
	return 0;
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
	if (ret < 0) {
		printk("modem_pipe transaction failed %d.\n", ret);
		return;
	}
}

bool received_ubx_ack = false;

static void modem_ubx_process_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, process_work);
	int ret;

	ret = modem_pipe_receive(ubx->pipe, ubx->receive_buf, ubx->receive_buf_size);
	printk("(%d) ", ret);
	if (ret < 1) {
		// printk("modem_pipe transaction failed %d.\n", ret);
		return;
	}

	for (int i = 0; i < ret; i++) {
		// modem_ubx_process_received_byte(ubx, ubx->receive_buf[i]);
		ubx->work_buf[ubx->work_buf_len] = ubx->receive_buf[i];
		++ubx->work_buf_len;

		if (ubx->receive_buf[i] == 0xB5) {
			received_ubx_ack = true;
			printk("received start of ubx frame.\n");
		}
	}

	if (ubx->work_buf_len == ubx->work_buf_size) {
		for (int i = 0; i < ubx->work_buf_size; ++i)
			printk("%x ", ubx->work_buf[i]);
		ubx->work_buf_len = 0;
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
	ubx->pipe = NULL;
}

int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config)
{
	__ASSERT_NO_MSG(ubx != NULL);
	__ASSERT_NO_MSG(config != NULL);
	__ASSERT_NO_MSG(config->receive_buf != NULL);
	__ASSERT_NO_MSG(config->receive_buf_size > 0);

	// struct modem_ubx *ubx = (struct modem_ubx *)dev->data;

	ubx->pipe = NULL;
	ubx->user_data = config->user_data;
	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->work_buf = config->work_buf;
	ubx->work_buf_size = config->work_buf_size;

	atomic_set(&ubx->state, 0);
	// ring_buf_init(&ubx->transmit_rb, ubx->buf_size, ubx->transmit_buf);
	k_work_init(&ubx->send_work, modem_ubx_send_handler);
	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	// k_fifo_init(&ubx->tx_pkt_fifo);

	return 0;
}
