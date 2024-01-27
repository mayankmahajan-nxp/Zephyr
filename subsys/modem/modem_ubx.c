/*
 * Copyright (c) 2022 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <string.h>

#include <zephyr/modem/ubx.h>

#define MODEM_UBX_SCRIPT_STATE_RUNNING_BIT	(0)

typedef void (*modem_ubx_script_handler)(const struct modem_ubx_script *frame);

static void modem_ubx_script_stop(struct modem_ubx *ubx, enum modem_ubx_script_result result)
{
	// if ((ubx == NULL) || (ubx->script == NULL)) {
	// 	return;
	// }

	// /* Handle result */
	// if (result == MODEM_UBX_SCRIPT_RESULT_SUCCESS) {
	// 	LOG_DBG("%s: complete", ubx->script->name);
	// } else if (result == MODEM_UBX_SCRIPT_RESULT_ABORT) {
	// 	LOG_WRN("%s: aborted", ubx->script->name);
	// } else {
	// 	LOG_WRN("%s: timed out", ubx->script->name);
	// }

	// /* Call back with result */
	// if (ubx->script->callback != NULL) {
	// 	ubx->script->callback(ubx, result, ubx->user_data);
	// }

	// /* Clear parse_match in case it is stored in the script being stopped */
	// if ((ubx->parse_match != NULL) &&
	//     ((ubx->parse_match_type == MODEM_UBX_MATCHES_INDEX_ABORT) ||
	//      (ubx->parse_match_type == MODEM_UBX_MATCHES_INDEX_RESPONSE))) {
	// 	ubx->parse_match = NULL;
	// 	ubx->parse_match_len = 0;
	// }

	// /* Clear reference to script */
	// ubx->script = NULL;

	// /* Clear response and abort commands */
	// ubx->matches[MODEM_UBX_MATCHES_INDEX_ABORT] = NULL;
	// ubx->matches_size[MODEM_UBX_MATCHES_INDEX_ABORT] = 0;
	// ubx->matches[MODEM_UBX_MATCHES_INDEX_RESPONSE] = NULL;
	// ubx->matches_size[MODEM_UBX_MATCHES_INDEX_RESPONSE] = 0;

	// /* Cancel work */
	// k_work_cancel_delayable(&ubx->script_timeout_work);
	// k_work_cancel_delayable(&ubx->script_send_work);
	// k_work_cancel_delayable(&ubx->script_send_timeout_work);

	// /* Clear script running state */
	// atomic_clear_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	// /* Store result of script for script stoppted indication */
	// ubx->script_result = result;

	// /* Indicate script stopped */
	// k_sem_give(&ubx->script_stopped_sem);
}

static void modem_ubx_script_send(struct modem_ubx *ubx)
{
	/* Initialize script send work */
	ubx->script_send_request_pos = 0;
	ubx->script_send_delimiter_pos = 0;

	/* Schedule script send work */
	k_work_schedule(&ubx->script_send_work, K_NO_WAIT);
}

static void modem_ubx_script_next(struct modem_ubx *ubx, bool initial)
{
	modem_ubx_script_send(ubx);
}

static void modem_ubx_script_start(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	/* Save script */
	ubx->script = script;

	/* Set first script command */
	modem_ubx_script_next(ubx, true);

	/* Start timeout work if script started */
	// if (ubx->script != NULL) {
	// 	k_work_schedule(&ubx->script_timeout_work, K_SECONDS(ubx->script->timeout));
	// }
}

static void modem_ubx_script_run_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, script_run_work);

	/* Start script */
	modem_ubx_script_start(ubx, ubx->pending_script);
}

static void modem_ubx_script_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_ubx *ubx = CONTAINER_OF(dwork, struct modem_ubx, script_timeout_work);

	/* Abort script */
	modem_ubx_script_stop(ubx, MODEM_UBX_SCRIPT_RESULT_TIMEOUT);
}

static void modem_ubx_script_abort_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, script_abort_work);

	/* Validate script is currently running */
	// if (ubx->script == NULL) {
	// 	return;
	// }

	/* Abort script */
	// modem_ubx_script_stop(ubx, MODEM_UBX_SCRIPT_RESULT_ABORT);
}

static bool modem_ubx_script_send_request(struct modem_ubx *ubx)
{
	// const struct modem_ubx_script_ubx *script_ubx =
	// 	&ubx->script->script_ubxs[ubx->script_ubx_it];

	// uint8_t *script_ubx_request_start;
	// uint16_t script_ubx_request_remaining;
	int ret;

	// script_ubx_request_start = (uint8_t *)&script_ubx->request[ubx->script_send_request_pos];
	// script_ubx_request_remaining = script_ubx->request_size - ubx->script_send_request_pos;

	/* Send data through pipe */
	ret = modem_pipe_transmit(ubx->pipe, ubx->pending_script->ubx_frame,
				  ubx->pending_script->ubx_frame_size);

	/* Validate transmit successful */
	if (ret < 1) {
		return false;
	} else {
		printk("modem_pipe_transmit successful.\n");
	}

	// /* Update script send position */
	// ubx->script_send_request_pos += (uint16_t)ret;

	// /* Check if data remains */
	// if (ubx->script_send_request_pos < script_ubx->request_size) {
	// 	return false;
	// }

	// return true;
}

static void modem_ubx_script_send_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_ubx *ubx = CONTAINER_OF(dwork, struct modem_ubx, script_send_timeout_work);

	/* Validate script is currently running */
	if (ubx->script == NULL) {
		return;
	}

	modem_ubx_script_next(ubx, false);
}

static void modem_ubx_script_send_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_ubx *ubx = CONTAINER_OF(dwork, struct modem_ubx, script_send_work);
	uint16_t timeout;

	/* Validate script running */
	if (ubx->script == NULL) {
		return;
	}

	/* Send request */
	if (modem_ubx_script_send_request(ubx) == false) {
		k_work_schedule(&ubx->script_send_work, ubx->process_timeout);
		return;
	}

	// /* Check if script command is no response */
	// if (modem_ubx_script_ubx_is_no_response(ubx)) {
	// 	timeout = modem_ubx_script_ubx_get_send_timeout(ubx);

	// 	if (timeout == 0) {
	// 		modem_ubx_script_next(ubx, false);
	// 	} else {
	// 		k_work_schedule(&ubx->script_send_timeout_work, K_MSEC(timeout));
	// 	}
	// } else {
	// 	modem_ubx_script_set_response_matches(ubx);
	// }
}

static void modem_ubx_process_bytes(struct modem_ubx *ubx)
{
	// for (uint16_t i = 0; i < ubx->work_buf_len; i++) {
	// 	if (modem_ubx_discard_byte(ubx, ubx->work_buf[i])) {
	// 		continue;
	// 	}

	// 	modem_ubx_process_byte(ubx, ubx->work_buf[i]);
	// }
}

static void modem_ubx_process_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_ubx *ubx = CONTAINER_OF(dwork, struct modem_ubx, process_work);
	int ret;

	/* Fill work buffer */
	ret = modem_pipe_receive(ubx->pipe, ubx->work_buf, sizeof(ubx->work_buf));
	if (ret < 1) {
		return;
	}

	/* Save received data length */
	ubx->work_buf_len = (size_t)ret;

	/* Process data */
	modem_ubx_process_bytes(ubx);
	k_work_schedule(&ubx->process_work, K_NO_WAIT);
}

static void modem_ubx_pipe_callback(struct modem_pipe *pipe, enum modem_pipe_event event,
				     void *user_data)
{
	struct modem_ubx *ubx = (struct modem_ubx *)user_data;

	if (event == MODEM_PIPE_EVENT_RECEIVE_READY) {
		k_work_schedule(&ubx->process_work, ubx->process_timeout);
	}
}

int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config)
{
	__ASSERT_NO_MSG(ubx != NULL);
	__ASSERT_NO_MSG(config != NULL);
	__ASSERT_NO_MSG(config->receive_buf != NULL);
	__ASSERT_NO_MSG(config->receive_buf_size > 0);

	memset(ubx, 0x00, sizeof(*ubx));
	ubx->pipe = NULL;
	ubx->user_data = config->user_data;
	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->process_timeout = config->process_timeout;
	atomic_set(&ubx->script_state, 0);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);
	k_work_init_delayable(&ubx->process_work, modem_ubx_process_handler);
	k_work_init(&ubx->script_run_work, modem_ubx_script_run_handler);
	k_work_init_delayable(&ubx->script_timeout_work, modem_ubx_script_timeout_handler);
	k_work_init(&ubx->script_abort_work, modem_ubx_script_abort_handler);
	k_work_init_delayable(&ubx->script_send_work, modem_ubx_script_send_handler);
	k_work_init_delayable(&ubx->script_send_timeout_work,
			      modem_ubx_script_send_timeout_handler);

	return 0;
}

int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe)
{
	ubx->pipe = pipe;
	modem_pipe_attach(ubx->pipe, modem_ubx_pipe_callback, ubx);
	return 0;
}

int modem_ubx_run_script_async(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	bool script_is_running;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	script_is_running =
		atomic_test_and_set_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	if (script_is_running == true) {
		return -EBUSY;
	}

	ubx->pending_script = script;
	printk("modem_ubx_run_script_async: k_work_submit before.\n");
	k_work_submit(&ubx->script_run_work);
	printk("modem_ubx_run_script_async: k_work_submit after.\n");
	return 0;
}

int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

	printk("modem_ubx_run_script_async: script_stopped_sem.\n");
	k_sleep(K_MSEC(1000)); // temp (mayank).
	k_sem_reset(&ubx->script_stopped_sem);

	ret = modem_ubx_run_script_async(ubx, script);
	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&ubx->script_stopped_sem, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	return ubx->script_result == MODEM_UBX_SCRIPT_RESULT_SUCCESS ? 0 : -EAGAIN;
}

void modem_ubx_script_abort(struct modem_ubx *ubx)
{
	k_work_submit(&ubx->script_abort_work);
}

void modem_ubx_release(struct modem_ubx *ubx)
{
	struct k_work_sync sync;

	if (ubx->pipe) {
		modem_pipe_release(ubx->pipe);
	}

	k_work_cancel_sync(&ubx->script_run_work, &sync);
	k_work_cancel_sync(&ubx->script_abort_work, &sync);
	k_work_cancel_delayable_sync(&ubx->process_work, &sync);
	k_work_cancel_delayable_sync(&ubx->script_send_work, &sync);

	ubx->pipe = NULL;
	ubx->receive_buf_len = 0;
	ubx->work_buf_len = 0;
	ubx->script = NULL;
	atomic_set(&ubx->script_state, 0);
	ubx->script_result = MODEM_UBX_SCRIPT_RESULT_ABORT;
	k_sem_reset(&ubx->script_stopped_sem);
	ubx->script_send_request_pos = 0;
	ubx->script_send_delimiter_pos = 0;
}
