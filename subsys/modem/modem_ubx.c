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

#define MODEM_UBX_MATCHES_INDEX_RESPONSE (0)
#define MODEM_UBX_MATCHES_INDEX_ABORT	  (1)
#define MODEM_UBX_MATCHES_INDEX_UNSOL	  (2)

#define MODEM_UBX_SCRIPT_STATE_RUNNING_BIT (0)

#if defined(CONFIG_LOG) && (CONFIG_MODEM_MODULES_LOG_LEVEL == LOG_LEVEL_DBG)

static char log_buffer[CONFIG_MODEM_UBX_LOG_BUFFER];

static void modem_ubx_log_received_command(struct modem_ubx *ubx)
{
	uint16_t log_buffer_pos = 0;
	uint16_t argv_len;

	for (uint16_t i = 0; i < ubx->argc; i++) {
		argv_len = (uint16_t)strlen(ubx->argv[i]);

		/* Validate argument fits in log buffer including termination */
		if (sizeof(log_buffer) < (log_buffer_pos + argv_len + 1)) {
			LOG_WRN("log buffer overrun");
			break;
		}

		/* Copy argument and append space */
		memcpy(&log_buffer[log_buffer_pos], ubx->argv[i], argv_len);
		log_buffer_pos += argv_len;
		log_buffer[log_buffer_pos] = ' ';
		log_buffer_pos++;
	}

	/* Terminate line after last argument, overwriting trailing space */
	log_buffer_pos = log_buffer_pos == 0 ? log_buffer_pos : log_buffer_pos - 1;
	log_buffer[log_buffer_pos] = '\0';

	LOG_DBG("%s", log_buffer);
}

#else

static void modem_ubx_log_received_command(struct modem_ubx *ubx)
{
}

#endif

static void modem_ubx_script_stop(struct modem_ubx *ubx, enum modem_ubx_script_result result)
{
	if ((ubx == NULL) || (ubx->script == NULL)) {
		return;
	}

	/* Handle result */
	if (result == MODEM_UBX_SCRIPT_RESULT_SUCCESS) {
		LOG_DBG("%s: complete", ubx->script->name);
	} else if (result == MODEM_UBX_SCRIPT_RESULT_ABORT) {
		LOG_WRN("%s: aborted", ubx->script->name);
	} else {
		LOG_WRN("%s: timed out", ubx->script->name);
	}

	/* Call back with result */
	if (ubx->script->callback != NULL) {
		ubx->script->callback(ubx, result, ubx->user_data);
	}

	/* Clear parse_match in case it is stored in the script being stopped */
	if ((ubx->parse_match != NULL) &&
	    ((ubx->parse_match_type == MODEM_UBX_MATCHES_INDEX_ABORT) ||
	     (ubx->parse_match_type == MODEM_UBX_MATCHES_INDEX_RESPONSE))) {
		ubx->parse_match = NULL;
		ubx->parse_match_len = 0;
	}

	/* Clear reference to script */
	ubx->script = NULL;

	/* Clear response and abort commands */
	ubx->matches[MODEM_UBX_MATCHES_INDEX_ABORT] = NULL;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_ABORT] = 0;
	ubx->matches[MODEM_UBX_MATCHES_INDEX_RESPONSE] = NULL;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_RESPONSE] = 0;

	/* Cancel work */
	k_work_cancel_delayable(&ubx->script_timeout_work);
	k_work_cancel_delayable(&ubx->script_send_work);
	k_work_cancel_delayable(&ubx->script_send_timeout_work);

	/* Clear script running state */
	atomic_clear_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	/* Store result of script for script stoppted indication */
	ubx->script_result = result;

	/* Indicate script stopped */
	k_sem_give(&ubx->script_stopped_sem);
}

static void modem_ubx_script_send(struct modem_ubx *ubx)
{
	/* Initialize script send work */
	ubx->script_send_request_pos = 0;
	ubx->script_send_delimiter_pos = 0;

	/* Schedule script send work */
	k_work_schedule(&ubx->script_send_work, K_NO_WAIT);
}

static void modem_ubx_script_set_response_matches(struct modem_ubx *ubx)
{
	const struct modem_ubx_script_chat *script_chat =
		&ubx->script->script_chats[ubx->script_chat_it];

	ubx->matches[MODEM_UBX_MATCHES_INDEX_RESPONSE] = script_chat->response_matches;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_RESPONSE] = script_chat->response_matches_size;
}

static void modem_ubx_script_clear_response_matches(struct modem_ubx *ubx)
{
	ubx->matches[MODEM_UBX_MATCHES_INDEX_RESPONSE] = NULL;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_RESPONSE] = 0;
}

static void modem_ubx_script_next(struct modem_ubx *ubx, bool initial)
{
	const struct modem_ubx_script_chat *script_chat;

	/* Advance iterator if not initial */
	if (initial == true) {
		/* Reset iterator */
		ubx->script_chat_it = 0;
	} else {
		/* Advance iterator */
		ubx->script_chat_it++;
	}

	/* Check if end of script reached */
	if (ubx->script_chat_it == ubx->script->script_chats_size) {
		modem_ubx_script_stop(ubx, MODEM_UBX_SCRIPT_RESULT_SUCCESS);

		return;
	}

	LOG_DBG("%s: step: %u", ubx->script->name, ubx->script_chat_it);

	script_chat = &ubx->script->script_chats[ubx->script_chat_it];

	/* Check if request must be sent */
	if (script_chat->request_size > 0) {
		LOG_DBG("sending: %.*s", script_chat->request_size, script_chat->request);
		modem_ubx_script_clear_response_matches(ubx);
		modem_ubx_script_send(ubx);
	} else {
		modem_ubx_script_set_response_matches(ubx);
	}
}

static void modem_ubx_script_start(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	/* Save script */
	ubx->script = script;

	/* Set abort matches */
	ubx->matches[MODEM_UBX_MATCHES_INDEX_ABORT] = script->abort_matches;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_ABORT] = script->abort_matches_size;

	LOG_DBG("running script: %s", ubx->script->name);

	/* Set first script command */
	modem_ubx_script_next(ubx, true);

	/* Start timeout work if script started */
	if (ubx->script != NULL) {
		k_work_schedule(&ubx->script_timeout_work, K_SECONDS(ubx->script->timeout));
	}
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
	if (ubx->script == NULL) {
		return;
	}

	/* Abort script */
	modem_ubx_script_stop(ubx, MODEM_UBX_SCRIPT_RESULT_ABORT);
}

static bool modem_ubx_script_send_request(struct modem_ubx *ubx)
{
	const struct modem_ubx_script_chat *script_chat =
		&ubx->script->script_chats[ubx->script_chat_it];

	uint8_t *script_chat_request_start;
	uint16_t script_chat_request_remaining;
	int ret;

	/* Validate data to send */
	if (script_chat->request_size == ubx->script_send_request_pos) {
		return true;
	}

	script_chat_request_start = (uint8_t *)&script_chat->request[ubx->script_send_request_pos];
	script_chat_request_remaining = script_chat->request_size - ubx->script_send_request_pos;

	/* Send data through pipe */
	ret = modem_pipe_transmit(ubx->pipe, script_chat_request_start,
				  script_chat_request_remaining);

	/* Validate transmit successful */
	if (ret < 1) {
		return false;
	}

	/* Update script send position */
	ubx->script_send_request_pos += (uint16_t)ret;

	/* Check if data remains */
	if (ubx->script_send_request_pos < script_chat->request_size) {
		return false;
	}

	return true;
}

static bool modem_ubx_script_send_delimiter(struct modem_ubx *ubx)
{
	uint8_t *script_chat_delimiter_start;
	uint8_t script_chat_delimiter_remaining;
	int ret;

	/* Validate data to send */
	if (ubx->delimiter_size == ubx->script_send_delimiter_pos) {
		return true;
	}

	script_chat_delimiter_start = (uint8_t *)&ubx->delimiter[ubx->script_send_delimiter_pos];
	script_chat_delimiter_remaining = ubx->delimiter_size - ubx->script_send_delimiter_pos;

	/* Send data through pipe */
	ret = modem_pipe_transmit(ubx->pipe, script_chat_delimiter_start,
				  script_chat_delimiter_remaining);

	/* Validate transmit successful */
	if (ret < 1) {
		return false;
	}

	/* Update script send position */
	ubx->script_send_delimiter_pos += (uint8_t)ret;

	/* Check if data remains */
	if (ubx->script_send_delimiter_pos < ubx->delimiter_size) {
		return false;
	}

	return true;
}

static bool modem_ubx_script_chat_is_no_response(struct modem_ubx *ubx)
{
	const struct modem_ubx_script_chat *script_chat =
		&ubx->script->script_chats[ubx->script_chat_it];

	return (script_chat->response_matches_size == 0) ? true : false;
}

static uint16_t modem_ubx_script_chat_get_send_timeout(struct modem_ubx *ubx)
{
	const struct modem_ubx_script_chat *script_chat =
		&ubx->script->script_chats[ubx->script_chat_it];

	return script_chat->timeout;
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

	/* Send delimiter */
	if (modem_ubx_script_send_delimiter(ubx) == false) {
		k_work_schedule(&ubx->script_send_work, ubx->process_timeout);
		return;
	}

	/* Check if script command is no response */
	if (modem_ubx_script_chat_is_no_response(ubx)) {
		timeout = modem_ubx_script_chat_get_send_timeout(ubx);

		if (timeout == 0) {
			modem_ubx_script_next(ubx, false);
		} else {
			k_work_schedule(&ubx->script_send_timeout_work, K_MSEC(timeout));
		}
	} else {
		modem_ubx_script_set_response_matches(ubx);
	}
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

static void modem_ubx_parse_reset(struct modem_ubx *ubx)
{
	/* Reset parameters used for parsing */
	ubx->receive_buf_len = 0;
	ubx->delimiter_match_len = 0;
	ubx->argc = 0;
	ubx->parse_match = NULL;
}

/* Exact match is stored at end of receive buffer */
static void modem_ubx_parse_save_match(struct modem_ubx *ubx)
{
	uint8_t *argv;

	/* Store length of match including NULL to avoid overwriting it if buffer overruns */
	ubx->parse_match_len = ubx->receive_buf_len + 1;

	/* Copy match to end of receive buffer */
	argv = &ubx->receive_buf[ubx->receive_buf_size - ubx->parse_match_len];

	/* Copy match to end of receive buffer (excluding NULL) */
	memcpy(argv, &ubx->receive_buf[0], ubx->parse_match_len - 1);

	/* Save match */
	ubx->argv[ubx->argc] = argv;

	/* Terminate match */
	ubx->receive_buf[ubx->receive_buf_size - 1] = '\0';

	/* Increment argument count */
	ubx->argc++;
}

static bool modem_ubx_match_matches_received(struct modem_ubx *ubx,
					      const struct modem_ubx_match *match)
{
	for (uint16_t i = 0; i < match->match_size; i++) {
		if ((match->match[i] == ubx->receive_buf[i]) ||
		    (match->wildcards == true && match->match[i] == '?')) {
			continue;
		}

		return false;
	}

	return true;
}

static bool modem_ubx_parse_find_match(struct modem_ubx *ubx)
{
	/* Find in all matches types */
	for (uint16_t i = 0; i < ARRAY_SIZE(ubx->matches); i++) {
		/* Find in all matches of matches type */
		for (uint16_t u = 0; u < ubx->matches_size[i]; u++) {
			/* Validate match size matches received data length */
			if (ubx->matches[i][u].match_size != ubx->receive_buf_len) {
				continue;
			}

			/* Validate match */
			if (modem_ubx_match_matches_received(ubx, &ubx->matches[i][u]) ==
			    false) {
				continue;
			}

			/* Complete match found */
			ubx->parse_match = &ubx->matches[i][u];
			ubx->parse_match_type = i;
			return true;
		}
	}

	return false;
}

static bool modem_ubx_parse_is_separator(struct modem_ubx *ubx)
{
	for (uint16_t i = 0; i < ubx->parse_match->separators_size; i++) {
		if ((ubx->parse_match->separators[i]) ==
		    (ubx->receive_buf[ubx->receive_buf_len - 1])) {
			return true;
		}
	}

	return false;
}

static bool modem_ubx_parse_end_del_start(struct modem_ubx *ubx)
{
	for (uint8_t i = 0; i < ubx->delimiter_size; i++) {
		if (ubx->receive_buf[ubx->receive_buf_len - 1] == ubx->delimiter[i]) {
			return true;
		}
	}

	return false;
}

static bool modem_ubx_parse_end_del_complete(struct modem_ubx *ubx)
{
	/* Validate length of end delimiter */
	if (ubx->receive_buf_len < ubx->delimiter_size) {
		return false;
	}

	/* Compare end delimiter with receive buffer content */
	return (memcmp(&ubx->receive_buf[ubx->receive_buf_len - ubx->delimiter_size],
		       ubx->delimiter, ubx->delimiter_size) == 0)
		       ? true
		       : false;
}

static void modem_ubx_on_command_received_unsol(struct modem_ubx *ubx)
{
	/* Callback */
	if (ubx->parse_match->callback != NULL) {
		ubx->parse_match->callback(ubx, (char **)ubx->argv, ubx->argc, ubx->user_data);
	}
}

static void modem_ubx_on_command_received_abort(struct modem_ubx *ubx)
{
	/* Callback */
	if (ubx->parse_match->callback != NULL) {
		ubx->parse_match->callback(ubx, (char **)ubx->argv, ubx->argc, ubx->user_data);
	}

	/* Abort script */
	modem_ubx_script_stop(ubx, MODEM_UBX_SCRIPT_RESULT_ABORT);
}

static void modem_ubx_on_command_received_resp(struct modem_ubx *ubx)
{
	/* Callback */
	if (ubx->parse_match->callback != NULL) {
		ubx->parse_match->callback(ubx, (char **)ubx->argv, ubx->argc, ubx->user_data);
	}

	/* Validate response command is not partial */
	if (ubx->parse_match->partial) {
		return;
	}

	/* Advance script */
	modem_ubx_script_next(ubx, false);
}

static bool modem_ubx_parse_find_catch_all_match(struct modem_ubx *ubx)
{
	/* Find in all matches types */
	for (uint16_t i = 0; i < ARRAY_SIZE(ubx->matches); i++) {
		/* Find in all matches of matches type */
		for (uint16_t u = 0; u < ubx->matches_size[i]; u++) {
			/* Validate match config is matching previous bytes */
			if (ubx->matches[i][u].match_size == 0) {
				ubx->parse_match = &ubx->matches[i][u];
				ubx->parse_match_type = i;
				return true;
			}
		}
	}

	return false;
}

static void modem_ubx_on_command_received(struct modem_ubx *ubx)
{
	modem_ubx_log_received_command(ubx);

	switch (ubx->parse_match_type) {
	case MODEM_UBX_MATCHES_INDEX_UNSOL:
		modem_ubx_on_command_received_unsol(ubx);
		break;

	case MODEM_UBX_MATCHES_INDEX_ABORT:
		modem_ubx_on_command_received_abort(ubx);
		break;

	case MODEM_UBX_MATCHES_INDEX_RESPONSE:
		modem_ubx_on_command_received_resp(ubx);
		break;
	}
}

static void modem_ubx_on_unknown_command_received(struct modem_ubx *ubx)
{
	/* Terminate received command */
	ubx->receive_buf[ubx->receive_buf_len - ubx->delimiter_size] = '\0';

	/* Try to find catch all match */
	if (modem_ubx_parse_find_catch_all_match(ubx) == false) {
		LOG_DBG("%s", ubx->receive_buf);
		return;
	}

	/* Parse command */
	ubx->argv[0] = "";
	ubx->argv[1] = ubx->receive_buf;
	ubx->argc = 2;

	modem_ubx_on_command_received(ubx);
}

static void modem_ubx_process_byte(struct modem_ubx *ubx, uint8_t byte)
{
	/* Validate receive buffer not overrun */
	if (ubx->receive_buf_size == ubx->receive_buf_len) {
		LOG_WRN("receive buffer overrun");
		modem_ubx_parse_reset(ubx);
		return;
	}

	/* Validate argv buffer not overrun */
	if (ubx->argc == ubx->argv_size) {
		LOG_WRN("argv buffer overrun");
		modem_ubx_parse_reset(ubx);
		return;
	}

	/* Copy byte to receive buffer */
	ubx->receive_buf[ubx->receive_buf_len] = byte;
	ubx->receive_buf_len++;

	/* Validate end delimiter not complete */
	if (modem_ubx_parse_end_del_complete(ubx) == true) {
		/* Filter out empty lines */
		if (ubx->receive_buf_len == ubx->delimiter_size) {
			/* Reset parser */
			modem_ubx_parse_reset(ubx);
			return;
		}

		/* Check if match exists */
		if (ubx->parse_match == NULL) {
			/* Handle unknown command */
			modem_ubx_on_unknown_command_received(ubx);

			/* Reset parser */
			modem_ubx_parse_reset(ubx);
			return;
		}

		/* Check if trailing argument exists */
		if (ubx->parse_arg_len > 0) {
			ubx->argv[ubx->argc] =
				&ubx->receive_buf[ubx->receive_buf_len - ubx->delimiter_size -
						   ubx->parse_arg_len];
			ubx->receive_buf[ubx->receive_buf_len - ubx->delimiter_size] = '\0';
			ubx->argc++;
		}

		/* Handle received command */
		modem_ubx_on_command_received(ubx);

		/* Reset parser */
		modem_ubx_parse_reset(ubx);
		return;
	}

	/* Validate end delimiter not started */
	if (modem_ubx_parse_end_del_start(ubx) == true) {
		return;
	}

	/* Find matching command if missing */
	if (ubx->parse_match == NULL) {
		/* Find matching command */
		if (modem_ubx_parse_find_match(ubx) == false) {
			return;
		}

		/* Save match */
		modem_ubx_parse_save_match(ubx);

		/* Prepare argument parser */
		ubx->parse_arg_len = 0;
		return;
	}

	/* Check if separator reached */
	if (modem_ubx_parse_is_separator(ubx) == true) {
		/* Check if argument is empty */
		if (ubx->parse_arg_len == 0) {
			/* Save empty argument */
			ubx->argv[ubx->argc] = "";
		} else {
			/* Save pointer to start of argument */
			ubx->argv[ubx->argc] =
				&ubx->receive_buf[ubx->receive_buf_len - ubx->parse_arg_len - 1];

			/* Replace separator with string terminator */
			ubx->receive_buf[ubx->receive_buf_len - 1] = '\0';
		}

		/* Increment argument count */
		ubx->argc++;

		/* Reset parse argument length */
		ubx->parse_arg_len = 0;
		return;
	}

	/* Increment argument length */
	ubx->parse_arg_len++;
}

static bool modem_ubx_discard_byte(struct modem_ubx *ubx, uint8_t byte)
{
	for (uint8_t i = 0; i < ubx->filter_size; i++) {
		if (byte == ubx->filter[i]) {
			return true;
		}
	}

	return false;
}

/* Process chunk of received bytes */
static void modem_ubx_process_bytes(struct modem_ubx *ubx)
{
	for (uint16_t i = 0; i < ubx->work_buf_len; i++) {
		if (modem_ubx_discard_byte(ubx, ubx->work_buf[i])) {
			continue;
		}

		modem_ubx_process_byte(ubx, ubx->work_buf[i]);
	}
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
	__ASSERT_NO_MSG(config->argv != NULL);
	__ASSERT_NO_MSG(config->argv_size > 0);
	__ASSERT_NO_MSG(config->delimiter != NULL);
	__ASSERT_NO_MSG(config->delimiter_size > 0);
	__ASSERT_NO_MSG(!((config->filter == NULL) && (config->filter > 0)));
	__ASSERT_NO_MSG(!((config->unsol_matches == NULL) && (config->unsol_matches_size > 0)));

	memset(ubx, 0x00, sizeof(*ubx));
	ubx->pipe = NULL;
	ubx->user_data = config->user_data;
	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;
	ubx->argv = config->argv;
	ubx->argv_size = config->argv_size;
	ubx->delimiter = config->delimiter;
	ubx->delimiter_size = config->delimiter_size;
	ubx->filter = config->filter;
	ubx->filter_size = config->filter_size;
	ubx->matches[MODEM_UBX_MATCHES_INDEX_UNSOL] = config->unsol_matches;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_UNSOL] = config->unsol_matches_size;
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
	modem_ubx_parse_reset(ubx);
	modem_pipe_attach(ubx->pipe, modem_ubx_pipe_callback, ubx);
	return 0;
}

int modem_ubx_run_script_async(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	bool script_is_running;

	if (ubx->pipe == NULL) {
		return -EPERM;
	}

	/* Validate script */
	if ((script->script_chats == NULL) || (script->script_chats_size == 0) ||
	    ((script->abort_matches != NULL) && (script->abort_matches_size == 0))) {
		return -EINVAL;
	}

	/* Validate script commands */
	for (uint16_t i = 0; i < script->script_chats_size; i++) {
		if ((script->script_chats[i].request_size == 0) &&
		    (script->script_chats[i].response_matches_size == 0)) {
			return -EINVAL;
		}
	}

	script_is_running =
		atomic_test_and_set_bit(&ubx->script_state, MODEM_UBX_SCRIPT_STATE_RUNNING_BIT);

	if (script_is_running == true) {
		return -EBUSY;
	}

	ubx->pending_script = script;
	k_work_submit(&ubx->script_run_work);
	return 0;
}

int modem_ubx_run_script(struct modem_ubx *ubx, const struct modem_ubx_script *script)
{
	int ret;

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
	ubx->argc = 0;
	ubx->script = NULL;
	ubx->script_chat_it = 0;
	atomic_set(&ubx->script_state, 0);
	ubx->script_result = MODEM_UBX_SCRIPT_RESULT_ABORT;
	k_sem_reset(&ubx->script_stopped_sem);
	ubx->script_send_request_pos = 0;
	ubx->script_send_delimiter_pos = 0;
	ubx->parse_match = NULL;
	ubx->parse_match_len = 0;
	ubx->parse_arg_len = 0;
	ubx->matches[MODEM_UBX_MATCHES_INDEX_ABORT] = NULL;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_ABORT] = 0;
	ubx->matches[MODEM_UBX_MATCHES_INDEX_RESPONSE] = NULL;
	ubx->matches_size[MODEM_UBX_MATCHES_INDEX_RESPONSE] = 0;
}
