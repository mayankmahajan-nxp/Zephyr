/*
 * Copyright (c) 2023 Trackunit Corporation
 * Copyright (c) 2023 Bjarki Arge Andreasen
 * Copyright 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/ubx.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include "gnss_nmea0183.h"
#include "gnss_nmea0183_match.h"
#include "gnss_parse.h"

#include "u_blox_protocol.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define DT_DRV_COMPAT u_blox_m10

#define UART_RECV_BUF_SZ 128
#define UART_TRNF_BUF_SZ 128

#define CHAT_RECV_BUF_SZ 256
#define CHAT_ARGV_SZ 32

#define UBX_RECV_BUF_SZ 256
#define UBX_ARGV_SZ 32

struct u_blox_m10_config {
	const struct device *uart;
};

struct u_blox_m10_data {
	struct gnss_nmea0183_match_data match_data;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_U_BLOX_M10_SATELLITES_COUNT];
#endif

	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[UART_RECV_BUF_SZ];
	uint8_t uart_backend_transmit_buf[UART_TRNF_BUF_SZ];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CHAT_RECV_BUF_SZ];
	uint8_t *chat_argv[CHAT_ARGV_SZ];

	/* Modem ubx */
	struct modem_ubx ubx;
	uint8_t ubx_receive_buf[UBX_RECV_BUF_SZ];
	uint8_t *ubx_argv[UBX_ARGV_SZ];

	struct k_spinlock lock;
};

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

// static int u_blox_m10_send_ublox_m8_message(const struct device *dev, uint8_t *ubx_frame,
// 					      uint8_t ubx_frame_len)
// {
// 	struct u_blox_m10_data *data = dev->data;

// 	int ret;

// 	u_blox_m10_chat_parse_reset(&data->chat_ubx);

// 	// ret = modem_pipe_transmit(data->chat.pipe, ubx_frame, ubx_frame_len);

// 	// MODEM_CHAT_SCRIPT_CMDS_DEFINE(
// 	// 	ublox_m8_script_chat,
// 	// 	MODEM_CHAT_SCRIPT_CMD_RESP(ubx_frame, ublox_m8_ack),
// 	// ); // temp (mayank): could use this, but we will have to change size of ubx_frame to 28.
// 	const struct modem_chat_script_chat ublox_m8_script_chat = {
// 		.request = ubx_frame,
// 		.request_size = ubx_frame_len,
// 		.response_matches = &ublox_m8_ack,
// 		.response_matches_size = 1,
// 		.timeout = 0,
// 	};
// 	const struct modem_chat_script ublox_m8_script = {
// 		.name = "ublox_m8_script",
// 		.script_chats = &ublox_m8_script_chat,
// 		.script_chats_size = 1,
// 		.abort_matches = NULL,
// 		.abort_matches_size = 0,
// 		.callback = NULL,
// 		.timeout = 1,
// 	};

//   u_blox_m10_uart_flush(dev);
// 	ret = modem_chat_run_script(&data->chat_ubx, &ublox_m8_script);
// 	modem_chat_release(&data->chat_ubx);
//   u_blox_m10_turn_off(dev);
//   u_blox_m10_uart_read_reg(dev);
// 	k_sleep(K_MSEC(100)); // temp (mayank): check if this is necessary.
// 	printk("ret for modem_chat_run_script = %d.\n", ret);

// 	return ret;
// }

// static int u_blox_m10_get_uart_baudrate(const struct device *dev) {
// 	const struct u_blox_m10_config *cfg = dev->config;

// 	const struct uart_driver_api *uart_api = cfg->uart->api;
// 	struct uart_config uart_config;

// 	uart_api->config_get(cfg->uart, &uart_config);
// 	uint32_t baudrate = uart_config.baudrate;

// 	return baudrate;
// }

// static int u_blox_m10_set_uart_baudrate(const struct device *dev, uint32_t baudrate) {
// 	const struct u_blox_m10_config *cfg = dev->config;

// 	const struct uart_driver_api *uart_api = cfg->uart->api;
// 	struct uart_config uart_config;

// 	// temp (mayank): check which things need to be re-initialized here.
// 	u_blox_m10_turn_off(dev);

// 	uart_api->config_get(cfg->uart, &uart_config);
// 	uart_config.baudrate = baudrate;

// 	// temp (mayank): check which of the following are necessary here.
// 	int ret = uart_api->configure(cfg->uart, &uart_config);
// 	uart_configure(cfg->uart, &uart_config);

// 	// u_blox_m10_init_nmea0183_match(dev);
// 	u_blox_m10_init_pipe(dev);
// 	// u_blox_m10_init_chat(dev);
// 	u_blox_m10_resume(dev);
// 	printk("u_blox_m10_set_uart_baudrate completed %d.\n\n", baudrate);

// 	return ret;
// }

// static int u_blox_m10_set_device_baudrate(const struct device *dev, uint32_t baudrate) {
// 	struct u_blox_m10_data *data = dev->data;
// 	k_spinlock_key_t key;

// 	int ret;

// 	key = k_spin_lock(&data->lock);

// 	u_blox_m10_turn_off(dev);
// 	u_blox_m10_resume_ubx(dev);

// 	uint8_t ubx_frame[256], ubx_frame_len;

// 	ublox_m8_get_cfg_rst_msg(dev, ubx_frame, &ubx_frame_len, 0x08);
// 	ret = u_blox_m10_send_ublox_m8_message(dev, ubx_frame, ubx_frame_len);

// 	ublox_m8_get_cfg_prt_msg(dev, ubx_frame, &ubx_frame_len, 0x01, baudrate);
// 	int success_count = 0, total_count = 3;
// 	for (int i = 0; i < total_count; ++i) {
// 		ret = u_blox_m10_send_ublox_m8_message(dev, ubx_frame, ubx_frame_len);
// 		if (ret == 0)
// 			++success_count;
// 	}
// 	printk("success_count = %d.\n", success_count);
// 		// note: our delimeter is not the end of the message. so following bytes roll over.
// 			// if and only if we are not using the filter array to filter other stuff.
// 		// that's why the following call returns -11. solution: clear the receive buffer.

// 	ublox_m8_get_cfg_rst_msg(dev, ubx_frame, &ubx_frame_len, 0x09);
// 	ret = u_blox_m10_send_ublox_m8_message(dev, ubx_frame, ubx_frame_len);

// 	if (ret < 0) {
// 		goto unlock_return;
// 	}

// unlock_return:
// 	k_spin_unlock(&data->lock, key);

// 	u_blox_m10_turn_off(dev);
// 	u_blox_m10_resume(dev);

// 	return ret;
// }

// static int u_blox_m10_configure_baudrate(const struct device *dev) {
// 	uint32_t target_baudrate = u_blox_m10_get_uart_baudrate(dev);

// 	int ret;

// 	/* Try to comply the baudrate of UART with the preset baudrate of dev,
// 		by writing to dev with UART baudrate as all possible baudrates. */
// 	for (int i = 0; i < U_BLOX_M10_BAUDRATES_COUNT; ++i) {
// 		LOG_ERR("%d %d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", i, u_blox_m10_baudrates[i]);
// 		ret = u_blox_m10_set_uart_baudrate(dev, u_blox_m10_baudrates[i]);
// 		if (ret < 0) {
// 			return ret;
// 		}

// 		// k_sleep(K_MSEC(100));
// 		for (int j = 0; j < 1; ++j)
// 			ret = u_blox_m10_set_device_baudrate(dev, target_baudrate);
// 		// if (ret == 0) { // temp (mayank): commenting, as right now we can't detect ack.
// 		// 	break;
// 		// }
// 	}

// 	ret = u_blox_m10_set_uart_baudrate(dev, target_baudrate);
// 	if (ret < 0) {
// 		return ret;
// 	}

// 	return ret;
// }

static int u_blox_m10_resume(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;

	ret = modem_pipe_open(data->uart_pipe);
	if (ret < 0) {
		return ret;
	}

	ret = modem_chat_attach(&data->chat, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
		return ret;
	}

	return ret;
}

static int u_blox_m10_turn_off(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;

	return modem_pipe_close(data->uart_pipe);
}

static struct gnss_driver_api gnss_api = {
};

static int u_blox_m10_init_nmea0183_match(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;

	const struct gnss_nmea0183_match_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &match_config);
}

static void u_blox_m10_init_pipe(const struct device *dev)
{
	const struct u_blox_m10_config *cfg = dev->config;
	struct u_blox_m10_data *data = dev->data;

	const struct modem_backend_uart_config uart_backend_config = {
		.uart = cfg->uart,
		.receive_buf = data->uart_backend_receive_buf,
		.receive_buf_size = sizeof(data->uart_backend_receive_buf),
		.transmit_buf = data->uart_backend_transmit_buf, // temp: check whether this is needed.
		.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf), // temp: check whether this is needed.
	};

	data->uart_pipe = modem_backend_uart_init(&data->uart_backend, &uart_backend_config);
}

static uint8_t u_blox_m10_char_delimiter[] = {'\r', '\n'};

static int u_blox_m10_init_chat(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = sizeof(data->chat_receive_buf),
		.delimiter = u_blox_m10_char_delimiter,
		.delimiter_size = ARRAY_SIZE(u_blox_m10_char_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.unsol_matches = unsol_matches,
		.unsol_matches_size = ARRAY_SIZE(unsol_matches),
		.process_timeout = K_MSEC(2),
	};

	return modem_chat_init(&data->chat, &chat_config);
}

static int u_blox_m10_init_ubx(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;

	const struct modem_ubx_config ubx_config = {
		.user_data = data,
		.receive_buf = data->ubx_receive_buf,
		.receive_buf_size = sizeof(data->ubx_receive_buf),
		.delimiter = u_blox_m10_char_delimiter,
		.delimiter_size = ARRAY_SIZE(u_blox_m10_char_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->ubx_argv,
		.argv_size = ARRAY_SIZE(data->ubx_argv),
		// .unsol_matches = unsol_matches,
		// .unsol_matches_size = ARRAY_SIZE(unsol_matches),
		.process_timeout = K_MSEC(2000),
	};

	return modem_ubx_init(&data->ubx, &ubx_config);
}

void u_blox_m10_ubx_callback(struct modem_ubx *ubx, char **argv, uint16_t argc, void *user_data)
{
	LOG_ERR("u_blox_m10_ubx_callback: beginning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

const static struct modem_ubx_match u_blox_m10_ack = MODEM_UBX_MATCH_WILDCARD (
	"\xb5\x62\x05\x01",
	"",
	u_blox_m10_ubx_callback
);

static int u_blox_m10_configure(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;

	// Release chat, attach ubx.
	modem_chat_release(&data->chat);
	ret = modem_ubx_attach(&data->ubx, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
		return ret;
	}

	// Send UBX_CFG_PRT to change device baudrate.
	uint8_t ubx_frame[128];
	uint8_t ubx_frame_size;
	u_blox_get_cfg_prt(ubx_frame, &ubx_frame_size, 0x01, 9600);
	const struct modem_ubx_script_ubx u_blox_m10_script = {
		.request = ubx_frame,
		.request_size = ubx_frame_size,
		.response_matches = &u_blox_m10_ack,
		.response_matches_size = 1,
		.timeout = 0,
	};
	const struct modem_ubx_script u_blox_m10_script_hold = {
		.name = "u_blox_m10_script_hold",
		.script_ubxs = &u_blox_m10_script,
		.script_ubxs_size = 1,
		.abort_matches = NULL,
		.abort_matches_size = 0,
		.callback = NULL,
		.timeout = 1,
	};

	ret = modem_ubx_run_script(&data->ubx, &u_blox_m10_script_hold);
	printk("modem_ubx_run_script: ret = %d.\n", ret);
	k_sleep(K_MSEC(2000));

	// Release ubx, attach chat.
	modem_ubx_release(&data->ubx);
	ret = modem_chat_attach(&data->chat, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
		return ret;
	}

	printk("u_blox_m10_configure: exited cleanly.\n");
	return 0;
}

static int u_blox_m10_init(const struct device *dev)
{
	int ret;
	printk("u_blox_m10_init beginning (temp).\n");

	ret = u_blox_m10_init_nmea0183_match(dev);
	if (ret < 0) {
		return ret;
	}

	u_blox_m10_init_pipe(dev);

	ret = u_blox_m10_init_chat(dev);
	if (ret < 0) {
		return ret;
	}

	ret = u_blox_m10_init_ubx(dev);
	if (ret < 0) {
		return ret;
	}

	ret = u_blox_m10_resume(dev);
	if (ret < 0) {
		return ret;
	}

	ret = u_blox_m10_configure(dev);
	if (ret < 0) {
		return ret;
	}

	printk("u_blox_m10_init completed (temp).\n");

	return 0;
}

#define U_BLOX_M10(inst)								\
	static struct u_blox_m10_config u_blox_m10_cfg_##inst = {			\
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),				\
	};										\
											\
	static struct u_blox_m10_data u_blox_m10_data_##inst;				\
											\
	DEVICE_DT_INST_DEFINE(inst, u_blox_m10_init, NULL,				\
			      &u_blox_m10_data_##inst,					\
			      &u_blox_m10_cfg_##inst,					\
			      POST_KERNEL, CONFIG_GNSS_INIT_PRIORITY, &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(U_BLOX_M10)
