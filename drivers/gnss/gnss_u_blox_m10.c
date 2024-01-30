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

#define UART_RECV_BUF_SZ	128
#define UART_TRNF_BUF_SZ	128

#define CHAT_RECV_BUF_SZ	256
#define CHAT_ARGV_SZ		32

#define UBX_RECV_BUF_SZ		128
#define UBX_WORK_BUF_SZ		128

#define UBX_MESSAGE_TIMEOUT_MS	500

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
	uint8_t ubx_work_buf[UBX_WORK_BUF_SZ];

	struct k_spinlock lock;
};

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

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
		.transmit_buf = data->uart_backend_transmit_buf,
		.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
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
		.work_buf = data->ubx_work_buf,
		.work_buf_size = sizeof(data->ubx_work_buf),
		.process_timeout = K_MSEC(UBX_MESSAGE_TIMEOUT_MS),
	};

	return modem_ubx_init(&data->ubx, &ubx_config);
}

static int u_blox_m10_release_chat_attach_ubx(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;

	// Release chat, attach ubx.
	modem_chat_release(&data->chat);
	ret = modem_ubx_attach(&data->ubx, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
	}

	return ret;
}

static int u_blox_m10_release_ubx_attach_chat(const struct device *dev)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;

	// Release ubx, attach chat.
	modem_ubx_release(&data->ubx);
	ret = modem_chat_attach(&data->chat, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
	}

	return ret;
}

static int u_blox_m10_configure_baudrate(const struct device *dev, uint32_t baudrate)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;
	bool script_failure = true;

	ret = u_blox_m10_release_chat_attach_ubx(dev);
	if (ret < 0) {
		goto out;
	}

	// Send UBX_CFG_PRT to change device baudrate.
	uint8_t ubx_frame[128];
	uint8_t ubx_frame_size;
	u_blox_get_cfg_prt(ubx_frame, &ubx_frame_size, 0x01, baudrate);
	struct modem_ubx_frame modem_ubx_frame = {
		.ubx_frame = ubx_frame,
		.ubx_frame_size = ubx_frame_size,
	};

	ret = modem_ubx_transmit(&data->ubx, &modem_ubx_frame);
	if (ret == 0) {
		script_failure = false;
	}
	if (ret < 0) {
		LOG_ERR("modem_ubx_transmit failed. exiting.");
		goto out;
	}

out:
	ret = u_blox_m10_release_ubx_attach_chat(dev);
	if (ret < 0) {
		return ret;
	}
	if (script_failure) {
		return -1;
	}

	LOG_ERR("modem_ubx_transmit: exited cleanly (temp) &&&&&&&&&&&&&&&&&&&&&.");
	return 0;
}

static int u_blox_m10_get_uart_baudrate(const struct device *dev) {
	const struct u_blox_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_config;

	uart_api->config_get(cfg->uart, &uart_config);
	uint32_t baudrate = uart_config.baudrate;

	return baudrate;
}

static int u_blox_m10_set_uart_baudrate(const struct device *dev, uint32_t baudrate) {
	const struct u_blox_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_config;

	u_blox_m10_turn_off(dev);

	uart_api->config_get(cfg->uart, &uart_config);
	uart_config.baudrate = baudrate;

	int ret = uart_api->configure(cfg->uart, &uart_config);

	u_blox_m10_init_pipe(dev);
	u_blox_m10_resume(dev);

	return ret;
}

static int u_blox_m10_configure(const struct device *dev)
{
	int ret, retry_count = 10;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	bool configuration_failed = true;
	/* Try configuring baudrate of device with all possible baudrates. */
	for (int i = 0; i < U_BLOX_BAUDRATE_COUNT; ++i) {
		/* Set baudrate of UART pipe as u_blox_baudrate[i]. */
		ret = u_blox_m10_set_uart_baudrate(dev, u_blox_baudrate[i]);
		if (ret < 0) {
			return ret;
		}

		/* Try setting baudrate of device as target_baudrate. */
		ret = u_blox_m10_configure_baudrate(dev, target_baudrate);
		if (ret == 0) {
			configuration_failed = false;
			break;
		}
	}

	/* Reset baudrate of UART pipe as target_baudrate. */
	ret = u_blox_m10_set_uart_baudrate(dev, target_baudrate);
	if (ret < 0) {
		return ret;
	}

	/* Retry in case didn't receive acknowledgement in previous attempts. */
	for (int j = 0; j < retry_count && configuration_failed; ++j) {
		ret = u_blox_m10_configure_baudrate(dev, target_baudrate);
		if (ret == 0) {
			configuration_failed = false;
			break;
		}
	}

	/* Benchmarking (temp). */
	int total_count = 10, success_count = 0;
	for (int j = 0; j < total_count; ++j) {
		ret = u_blox_m10_configure_baudrate(dev, target_baudrate);
		if (ret == 0) {
			++success_count;
		}
	}
	LOG_ERR("success rate = %d/%d.", success_count,total_count);

	if (configuration_failed) {
		return -1;
	}

	LOG_ERR("u_blox_m10_configure: exited cleanly (temp) &&&&&&&&&&&&&&&&&&&&&.");
	return ret;
}

static int u_blox_m10_init(const struct device *dev)
{
	int ret;
	printk("\n\n\nu_blox_m10_init beginning (temp).\n");

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
