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

enum modem_module {
	modem_module_chat = 0,
	modem_module_ubx = 1,
};

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

static int u_blox_m10_modem_ubx_script_send(const struct device *dev,
					   struct modem_ubx_script_ubx *modem_ubx_script_ubx_tx) {
	struct u_blox_m10_data *data = dev->data;
	int ret;

	ret = u_blox_m10_release_chat_attach_ubx(dev);
	if (ret < 0) {
		goto out;
	}

	ret = modem_ubx_transmit(&data->ubx, modem_ubx_script_ubx_tx);
	if (ret < 0) {
		goto out;
	}

out:
	ret |= u_blox_m10_release_ubx_attach_chat(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_modem_ubx_script_send: failed (temp). ret = %d.", ret);
		return ret;
	}

	LOG_ERR("u_blox_m10_modem_ubx_script_send: exited cleanly (temp) &&&&&&&&&&&&&&&&&&&&&.");
	return 0;
}

static int u_blox_m10_ubx_cfg_prt_send(const struct device *dev, uint32_t baudrate, uint16_t retry_count)
{
	int ret;

	// Send UBX_CFG_PRT to change device baudrate.
	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_size;
	u_blox_get_cfg_prt(ubx_frame, &ubx_frame_size, 0x01, baudrate);
	struct modem_ubx_script_ubx modem_ubx_script_ubx_cfg_prt = {
		.ubx_frame = ubx_frame,
		.ubx_frame_size = ubx_frame_size,
		.retry_count = retry_count,
	};
	ret = u_blox_m10_modem_ubx_script_send(dev, &modem_ubx_script_ubx_cfg_prt);
	if (ret < 0) {
		return ret;
	}

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
	int ret;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	bool configuration_failed = true;
	/* Try configuring baudrate of device with all possible baudrates. */
	for (int i = 0; i < U_BLOX_BAUDRATE_COUNT; ++i) {
		/* Set baudrate of UART pipe as u_blox_baudrate[i]. */
		ret = u_blox_m10_set_uart_baudrate(dev, u_blox_baudrate[i]);
		if (ret < 0) {
			return ret;
		}
		printk("%d\n", u_blox_baudrate[i]);

		/* Try setting baudrate of device as target_baudrate. */
		ret = u_blox_m10_ubx_cfg_prt_send(dev, target_baudrate, 2);
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

	/* Retry in case didn't receive acknowledgement in previous attempt. */
	ret = u_blox_m10_ubx_cfg_prt_send(dev, target_baudrate, 5);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_modem_ubx_script_send cfg_prt failed. exiting.");
		return ret;
	}

	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_size;
	struct modem_ubx_script_ubx modem_ubx_script_ubx_cfg_msg;
	u_blox_get_cfg_msg(ubx_frame, &ubx_frame_size, NMEA_DTM, 0);
	modem_ubx_script_ubx_cfg_msg.ubx_frame = ubx_frame;
	modem_ubx_script_ubx_cfg_msg.ubx_frame_size = ubx_frame_size;
	modem_ubx_script_ubx_cfg_msg.retry_count = 5;
	ret = u_blox_m10_modem_ubx_script_send(dev, &modem_ubx_script_ubx_cfg_msg);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_modem_ubx_script_send cfg_msg failed. exiting.");
		return ret;
	}

	// ret &= neo_api->cfg_msg(neo_dev, NMEA_DTM, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GBS, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GLL, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GNS, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GRS, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GSA, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GST, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_GSV, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_RMC, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_VLW, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_VTG, 0);
	// ret &= neo_api->cfg_msg(neo_dev, NMEA_ZDA, 0);

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
