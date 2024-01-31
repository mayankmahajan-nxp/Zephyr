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

#include "u_blox_protocol/u_blox_protocol.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define DT_DRV_COMPAT u_blox_m10

#define UART_RECV_BUF_SZ	128
#define UART_TRNF_BUF_SZ	128

#define CHAT_RECV_BUF_SZ	256
#define CHAT_ARGV_SZ		32

#define UBX_RECV_BUF_SZ		128
#define UBX_WORK_BUF_SZ		128
#define UBX_SUPP_BUF_SZ		128

#define UBX_MESSAGE_TIMEOUT_MS	500

#define U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_name, ubx_frame, ubx_frame_len, retry_count)	\
	struct modem_ubx_script script_name = {							\
		.ubx_frame = ubx_frame,								\
		.ubx_frame_size = &ubx_frame_len,						\
		.retry_count = retry_count,							\
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
	uint8_t ubx_supp_buf[UBX_SUPP_BUF_SZ];

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

static int u_blox_m10_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	return 0;
}
static int u_blox_m10_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	return 0;
}
static int u_blox_m10_set_periodic_config(const struct device *dev,
					  const struct gnss_periodic_config *periodic_config)
{
	return 0;
}
static int u_blox_m10_get_periodic_config(const struct device *dev,
					  struct gnss_periodic_config *periodic_config)
{
	return 0;
}
static int u_blox_m10_set_navigation_mode(const struct device *dev, enum gnss_navigation_mode mode)
{
	return 0;
}
static int u_blox_m10_get_navigation_mode(const struct device *dev, enum gnss_navigation_mode *mode)
{
	return 0;
}
static int u_blox_m10_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	return 0;
}
static int u_blox_m10_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	return 0;
}

static int u_blox_m10_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_SBAS | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_IMES);
	return 0;
}

static struct gnss_driver_api gnss_api = {
	.set_fix_rate = u_blox_m10_set_fix_rate,
	.get_fix_rate = u_blox_m10_get_fix_rate,
	.set_periodic_config = u_blox_m10_set_periodic_config,
	.get_periodic_config = u_blox_m10_get_periodic_config,
	.set_navigation_mode = u_blox_m10_set_navigation_mode,
	.get_navigation_mode = u_blox_m10_get_navigation_mode,
	.set_enabled_systems = u_blox_m10_set_enabled_systems,
	.get_enabled_systems = u_blox_m10_get_enabled_systems,
	.get_supported_systems = u_blox_m10_get_supported_systems,
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
		.supplementary_buf = data->ubx_supp_buf,
		.supplementary_buf_size = sizeof(data->ubx_supp_buf),
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
					   struct modem_ubx_script *modem_ubx_script_tx) {
	struct u_blox_m10_data *data = dev->data;
	int ret;

	ret = u_blox_m10_release_chat_attach_ubx(dev);
	if (ret < 0) {
		goto out;
	}

	ret = modem_ubx_transmit(&data->ubx, modem_ubx_script_tx);
	if (ret < 0) {
		goto out;
	}

out:
	ret |= u_blox_m10_release_ubx_attach_chat(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_modem_ubx_script_send: failed (temp). ret = %d.", ret);
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_prt_get_send(const struct device *dev, uint16_t retry_count)
{
	int ret;

	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_len;
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	u_blox_get_cfg_prt_get(script_inst.ubx_frame, script_inst.ubx_frame_size, PORT_NUMBER_UART);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < *script_inst.ubx_frame_size; ++i)
		printk("%x ", script_inst.ubx_frame[i]);
	printk("\n");

	return 0;
}

static int u_blox_m10_ubx_cfg_prt_set_send(const struct device *dev, uint32_t baudrate,
					   uint16_t retry_count)
{
	int ret;

	// Send UBX_CFG_PRT to change device baudrate.
	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_len;
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	u_blox_get_cfg_prt_set(script_inst.ubx_frame, script_inst.ubx_frame_size, PORT_NUMBER_UART, baudrate);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_rst_send(const struct device *dev, uint8_t reset_mode)
{
	int ret, retry_count = 1;

	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_len;
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	u_blox_get_cfg_rst(script_inst.ubx_frame, script_inst.ubx_frame_size, reset_mode);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
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
	ret = u_blox_m10_resume(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure_baudrate_prerequisite(const struct device *dev) {
	int ret;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	/* Try configuring baudrate of device with all possible baudrates. */
	for (int i = 0; i < U_BLOX_BAUDRATE_COUNT; ++i) {
		/* Set baudrate of UART pipe as u_blox_baudrate[i]. */
		ret = u_blox_m10_set_uart_baudrate(dev, u_blox_baudrate[i]);
		if (ret < 0) {
			return ret;
		}

		/* Try setting baudrate of device as target_baudrate. */
		ret = u_blox_m10_ubx_cfg_prt_set_send(dev, target_baudrate, 2);
		if (ret == 0) {
			break;
		}
	}

	/* Reset baudrate of UART pipe as target_baudrate. */
	ret = u_blox_m10_set_uart_baudrate(dev, target_baudrate);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(U_BLOX_CFG_PRT_WAIT_MS));

	return ret;
}

static int u_blox_m10_configure_baudrate(const struct device *dev) {
	int ret;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	ret = u_blox_m10_ubx_cfg_prt_set_send(dev, target_baudrate, 10);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure_messages(const struct device *dev) {
	int ret = 0, retry_count = 7;

	uint8_t ubx_frame[U_BLOX_MESSAGE_LEN_MAX];
	uint16_t ubx_frame_len;
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GGA, 1);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_RMC, 1);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GSV, 1);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_DTM, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GBS, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GLL, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GNS, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GRS, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GSA, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_GST, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_VLW, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_VTG, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	u_blox_get_cfg_msg(script_inst.ubx_frame, script_inst.ubx_frame_size, NMEA_ZDA, 0);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure(const struct device *dev)
{
	int ret;

	u_blox_m10_configure_baudrate_prerequisite(dev);

	u_blox_m10_ubx_cfg_rst_send(dev, 0x08);
	k_sleep(K_MSEC(U_BLOX_CFG_RST_WAIT_MS));

	ret = u_blox_m10_configure_baudrate(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_configure_baudrate failed. exiting.");
		goto out;
	}

	ret = u_blox_m10_configure_messages(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_configure_messages failed. exiting.");
		goto out;
	}

	ret = u_blox_m10_ubx_cfg_prt_get_send(dev, 7);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_ubx_cfg_prt_get_send failed. exiting.");
		goto out;
	}

out:
	u_blox_m10_ubx_cfg_rst_send(dev, 0x09);

	if (ret < 0) {
		return ret;
	}

	LOG_ERR("u_blox_m10_configure: exited cleanly (temp) &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.");
	return 0;
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

	k_sleep(K_MSEC(4000));

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
