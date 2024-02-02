/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Referred from the file "zephyr/drivers/gnss/gnss_nmea_generic.c". */

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

#include "gnss_u_blox_protocol/gnss_u_blox_protocol.h"

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

	modem_ubx_release(&data->ubx);
	ret = modem_chat_attach(&data->chat, data->uart_pipe);
	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
	}

	return ret;
}

static int u_blox_m10_modem_ubx_script_send(const struct device *dev,
					    struct modem_ubx_script *modem_ubx_script_tx)
{
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
		LOG_ERR("%s: failed %d.", __func__, ret);
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_prt_get_send(const struct device *dev, uint16_t retry_count)
{
	int ret;
	printk("config prt get being sent.\n");
	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	struct u_blox_cfg_prt_get_data data = { .port_id = UBX_PORT_NUMBER_UART };
	ubx_frame_len = u_blox_cfg_prt_get(script_inst.ubx_frame, ubx_frame_size, &data);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_prt_set_send(const struct device *dev, uint32_t baudrate,
					   uint16_t retry_count)
{
	int ret;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	struct u_blox_cfg_prt_set_data data;
	u_blox_cfg_prt_set_data_default(&data);
	data.baudrate = 9600;
	ubx_frame_len = u_blox_cfg_prt_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_rst_set_send(const struct device *dev, uint8_t reset_mode)
{
	int ret, retry_count = 2;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	struct u_blox_cfg_rst_set_data data = {
		. nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START,
		.reset_mode = reset_mode
	};
	ubx_frame_len = u_blox_cfg_rst_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_get_uart_baudrate(const struct device *dev)
{
	const struct u_blox_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_cfg;

	uart_api->config_get(cfg->uart, &uart_cfg);
	uint32_t baudrate = uart_cfg.baudrate;

	return baudrate;
}

static int u_blox_m10_set_uart_baudrate(const struct device *dev, uint32_t baudrate)
{
	const struct u_blox_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_cfg;

	u_blox_m10_turn_off(dev);

	uart_api->config_get(cfg->uart, &uart_cfg);
	uart_cfg.baudrate = baudrate;

	int ret = uart_api->configure(cfg->uart, &uart_cfg);

	u_blox_m10_init_pipe(dev);
	ret = u_blox_m10_resume(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure_baudrate_prerequisite(const struct device *dev)
{
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

static int u_blox_m10_configure_baudrate(const struct device *dev)
{
	int ret;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	ret = u_blox_m10_ubx_cfg_prt_set_send(dev, target_baudrate, 10);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure_messages(const struct device *dev)
{
	int ret = 0, retry_count = 7;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	struct u_blox_cfg_msg_set_data data;
	u_blox_cfg_msg_set_data_default(&data);

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);

	data.message_id = UBX_NMEA_GGA;
	data.rate = 1;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_RMC;
	data.rate = 1;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GSV;
	data.rate = 1;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_DTM;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GBS;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GLL;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GNS;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GRS;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GSA;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_GST;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_VLW;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_VTG;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	data.message_id = UBX_NMEA_ZDA;
	data.rate = 0;
	ubx_frame_len = u_blox_cfg_msg_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret |= u_blox_m10_modem_ubx_script_send(dev, &script_inst);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	return 0;
}

static int u_blox_m10_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	return 0;
}

static int u_blox_m10_set_navigation_mode(const struct device *dev, enum gnss_navigation_mode mode)
{
	enum ubx_dynamic_model d_model = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	switch (mode) {
	case GNSS_NAVIGATION_MODE_ZERO_DYNAMICS:
		d_model = UBX_DYN_MODEL_Stationary;
		break;
	case GNSS_NAVIGATION_MODE_LOW_DYNAMICS:
		d_model = UBX_DYN_MODEL_Portable;
		break;
	case GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS:
		d_model = UBX_DYN_MODEL_Airbone1G;
		break;
	case GNSS_NAVIGATION_MODE_HIGH_DYNAMICS:
		d_model = UBX_DYN_MODEL_Airbone4G;
		break;
	default:
		break;
	}

	int ret, retry_count = 7;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	struct u_blox_cfg_nav5_set_data data;
	u_blox_cfg_nav5_set_data_default(&data);
	data.dyn_model = d_model;
	ubx_frame_len = u_blox_cfg_nav5_set(script_inst.ubx_frame, ubx_frame_size, &data);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_get_navigation_mode(const struct device *dev, enum gnss_navigation_mode *mode)
{
	int ret, retry_count = 7;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	ubx_frame_len = u_blox_cfg_nav5_get(script_inst.ubx_frame, ubx_frame_size);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	switch (script_inst.ubx_frame[8]) {
	case UBX_DYN_MODEL_Stationary:
		*mode = GNSS_NAVIGATION_MODE_ZERO_DYNAMICS;
		break;
	case UBX_DYN_MODEL_Portable:
		*mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
		break;
	case UBX_DYN_MODEL_Airbone1G:
		*mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
		break;
	case UBX_DYN_MODEL_Airbone4G:
		*mode = GNSS_NAVIGATION_MODE_HIGH_DYNAMICS;
		break;
	default:
		break;
	}

	return 0;
}

static int u_blox_m10_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	uint16_t config_size_max = 7 * 8, config_len = 0;
	uint8_t config_gnss[config_size_max];

	for (int i = 0; i < 8; ++i) {
		if (systems & (1 << i)) {
			uint64_t val = 0;

			switch (systems & (1 << i)) {
			case GNSS_SYSTEM_GPS:
				val = 0x0101000100160800;
				break;
			case GNSS_SYSTEM_GLONASS:
				val = 0x0101000100140806;
				break;
			case GNSS_SYSTEM_GALILEO:
				val = 0x0101000100030002;
				break;
			case GNSS_SYSTEM_BEIDOU:
				val = 0x0101000000160803;
				break;
			case GNSS_SYSTEM_QZSS:
				val = 0x0101000100030005;
				break;
			case GNSS_SYSTEM_SBAS:
				val = 0x0101000100030101;
				break;
			default:
				break;
			};

			memcpy(config_gnss + config_len, &val, 8);
			config_len += 8;
		}
	}

	int ret, retry_count = 7;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	ubx_frame_len = u_blox_cfg_gnss_set(script_inst.ubx_frame, ubx_frame_size, 0, 32,
				config_gnss, config_len);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	int ret, retry_count = 7;

	uint16_t ubx_frame_size = U_BLOX_MESSAGE_LEN_MAX;
	uint8_t ubx_frame[ubx_frame_size];
	uint16_t ubx_frame_len;

	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frame, ubx_frame_len, retry_count);
	ubx_frame_len = u_blox_cfg_gnss_get(script_inst.ubx_frame, ubx_frame_size);
	ret = u_blox_m10_modem_ubx_script_send(dev, &script_inst);
	if (ret < 0) {
		return ret;
	}

	for (int i = 10; i < ubx_frame_len - 2; i += 8) {
		switch (script_inst.ubx_frame[i]) {
		case UBX_GNSS_ID_GPS:
			*systems |= GNSS_SYSTEM_GPS;
			break;
		case UBX_GNSS_ID_SBAS:
			*systems |= GNSS_SYSTEM_SBAS;
			break;
		case UBX_GNSS_ID_Galileo:
			*systems |= GNSS_SYSTEM_GALILEO;
			break;
		case UBX_GNSS_ID_BeiDou:
			*systems |= GNSS_SYSTEM_BEIDOU;
			break;
		case UBX_GNSS_ID_QZSS:
			*systems |= GNSS_SYSTEM_QZSS;
			break;
		case UBX_GNSS_ID_GLONAS:
			*systems |= GNSS_SYSTEM_GLONASS;
			break;
		default:
			break;
		};
	}

	return 0;
}

static int u_blox_m10_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_SBAS | GNSS_SYSTEM_QZSS);
	return 0;
}

static struct gnss_driver_api gnss_api = {
	.set_fix_rate = u_blox_m10_set_fix_rate,
	.get_fix_rate = u_blox_m10_get_fix_rate,
	.set_navigation_mode = u_blox_m10_set_navigation_mode,
	.get_navigation_mode = u_blox_m10_get_navigation_mode,
	.set_enabled_systems = u_blox_m10_set_enabled_systems,
	.get_enabled_systems = u_blox_m10_get_enabled_systems,
	.get_supported_systems = u_blox_m10_get_supported_systems,
};

static int u_blox_m10_configure(const struct device *dev)
{
	int ret;

	u_blox_m10_configure_baudrate_prerequisite(dev);

	u_blox_m10_ubx_cfg_rst_set_send(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_STOP);
	k_sleep(K_MSEC(U_BLOX_CFG_RST_WAIT_MS));

	ret = u_blox_m10_configure_baudrate(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_configure_baudrate failed %d. exiting.", ret);
		goto out;
	}

	ret = u_blox_m10_configure_messages(dev);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_configure_messages failed %d. exiting.", ret);
		goto out;
	}

	ret = u_blox_m10_ubx_cfg_prt_get_send(dev, 7);
	if (ret < 0) {
		LOG_ERR("u_blox_m10_ubx_cfg_prt_get_send failed %d. exiting.", ret);
		goto out;
	}

out:
	u_blox_m10_ubx_cfg_rst_set_send(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_START);

	gnss_systems_t systems = 0;

	printk("u_blox_m10_get_enabled_systems.\n");
	u_blox_m10_get_enabled_systems(dev, &systems);
	systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_GALILEO
		| GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_SBAS | GNSS_SYSTEM_QZSS;
	printk("u_blox_m10_set_enabled_systems.\n");
	u_blox_m10_set_enabled_systems(dev, systems);
	systems = 0;
	printk("u_blox_m10_get_enabled_systems.\n");
	u_blox_m10_get_enabled_systems(dev, &systems);

	enum gnss_navigation_mode nav_mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
	u_blox_m10_get_navigation_mode(dev, &nav_mode);
	printk("u_blox_m10_get_navigation_mode. %d\n", nav_mode);
	nav_mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	u_blox_m10_set_navigation_mode(dev, nav_mode);
	printk("u_blox_m10_set_navigation_mode. %d\n", nav_mode);
	nav_mode = 0;
	u_blox_m10_get_navigation_mode(dev, &nav_mode);
	printk("u_blox_m10_get_navigation_mode. %d\n", nav_mode);

	if (ret < 0) {
		return ret;
	}

	LOG_ERR("%s: exited cleanly.", __func__);
	return 0;
}

static int u_blox_m10_init(const struct device *dev)
{
	int ret;

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
