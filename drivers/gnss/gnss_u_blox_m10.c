/*
 * Copyright 2024 NXP
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

#include <zephyr/drivers/gnss/gnss_u_blox_protocol.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ubx_m10, CONFIG_GNSS_LOG_LEVEL);

#define DT_DRV_COMPAT u_blox_m10

#define UART_RECV_BUF_SZ	128
#define UART_TRNF_BUF_SZ	128

#define CHAT_RECV_BUF_SZ	256
#define CHAT_ARGV_SZ		32

#define UBX_RECV_BUF_SZ		UBX_FRM_SZ_MAX
#define UBX_TRNS_BUF_SZ		UBX_FRM_SZ_MAX
#define UBX_WORK_BUF_SZ		UBX_FRM_SZ_MAX

#define UBX_FRM_BUF_SZ		UBX_FRM_SZ_MAX

#define MODEM_UBX_FRM_TIMEOUT_MS	500

#define UBX_M10_GNSS_SYS_CNT_MAX	6

enum MODEM_MODULE {
	MODEM_MODULE_CHAT = 0,
	MODEM_MODULE_UBX,
};

struct ubx_m10_config {
	const struct device *uart;
};

struct ubx_m10_data {
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

	/* Ubx frame */
	uint8_t ubx_frame_buf[UBX_FRM_BUF_SZ];

	struct k_spinlock lock;
};

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

static int ubx_m10_resume(const struct device *dev)
{
	struct ubx_m10_data *data = dev->data;
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

static int ubx_m10_turn_off(const struct device *dev)
{
	struct ubx_m10_data *data = dev->data;

	return modem_pipe_close(data->uart_pipe);
}

static int ubx_m10_init_nmea0183_match(const struct device *dev)
{
	struct ubx_m10_data *data = dev->data;

	const struct gnss_nmea0183_match_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &match_config);
}

static void ubx_m10_init_pipe(const struct device *dev)
{
	const struct ubx_m10_config *cfg = dev->config;
	struct ubx_m10_data *data = dev->data;

	const struct modem_backend_uart_config uart_backend_config = {
		.uart = cfg->uart,
		.receive_buf = data->uart_backend_receive_buf,
		.receive_buf_size = sizeof(data->uart_backend_receive_buf),
		.transmit_buf = data->uart_backend_transmit_buf,
		.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
	};

	data->uart_pipe = modem_backend_uart_init(&data->uart_backend, &uart_backend_config);
}

static uint8_t ubx_m10_char_delimiter[] = {'\r', '\n'};

static int ubx_m10_init_chat(const struct device *dev)
{
	struct ubx_m10_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = sizeof(data->chat_receive_buf),
		.delimiter = ubx_m10_char_delimiter,
		.delimiter_size = ARRAY_SIZE(ubx_m10_char_delimiter),
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

static int ubx_m10_init_ubx(const struct device *dev)
{
	struct ubx_m10_data *data = dev->data;

	const struct modem_ubx_config ubx_config = {
		.user_data = data,
		.receive_buf = data->ubx_receive_buf,
		.receive_buf_size = sizeof(data->ubx_receive_buf),
		.work_buf = data->ubx_work_buf,
		.work_buf_size = sizeof(data->ubx_work_buf),
		.process_timeout = K_MSEC(MODEM_UBX_FRM_TIMEOUT_MS),
	};

	return modem_ubx_init(&data->ubx, &ubx_config);
}

static int ubx_m10_modem_module_switch(const struct device *dev,
				       enum MODEM_MODULE release,
				       enum MODEM_MODULE attach) {
	struct ubx_m10_data *data = dev->data;
	int ret;

	switch (release) {
	case MODEM_MODULE_CHAT:
		modem_chat_release(&data->chat);
		break;
	case MODEM_MODULE_UBX:
		modem_ubx_release(&data->ubx);
		break;
	default:
		return -1;
	}

	switch (attach) {
	case MODEM_MODULE_CHAT:
		ret = modem_chat_attach(&data->chat, data->uart_pipe);
		break;
	case MODEM_MODULE_UBX:
		ret = modem_ubx_attach(&data->ubx, data->uart_pipe);
		break;
	default:
		return -1;
	}

	if (ret < 0) {
		modem_pipe_close(data->uart_pipe);
	}

	return ret;
}

static int ubx_m10_modem_ubx_run_script(const struct device *dev,
					struct modem_ubx_script *modem_ubx_script_tx)
{
	struct ubx_m10_data *data = dev->data;
	int ret;

	ret = ubx_m10_modem_module_switch(dev, MODEM_MODULE_CHAT, MODEM_MODULE_UBX);
	if (ret < 0) {
		goto out;
	}

	ret = modem_ubx_run_script(&data->ubx, modem_ubx_script_tx);
	if (ret < 0) {
		goto out;
	}

out:
	ret |= ubx_m10_modem_module_switch(dev, MODEM_MODULE_UBX, MODEM_MODULE_CHAT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void ubx_m10_modem_ubx_script_fill(const struct device *dev, struct modem_ubx_script *script,
					  uint8_t *ubx_frame, uint8_t retry)
{
	script->ubx_frame = ubx_frame;
	script->retry_count = retry;
}

static int ubx_m10_modem_ubx_script_init(const struct device *dev, struct modem_ubx_script *script,
					 uint8_t *ubx_frame, uint16_t ubx_frame_size, uint8_t retry,
					 void *frame_data, enum ubx_msg_class msg_cls,
					 enum ubx_config_message msg_id, uint16_t payload_size)
{
	int ret;

	(void) ubx_m10_modem_ubx_script_fill(dev, script, ubx_frame, retry);

	ret = ubx_create_frame(ubx_frame, ubx_frame_size, msg_cls, msg_id, frame_data,
			       payload_size);
	return ret;
}

static int ubx_m10_ubx_cfg_prt_set(const struct device *dev, uint32_t target_baudrate,
				   uint8_t retry)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	UBX_CFG_PRT_SET_DATA_INIT(frame_data)
	frame_data.baudrate = target_baudrate;

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), retry, &frame_data, UBX_CLASS_CFG, UBX_CFG_PRT,
		UBX_CFG_PRT_SET_PAYLOAD_SZ);

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_ubx_cfg_rst(const struct device *dev, uint8_t reset_mode)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	int retry = 2;
	UBX_CFG_RST_DATA_INIT(frame_data)
	frame_data.nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	frame_data.reset_mode = reset_mode;

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), retry, &frame_data, UBX_CLASS_CFG, UBX_CFG_RST,
		UBX_CFG_RST_PAYLOAD_SZ);

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_get_uart_baudrate(const struct device *dev)
{
	const struct ubx_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_cfg;

	uart_api->config_get(cfg->uart, &uart_cfg);
	uint32_t baudrate = uart_cfg.baudrate;

	return baudrate;
}

static int ubx_m10_set_uart_baudrate(const struct device *dev, uint32_t baudrate)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);
	const struct ubx_m10_config *cfg = dev->config;

	const struct uart_driver_api *uart_api = cfg->uart->api;
	struct uart_config uart_cfg;

	ubx_m10_turn_off(dev);

	uart_api->config_get(cfg->uart, &uart_cfg);
	uart_cfg.baudrate = baudrate;

	ret = uart_api->configure(cfg->uart, &uart_cfg);

	ubx_m10_init_pipe(dev);
	ret = ubx_m10_resume(dev);
	if (ret < 0) {
		goto out;
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_configure_gnss_device_baudrate_prerequisite(const struct device *dev)
{
	int ret, retry_count = 2;

	int target_baudrate = ubx_m10_get_uart_baudrate(dev);

	/* Try communication with device with all possible baudrates. */
	for (int i = 0; i < UBX_BAUDRATE_COUNT; ++i) {
		/* Set baudrate of UART pipe as ubx_baudrate[i]. */
		ret = ubx_m10_set_uart_baudrate(dev, ubx_baudrate[i]);
		if (ret < 0) {
			return ret;
		}

		/* Try setting baudrate of device as target_baudrate. */
		ret = ubx_m10_ubx_cfg_prt_set(dev, target_baudrate, retry_count);
		if (ret == 0) {
			break;
		}
	}

	/* Reset baudrate of UART pipe as target_baudrate. */
	ret = ubx_m10_set_uart_baudrate(dev, target_baudrate);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ubx_m10_configure_gnss_device_baudrate(const struct device *dev)
{
	int ret;

	int target_baudrate = ubx_m10_get_uart_baudrate(dev);

	ret = ubx_m10_ubx_cfg_prt_set(dev, target_baudrate, MODEM_UBX_RETRY_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ubx_m10_configure_messages(const struct device *dev)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	UBX_CFG_MSG_DATA_INIT(frame_data)

	struct modem_ubx_script script;
	ubx_m10_modem_ubx_script_fill(dev, &script, data->ubx_frame_buf, MODEM_UBX_RETRY_DEFAULT);

	/* Enabling GGA, RMC and GSV messages. */
	frame_data.rate = 1;
	uint8_t message_enable[] = {UBX_NMEA_GGA, UBX_NMEA_RMC, UBX_NMEA_GSV};

	for (int i = 0; i < sizeof(message_enable); ++i) {
		frame_data.message_id = message_enable[i];
		script.ubx_frame_size = ubx_create_frame(script.ubx_frame, UBX_CFG_MSG_FRM_SZ,
			UBX_CLASS_CFG, UBX_CFG_MSG, &frame_data, UBX_CFG_MSG_PAYLOAD_SZ);

		ret = ubx_m10_modem_ubx_run_script(dev, &script);
		if (ret < 0) {
			goto out;
		}
	}

	/* Disabling DTM, GBS, GLL, GNS, GRS, GSA, GST, VLW, VTG and ZDA messages. */
	frame_data.rate = 0;
	uint8_t message_disable[] = {UBX_NMEA_DTM, UBX_NMEA_GBS, UBX_NMEA_GLL, UBX_NMEA_GNS,
				     UBX_NMEA_GRS, UBX_NMEA_GSA, UBX_NMEA_GST, UBX_NMEA_VLW,
				     UBX_NMEA_VTG, UBX_NMEA_ZDA};

	for (int i = 0; i < sizeof(message_disable); ++i) {
		frame_data.message_id = message_disable[i];
		script.ubx_frame_size = ubx_create_frame(script.ubx_frame, UBX_CFG_MSG_FRM_SZ,
			UBX_CLASS_CFG, UBX_CFG_MSG, &frame_data, UBX_CFG_MSG_PAYLOAD_SZ);

		ret = ubx_m10_modem_ubx_run_script(dev, &script);
		if (ret < 0) {
			goto out;
		}
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_set_navigation_mode(const struct device *dev, enum gnss_navigation_mode mode)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	enum ubx_dynamic_model dynamic_model = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	switch (mode) {
	case GNSS_NAVIGATION_MODE_ZERO_DYNAMICS:
		dynamic_model = UBX_DYN_MODEL_STATIONARY;
		break;
	case GNSS_NAVIGATION_MODE_LOW_DYNAMICS:
		dynamic_model = UBX_DYN_MODEL_PORTABLE;
		break;
	case GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS:
		dynamic_model = UBX_DYN_MODEL_AIRBONE1G;
		break;
	case GNSS_NAVIGATION_MODE_HIGH_DYNAMICS:
		dynamic_model = UBX_DYN_MODEL_AIRBONE4G;
		break;
	default:
		break;
	}

	UBX_CFG_NAV5_DATA_INIT(frame_data)
	frame_data.dyn_model = dynamic_model;

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), MODEM_UBX_RETRY_DEFAULT, &frame_data, UBX_CLASS_CFG,
		UBX_CFG_NAV5, UBX_CFG_NAV5_PAYLOAD_SZ);

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_get_navigation_mode(const struct device *dev, enum gnss_navigation_mode *mode)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), MODEM_UBX_RETRY_DEFAULT, NULL, UBX_CLASS_CFG,
		UBX_CFG_NAV5, UBX_FRM_GET_PAYLOAD_SZ);

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

	struct ubx_frame_t *frame = (struct ubx_frame_t *) data->ubx_frame_buf;

	switch (((struct ubx_cfg_nav5_data *)frame->payload_and_checksum)->dyn_model) {
	case UBX_DYN_MODEL_STATIONARY:
		*mode = GNSS_NAVIGATION_MODE_ZERO_DYNAMICS;
		break;
	case UBX_DYN_MODEL_PORTABLE:
		*mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
		break;
	case UBX_DYN_MODEL_AIRBONE1G:
		*mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
		break;
	case UBX_DYN_MODEL_AIRBONE4G:
		*mode = GNSS_NAVIGATION_MODE_HIGH_DYNAMICS;
		break;
	default:
		break;
	}

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	struct ubx_cfg_gnss_data *frame_data;

	uint8_t cfg_blocks_count = 0;
	gnss_systems_t temp = systems;
	while (temp > 0) {
	        ++cfg_blocks_count;
		temp = temp & (temp - 1);
	}

	frame_data = malloc(sizeof(*frame_data) + sizeof(struct ubx_cfg_gnss_data_config_block) * cfg_blocks_count);
	frame_data->num_config_blocks = cfg_blocks_count;

	(void) ubx_cfg_gnss_data_default(frame_data);

	uint8_t filled_blocks = 0;
	for (int i = 0; i < 8; ++i) {
		if (systems & (1 << i)) {
			switch (systems & (1 << i)) {
			case GNSS_SYSTEM_GPS:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_GPS;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_GPS_L1C_A;
				break;
			case GNSS_SYSTEM_GLONASS:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_GLONAS;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_GLONASS_L1;
				break;
			case GNSS_SYSTEM_GALILEO:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_GALILEO;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_GALILEO_E1;
				break;
			case GNSS_SYSTEM_BEIDOU:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_BEIDOU;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_BEIDOU_B1I;
				break;
			case GNSS_SYSTEM_QZSS:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_QZSS;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_QZSS_L1C_A;
				break;
			case GNSS_SYSTEM_SBAS:
				frame_data->config_blocks[filled_blocks].gnss_id = UBX_GNSS_ID_SBAS;
				frame_data->config_blocks[filled_blocks].flags =
					UBX_CFG_GNSS_FLAG_ENABLE |
					UBX_CFG_GNSS_FLAG_SGN_CNF_SBAS_L1C_A;
				break;
			default:
				break;
			};
			++filled_blocks;
		}
	}

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), MODEM_UBX_RETRY_DEFAULT, frame_data, UBX_CLASS_CFG,
		UBX_CFG_GNSS, UBX_CFG_GNSS_PAYLOAD_SZ(cfg_blocks_count));

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

out:
	free(frame_data);

	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	int ret;
	k_spinlock_key_t key;
	struct ubx_m10_data *data = dev->data;

	key = k_spin_lock(&data->lock);

	struct modem_ubx_script script;
	script.ubx_frame_size = ubx_m10_modem_ubx_script_init(dev, &script, data->ubx_frame_buf,
		sizeof(data->ubx_frame_buf), MODEM_UBX_RETRY_DEFAULT, NULL, UBX_CLASS_CFG,
		UBX_CFG_GNSS, UBX_FRM_GET_PAYLOAD_SZ);

	ret = ubx_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

	for (int i = 10; i < script.ubx_frame_size - UBX_CHECKSUM_STOP_IDX_FROM_END;
	     i += UBX_CFG_GNSS_PAYLOAD_CFG_BLK_SZ) {
		switch (script.ubx_frame[i]) {
		case UBX_GNSS_ID_GPS:
			*systems |= GNSS_SYSTEM_GPS;
			break;
		case UBX_GNSS_ID_SBAS:
			*systems |= GNSS_SYSTEM_SBAS;
			break;
		case UBX_GNSS_ID_GALILEO:
			*systems |= GNSS_SYSTEM_GALILEO;
			break;
		case UBX_GNSS_ID_BEIDOU:
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

out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ubx_m10_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_SBAS | GNSS_SYSTEM_QZSS);
	return 0;
}

static struct gnss_driver_api gnss_api = {
	.set_navigation_mode = ubx_m10_set_navigation_mode,
	.get_navigation_mode = ubx_m10_get_navigation_mode,
	.set_enabled_systems = ubx_m10_set_enabled_systems,
	.get_enabled_systems = ubx_m10_get_enabled_systems,
	.get_supported_systems = ubx_m10_get_supported_systems,
};

static int ubx_m10_configure(const struct device *dev)
{
	int ret;

	(void) ubx_m10_configure_gnss_device_baudrate_prerequisite(dev);

	(void) ubx_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_STOP);
	k_sleep(K_MSEC(UBX_CFG_RST_WAIT_MS));

	ret = ubx_m10_configure_gnss_device_baudrate(dev);
	if (ret < 0) {
		LOG_ERR("Configuring baudrate failed. Returned %d.", ret);
		goto out;
	}

	ret = ubx_m10_configure_messages(dev);
	if (ret < 0) {
		LOG_ERR("Configuring messages failed. Returned %d.", ret);
		goto out;
	}

	// temp. need to remove the following.
	gnss_systems_t systems = 0;
	printk("ubx_m10_get_enabled_systems.\n");
	ubx_m10_get_enabled_systems(dev, &systems);
	systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_GLONASS
		| GNSS_SYSTEM_SBAS;
	printk("ubx_m10_set_enabled_systems.\n");
	ubx_m10_set_enabled_systems(dev, systems);
	systems = 0;
	printk("ubx_m10_get_enabled_systems.\n");
	ubx_m10_get_enabled_systems(dev, &systems);

	enum gnss_navigation_mode nav_mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
	ubx_m10_get_navigation_mode(dev, &nav_mode);
	printk("ubx_m10_get_navigation_mode. %d\n", nav_mode);
	nav_mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	ubx_m10_set_navigation_mode(dev, nav_mode);
	printk("ubx_m10_set_navigation_mode. %d\n", nav_mode);
	nav_mode = 0;
	ubx_m10_get_navigation_mode(dev, &nav_mode);
	printk("ubx_m10_get_navigation_mode. %d\n", nav_mode);

out:
	(void) ubx_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_START);

	return ret;
}

static int ubx_m10_init(const struct device *dev)
{
	int ret;

	ret = ubx_m10_init_nmea0183_match(dev);
	if (ret < 0) {
		return ret;
	}

	ubx_m10_init_pipe(dev);

	ret = ubx_m10_init_chat(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ubx_m10_init_ubx(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ubx_m10_resume(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ubx_m10_configure(dev);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(UBX_CFG_RST_WAIT_MS));
	// temp. need to remove the following.
	gnss_systems_t systems = 0;
	printk("ubx_m10_get_enabled_systems.\n");
	ubx_m10_get_enabled_systems(dev, &systems);
	systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_GLONASS
		| GNSS_SYSTEM_SBAS;
	printk("ubx_m10_set_enabled_systems.\n");
	ubx_m10_set_enabled_systems(dev, systems);
	systems = 0;
	printk("ubx_m10_get_enabled_systems.\n");
	ubx_m10_get_enabled_systems(dev, &systems);

	enum gnss_navigation_mode nav_mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
	ubx_m10_get_navigation_mode(dev, &nav_mode);
	printk("ubx_m10_get_navigation_mode. %d\n", nav_mode);
	nav_mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	ubx_m10_set_navigation_mode(dev, nav_mode);
	printk("ubx_m10_set_navigation_mode. %d\n", nav_mode);
	nav_mode = 0;
	ubx_m10_get_navigation_mode(dev, &nav_mode);
	printk("ubx_m10_get_navigation_mode. %d\n", nav_mode);

	return 0;
}

#define UBX_M10(inst)							\
	static struct ubx_m10_config ubx_m10_cfg_##inst = {		\
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
	};								\
									\
	static struct ubx_m10_data ubx_m10_data_##inst;			\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      ubx_m10_init,				\
			      NULL,					\
			      &ubx_m10_data_##inst,			\
			      &ubx_m10_cfg_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_GNSS_INIT_PRIORITY,		\
			      &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(UBX_M10)
