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

#include "gnss_u_blox_protocol/gnss_u_blox_protocol.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define DT_DRV_COMPAT u_blox_m10

#define UART_RECV_BUF_SZ	128
#define UART_TRNF_BUF_SZ	128

#define CHAT_RECV_BUF_SZ	256
#define CHAT_ARGV_SZ		32

#define UBX_RECV_BUF_SZ		U_BLOX_FRM_SZ_MAX
#define UBX_WORK_BUF_SZ		U_BLOX_FRM_SZ_MAX
#define UBX_RESPONSE_BUF_SZ	U_BLOX_FRM_SZ_MAX

#define MODEM_UBX_FRM_TIMEOUT_MS	500

#define U_BLOX_M10_GNSS_SYS_CNT_MAX	6

#define U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE_TEMP(script_name, ubx_frame, ubx_frame_len, retry)	\
	struct modem_ubx_script script_name = {							\
		.ubx_frame = ubx_frame,								\
		.ubx_frame_size = ubx_frame_len,						\
		.retry_count = retry,								\
	};

#define U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frm, ubx_frm_sz, retry)		\
	uint8_t ubx_frm[ubx_frm_sz];								\
	struct modem_ubx_script script_inst = {							\
		.ubx_frame = ubx_frm,								\
		.retry_count = retry,								\
	};

#define U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script_inst, ubx_frm, ubx_frm_sz, retry, msg_class,	\
					 msg_id, data, payload_size)				\
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script_inst, ubx_frm, ubx_frm_sz, retry)		\
	script_inst.ubx_frame_size = u_blox_create_frame(script_inst.ubx_frame, ubx_frm_sz,	\
							 msg_class, msg_id, data, payload_size);

#define U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script_inst, script_ret)		\
	int script_ret;								\
	script_ret = u_blox_m10_modem_ubx_run_script(dev, &script_inst);	\
	if (script_ret < 0) {							\
		return script_ret;						\
	}

enum MODEM_MODULE {
	MODEM_MODULE_CHAT = 0,
	MODEM_MODULE_UBX,
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
	uint8_t ubx_response_buf[UBX_RESPONSE_BUF_SZ];

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
		.ubx_response_buf = data->ubx_response_buf,
		.ubx_response_buf_size = sizeof(data->ubx_response_buf),
		.process_timeout = K_MSEC(MODEM_UBX_FRM_TIMEOUT_MS),
	};

	return modem_ubx_init(&data->ubx, &ubx_config);
}

static int u_blox_m10_modem_module_switch(const struct device *dev,
					  enum MODEM_MODULE release,
					  enum MODEM_MODULE attach) {
	struct u_blox_m10_data *data = dev->data;
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

static int u_blox_m10_modem_ubx_run_script(const struct device *dev,
					    struct modem_ubx_script *modem_ubx_script_tx)
{
	struct u_blox_m10_data *data = dev->data;
	int ret;

	ret = u_blox_m10_modem_module_switch(dev, MODEM_MODULE_CHAT, MODEM_MODULE_UBX);
	if (ret < 0) {
		goto out;
	}

	ret = modem_ubx_run_script(&data->ubx, modem_ubx_script_tx);
	if (ret < 0) {
		goto out;
	}

out:
	ret |= u_blox_m10_modem_module_switch(dev, MODEM_MODULE_UBX, MODEM_MODULE_CHAT);
	if (ret < 0) {
		LOG_ERR("script failed %d.", ret);
		return ret;
	}

	return 0;
}

static int u_blox_m10_ubx_cfg_prt_set(const struct device *dev, uint32_t target_baudrate,
				      uint16_t retry)
{
	U_BLOX_CFG_PRT_SET_DATA_INIT(data)
	data.baudrate = target_baudrate;
	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_PRT_SET_FRM_SZ, retry,
		UBX_CLASS_CFG, UBX_CFG_PRT, &data, UBX_CFG_PRT_SET_PAYLOAD_SZ)
	U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)

	return 0;
}

static int u_blox_m10_ubx_cfg_rst(const struct device *dev, uint8_t reset_mode)
{
	int retry = 2;
	U_BLOX_CFG_RST_DATA_INIT(data)
	data.nav_bbr_mask = UBX_CFG_RST_NAV_BBR_MASK_HOT_START;
	data.reset_mode = reset_mode;
	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_RST_FRM_SZ, retry,
		UBX_CLASS_CFG, UBX_CFG_RST, &data, UBX_CFG_RST_PAYLOAD_SZ)
	U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)

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

static int u_blox_m10_configure_gnss_device_baudrate_prerequisite(const struct device *dev)
{
	int ret, retry_count = 2;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	/* Try communication with device with all possible baudrates. */
	for (int i = 0; i < U_BLOX_BAUDRATE_COUNT; ++i) {
		/* Set baudrate of UART pipe as u_blox_baudrate[i]. */
		ret = u_blox_m10_set_uart_baudrate(dev, u_blox_baudrate[i]);
		if (ret < 0) {
			return ret;
		}

		/* Try setting baudrate of device as target_baudrate. */
		ret = u_blox_m10_ubx_cfg_prt_set(dev, target_baudrate, retry_count);
		if (ret == 0) {
			break;
		}
	}

	/* Reset baudrate of UART pipe as target_baudrate. */
	ret = u_blox_m10_set_uart_baudrate(dev, target_baudrate);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int u_blox_m10_configure_gnss_device_baudrate(const struct device *dev)
{
	int ret;

	int target_baudrate = u_blox_m10_get_uart_baudrate(dev);

	ret = u_blox_m10_ubx_cfg_prt_set(dev, target_baudrate, MODEM_UBX_RETRY_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int u_blox_m10_configure_messages(const struct device *dev)
{
	U_BLOX_CFG_MSG_DATA_INIT(data)
	U_BLOX_M10_MODEM_UBX_SCRIPT_CREATE(script, ubx_frame, UBX_CFG_MSG_FRM_SZ,
		MODEM_UBX_RETRY_DEFAULT)

	/* Enabling GGA, RMC and GSV messages. */
	data.rate = 1;
	uint8_t message_enable[] = {UBX_NMEA_GGA, UBX_NMEA_RMC, UBX_NMEA_GSV};

	for (int i = 0; i < sizeof(message_enable); ++i) {
		data.message_id = message_enable[i];
		script.ubx_frame_size = u_blox_create_frame(script.ubx_frame, UBX_CFG_MSG_FRM_SZ,
			UBX_CLASS_CFG, UBX_CFG_MSG, &data, UBX_CFG_MSG_PAYLOAD_SZ);
		U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)
	}

	/* Disabling DTM, GBS, GLL, GNS, GRS, GSA, GST, VLW, VTG and ZDA messages. */
	data.rate = 0;
	uint8_t message_disable[] = {UBX_NMEA_DTM, UBX_NMEA_GBS, UBX_NMEA_GLL, UBX_NMEA_GNS,
				     UBX_NMEA_GRS, UBX_NMEA_GSA, UBX_NMEA_GST, UBX_NMEA_VLW,
				     UBX_NMEA_VTG, UBX_NMEA_ZDA};

	for (int i = 0; i < sizeof(message_disable); ++i) {
		data.message_id = message_disable[i];
		script.ubx_frame_size = u_blox_create_frame(script.ubx_frame, UBX_CFG_MSG_FRM_SZ,
			UBX_CLASS_CFG, UBX_CFG_MSG, &data, UBX_CFG_MSG_PAYLOAD_SZ);
		U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)
	}

	return 0;
}

static int u_blox_m10_set_navigation_mode(const struct device *dev, enum gnss_navigation_mode mode)
{
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

	U_BLOX_CFG_NAV5_DATA_INIT(data)
	data.dyn_model = dynamic_model;

	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_NAV5_FRM_SZ,
		MODEM_UBX_RETRY_DEFAULT, UBX_CLASS_CFG, UBX_CFG_NAV5, &data,
		UBX_CFG_NAV5_PAYLOAD_SZ)
	U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)

	return 0;
}

static int u_blox_m10_get_navigation_mode(const struct device *dev, enum gnss_navigation_mode *mode)
{
	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_NAV5_FRM_SZ,
		MODEM_UBX_RETRY_DEFAULT, UBX_CLASS_CFG, UBX_CFG_NAV5, NULL, UBX_FRM_GET_PAYLOAD_SZ)
	U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)

	struct ubx_frame_t *frame = (struct ubx_frame_t *) ubx_frame;

	switch (((struct u_blox_cfg_nav5_data *)frame->payload_and_checksum)->dyn_model) {
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

	return 0;
}

static int u_blox_m10_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	struct u_blox_cfg_gnss_data *data;

	uint8_t cfg_blocks_count = 0;
	gnss_systems_t temp = systems;
	while (temp > 0) {
	        ++cfg_blocks_count;
		temp = temp & (temp - 1);
	}

	data = malloc(sizeof(*data) + sizeof(struct u_blox_cfg_gnss_data_config_block) * cfg_blocks_count);
	data->num_config_blocks = cfg_blocks_count;

	(void) u_blox_cfg_gnss_data_default(data);

	uint8_t filled_blocks = 0;
	for (int i = 0; i < 8; ++i) {
		if (systems & (1 << i)) {
			switch (systems & (1 << i)) {
			case GNSS_SYSTEM_GPS:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_GPS;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_GPS_L1C_A;
				break;
			case GNSS_SYSTEM_GLONASS:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_GLONAS;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_GLONASS_L1;
				break;
			case GNSS_SYSTEM_GALILEO:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_GALILEO;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_Galileo_E1;
				break;
			case GNSS_SYSTEM_BEIDOU:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_BEIDOU;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_BeiDou_B1I;
				break;
			case GNSS_SYSTEM_QZSS:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_QZSS;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_QZSS_L1C_A;
				break;
			case GNSS_SYSTEM_SBAS:
				data->config_blocks[filled_blocks].gnssId = UBX_GNSS_ID_SBAS;
				data->config_blocks[filled_blocks].flags = U_BLOX_CFG_GNSS_CNF_BLK_FLAG_ENABLE |
					U_BLOX_CFG_GNSS_CNF_BLK_FLAG_SGN_CNF_MASK_SBAS_L1C_A;
				break;
			default:
				break;
			};
			++filled_blocks;
		}
	}


	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_GNSS_FRM_SZ(cfg_blocks_count),
					 MODEM_UBX_RETRY_DEFAULT, UBX_CLASS_CFG, UBX_CFG_GNSS,
					 data, UBX_CFG_GNSS_PAYLOAD_SZ(cfg_blocks_count))
	int ret;
	ret = u_blox_m10_modem_ubx_run_script(dev, &script);
	if (ret < 0) {
		goto out;
	}

out:
	free(data);

	return ret;
}

static int u_blox_m10_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	U_BLOX_M10_MODEM_UBX_SCRIPT_INIT(script, ubx_frame, UBX_CFG_GNSS_FRM_SZ(U_BLOX_M10_GNSS_SYS_CNT_MAX),
		MODEM_UBX_RETRY_DEFAULT, UBX_CLASS_CFG, UBX_CFG_GNSS, NULL, UBX_FRM_GET_PAYLOAD_SZ)
	U_BLOX_M10_MODEM_UBX_SCRIPT_RUN(dev, script, script_ret)

	for (int i = 10; i < script.ubx_frame_size - U_BLOX_CHECKSUM_STOP_IDX_FROM_END;
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

	return 0;
}

static int u_blox_m10_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	*systems = (GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
		    GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_SBAS | GNSS_SYSTEM_QZSS);
	return 0;
}

static struct gnss_driver_api gnss_api = {
	.set_navigation_mode = u_blox_m10_set_navigation_mode,
	.get_navigation_mode = u_blox_m10_get_navigation_mode,
	.set_enabled_systems = u_blox_m10_set_enabled_systems,
	.get_enabled_systems = u_blox_m10_get_enabled_systems,
	.get_supported_systems = u_blox_m10_get_supported_systems,
};

static int u_blox_m10_configure(const struct device *dev)
{
	int ret;

	(void) u_blox_m10_configure_gnss_device_baudrate_prerequisite(dev);

	(void) u_blox_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_STOP);
	k_sleep(K_MSEC(U_BLOX_CFG_RST_WAIT_MS));

	ret = u_blox_m10_configure_gnss_device_baudrate(dev);
	if (ret < 0) {
		LOG_ERR("configuring baudrate failed %d. exiting.", ret);
		goto out;
	}

	ret = u_blox_m10_configure_messages(dev);
	if (ret < 0) {
		LOG_ERR("configuring messages failed %d. exiting.", ret);
		goto out;
	}

	// temp. need to remove the following.
	gnss_systems_t systems = 0;
	printk("u_blox_m10_get_enabled_systems.\n");
	u_blox_m10_get_enabled_systems(dev, &systems);
	systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_GLONASS
		| GNSS_SYSTEM_SBAS;
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

out:
	(void) u_blox_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_START);

	if (ret < 0) {
		return ret;
	}

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

	ret = u_blox_m10_configure(dev);
	if (ret < 0) {
		return ret;
	}

	// (void) u_blox_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_STOP);
	k_sleep(K_MSEC(U_BLOX_CFG_RST_WAIT_MS));
	// temp. need to remove the following.
	gnss_systems_t systems = 0;
	printk("u_blox_m10_get_enabled_systems.\n");
	u_blox_m10_get_enabled_systems(dev, &systems);
	systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_QZSS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_GLONASS
		| GNSS_SYSTEM_SBAS;
	printk("u_blox_m10_set_enabled_systems.\n");
	u_blox_m10_set_enabled_systems(dev, systems);
	systems = 0;
	printk("u_blox_m10_get_enabled_systems.\n");
	u_blox_m10_get_enabled_systems(dev, &systems);

	enum gnss_navigation_mode nav_mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
	printk("navigation starting. %d\n", nav_mode);
	u_blox_m10_get_navigation_mode(dev, &nav_mode);
	printk("u_blox_m10_get_navigation_mode. %d\n", nav_mode);
	nav_mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
	u_blox_m10_set_navigation_mode(dev, nav_mode);
	printk("u_blox_m10_set_navigation_mode. %d\n", nav_mode);
	nav_mode = 0;
	u_blox_m10_get_navigation_mode(dev, &nav_mode);
	printk("u_blox_m10_get_navigation_mode. %d\n", nav_mode);
	// (void) u_blox_m10_ubx_cfg_rst(dev, UBX_CFG_RST_RESET_MODE_CONTROLLED_GNSS_START);

	return 0;
}

#define U_BLOX_M10(inst)						\
	static struct u_blox_m10_config u_blox_m10_cfg_##inst = {	\
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
	};								\
									\
	static struct u_blox_m10_data u_blox_m10_data_##inst;		\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      u_blox_m10_init,				\
			      NULL,					\
			      &u_blox_m10_data_##inst,			\
			      &u_blox_m10_cfg_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_GNSS_INIT_PRIORITY,		\
			      &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(U_BLOX_M10)
