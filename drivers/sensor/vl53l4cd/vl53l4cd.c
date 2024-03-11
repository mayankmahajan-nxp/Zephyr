/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l4cd

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include "VL53L4CD_api.h"
#include "platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_INITIAL_ADDR		0x29
#define VL53L4CD_BOOT_TIME		K_USEC(1200) /* VL53L4CD firmware boot period = 1.2 ms */
#define VL53L4CD_REG_WHO_AM_I		0x010F
#define VL53L4CD_CHIP_ID		0xEBAA

struct vl53l4cd_data {
	struct k_sem lock;
	bool started;
	VL53L4CD_Dev_t vl53l4cd;
	VL53L4CD_ResultsData_t result_data;
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
};

static int vl53l4cd_start(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	uint16_t vl53l4cd_id = 0;

	ret = VL53L4CD_RdWord(&(data->vl53l4cd), VL53L4CD_REG_WHO_AM_I, &vl53l4cd_id);
	if ((ret < 0) || (vl53l4cd_id != VL53L4CD_CHIP_ID)) {
		LOG_ERR("[%s] issue with device identification.", dev->name);
		return -ENOTSUP;
	}

	ret = VL53L4CD_StartRanging(&(data->vl53l4cd));
	if (ret < 0) {
		return ret;
	}

	data->started = true;

	return 0;
}

static int vl53l4cd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;

	__ASSERT_NO_MSG((chan == SENSOR_CHAN_DISTANCE) || (chan == SENSOR_CHAN_PROX));

	if (data->started == false) {
		ret = vl53l4cd_start(dev);
		if (ret < 0) {
			return ret;
		}
	}

	ret = VL53L4CD_GetResult(&(data->vl53l4cd), &(data->result_data));
	if (ret < 0) {
		LOG_ERR("[%s] could not perform measurement; error = %d.", dev->name, ret);
		return -EINVAL;
	}

	return 0;
}

static int vl53l4cd_channel_get(const struct device *dev, enum sensor_channel chan,
			        struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;

	if (chan == SENSOR_CHAN_PROX) {

	} else if (chan == SENSOR_CHAN_DISTANCE) {
		val->val1 = data->result_data.distance_mm / 1000;
		val->val2 = (data->result_data.distance_mm % 1000) * 1000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api vl53l4cd_api = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
};

static int vl53l4cd_init(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;

	k_sleep(VL53L4CD_BOOT_TIME);

	data->started = false;
	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_dev_addr = VL53L4CD_INITIAL_ADDR;

	ret = VL53L4CD_SensorInit(&(data->vl53l4cd));
	if (ret < 0) {
		return ret;
	}

	ret = vl53l4cd_start(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("[%s] initialized successfully.", dev->name);

	return 0;
}

#define VL53l4CD_INIT(inst)						\
	struct vl53l4cd_data vl53l4cd_data_##inst = {			\
	};								\
	struct vl53l4cd_config vl53l4cd_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
	};								\
	DEVICE_DT_INST_DEFINE(						\
		inst,							\
		vl53l4cd_init,						\
		NULL,							\
		&vl53l4cd_data_##inst,					\
		&vl53l4cd_config_##inst,				\
		POST_KERNEL,						\
		CONFIG_SENSOR_INIT_PRIORITY,				\
		&vl53l4cd_api							\
	);

DT_INST_FOREACH_STATUS_OKAY(VL53l4CD_INIT);
