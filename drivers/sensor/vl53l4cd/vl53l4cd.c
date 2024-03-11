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
#define T_BOOT				K_USEC(1200) /* VL53L4CD firmware boot period = 1.2 ms */

struct vl53l4cd_data {
	struct k_sem lock;
	VL53L4CD_Dev_t vl53l4cd;
	VL53L4CD_ResultsData_t result_data;
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
};

static int vl53l4cd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;

}

static int vl53l4cd_channel_get(const struct device *dev, enum sensor_channel chan,
			        struct sensor_value *val)
{
	int ret;

}

static const struct sensor_driver_api vl53l4cd_api = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
};

static int vl53l4cd_start(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;

	ret = VL53L4CD_StartRanging(&(data->vl53l4cd));

}

static int vl53l4cd_init(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;

	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_dev_addr = VL53L4CD_INITIAL_ADDR;

	k_sleep(T_BOOT);

	VL53L4CD_SensorInit(&(data->vl53l4cd));

	VL53L4CD_StartRanging(&(data->vl53l4cd));

	VL53L4CD_ResultsData_t p_result;
	while (true) {
		VL53L4CD_GetResult(&(data->vl53l4cd), &p_result);
		printk("p_result->distance_mm = %d\n", p_result.distance_mm);
		printk("p_result->range_status = %d\n", p_result.range_status);
		printk("p_result->signal_rate_kcps = %d\n", p_result.signal_rate_kcps);
		printk("\n");
		VL53L4CD_ClearInterrupt(&(data->vl53l4cd));
		k_sleep(K_MSEC(250));
	}

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
