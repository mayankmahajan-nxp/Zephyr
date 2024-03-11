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

#include "vl53l4cd_api.h"
#include "vl53l4cd_platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_INITIAL_ADDR			0x29

struct vl53l4cd_data {
	struct k_sem lock;
	VL53L4CD_Dev_t vl53l4cd;
	// VL53L4CD_RangingMeasurementData_t RangingMeasurementData;
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
	// struct gpio_dt_spec xshut;
};

static int vl53l4cd_init(const struct device *dev)
{
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;

	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_dev_addr = VL53L4CD_INITIAL_ADDR;

{ // testing.
	uint8_t reg[2] = {0x01, 0x0F};
	i2c_write(config->i2c.bus, reg, 2, 0x29);
	uint8_t buf;
	i2c_read(config->i2c.bus, &buf, 1, 0x29);
	printk("%x\n", buf);

	uint8_t buf_2 = 0;
	VL53L4CD_RdByte(&(data->vl53l4cd), 0x0110, &buf_2);
	printk("%x\n", buf_2);
	uint16_t buf_3 = 0;
	VL53L4CD_RdWord(&(data->vl53l4cd), 0x0110, &buf_3);
	printk("%x\n", buf_3);
	uint32_t buf_4 = 0;
	VL53L4CD_RdDWord(&(data->vl53l4cd), 0x010F, &buf_4);
	printk("%x\n", buf_4);

	uint16_t id;
	VL53L4CD_GetSensorId(&(data->vl53l4cd), &id);
	printk("%x\n", id);

	uint8_t id_2;
	VL53L4CD_CheckForDataReady(&(data->vl53l4cd), &id_2);
	printk("%x\n", id_2);

	VL53L4CD_SensorInit(&(data->vl53l4cd));

	VL53L4CD_StartRanging(&(data->vl53l4cd));

	VL53L4CD_ResultsData_t p_result;
	while (true) {
		VL53L4CD_GetResult(&(data->vl53l4cd), &p_result);
		printk("p_result->distance_mm = %d\n", p_result.distance_mm);
		printk("p_result->range_status = %d\n", p_result.range_status);
		printk("p_result->signal_rate_kcps = %d\n", p_result.signal_rate_kcps);
		k_sleep(K_MSEC(250));
	}
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
		NULL							\
	);

DT_INST_FOREACH_STATUS_OKAY(VL53l4CD_INIT);
