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

#include "VL53L4CD_ULD/Platform/platform.h"
#include "VL53L4CD_ULD/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

struct vl53l4cd_data {
	struct k_sem lock;
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
};

static int vl53l4cd_init(const struct device *dev)
{
	const struct vl53l4cd_config *config = dev->config;

	uint8_t reg[2] = {0x01, 0x0F};
	i2c_write(config->i2c.bus, reg, 2, 0x29);
	uint8_t buf;
	i2c_read(config->i2c.bus, &buf, 1, 0x29);
	printk("%x\n", buf);

	uint8_t buf_2 = 0;
	VL53L4CD_RdByte(config->i2c.bus, 0x0110, &buf_2);
	printk("%x\n", buf_2);
	uint16_t buf_3 = 0;
	VL53L4CD_RdWord(config->i2c.bus, 0x0110, &buf_3);
	printk("%x\n", buf_3);
	uint32_t buf_4 = 0;
	VL53L4CD_RdDWord(config->i2c.bus, 0x010F, &buf_4);
	printk("%x\n", buf_4);

	const struct device *i2c = config->i2c.bus;

	uint16_t id;
	VL53L4CD_GetSensorId(i2c, &id);
	printk("%x\n", id);

	id = 0x9999;
	VL53L4CD_CheckForDataReady(i2c, &id);
	printk("%x\n", id);

	VL53L4CD_SensorInit(i2c);

	VL53L4CD_StartRanging(i2c);

	VL53L4CD_ResultsData_t p_result;
	while (true) {
		VL53L4CD_GetResult(i2c, &p_result);
		printk("p_result->distance_mm = %d\n", p_result.distance_mm);
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
		NULL							\
	);

DT_INST_FOREACH_STATUS_OKAY(VL53l4CD_INIT);
