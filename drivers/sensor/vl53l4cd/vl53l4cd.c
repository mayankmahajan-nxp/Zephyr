/*
 * Copyright 2024 NXP
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
#include <zephyr/drivers/gpio.h>

#include "vl53l4cd_api.h"
#include "platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_INITIAL_ADDR		0x29
#define VL53L4CD_BOOT_TIME_US		1200		/* VL53L4CD firmware boot period = 1.2 ms */
#define VL53L4CD_REG_WHO_AM_I		0x010F		/* Register address for Model_ID */
	/* Note: the register address for Module_Type = 0x0110.
	 * Hence, reading 16 bits at VL53L4CD_REG_WHO_AM_I gives both Model_ID and Module_Type.
	 */
#define VL53L4CD_MODEL_ID		0xEB
#define VL53L4CD_MODULE_TYPE		0xAA
#define VL53L4CD_CHIP_ID		(((uint16_t) VL53L4CD_MODEL_ID) << 8 | VL53L4CD_MODULE_TYPE)

#define VL53L4CD_XSHUT_ON		1
#define VL53L4CD_XSHUT_OFF		0

struct vl53l4cd_data {
	/* struct k_sem lock; */ /* TODO: implement spin lock for data structure. */
	bool started;
	struct VL53L4CD_Dev vl53l4cd;
	struct VL53L4CD_ResultsData result_data;

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	struct gpio_callback gpio_cb;
	struct k_work work;
	const struct device *dev;
#endif
};

struct vl53l4cd_config {
	const struct i2c_dt_spec i2c;
#ifdef CONFIG_VL53L4CD_XSHUT
	struct gpio_dt_spec xshut;
#endif
#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	struct gpio_dt_spec gpio1;
#endif
};

static int vl53l4cd_read_sensor(struct vl53l4cd_data *data)
{
	enum VL53L4CD_Error ret;

	ret = VL53L4CD_GetResult(&data->vl53l4cd, &data->result_data);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Error while getting data from sensor. Returned %d.", ret);
		return -EIO;
	}

	ret = VL53L4CD_ClearInterrupt(&data->vl53l4cd);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Error clearing interrupt on the sensor. Returned %d.", ret);
		return -EBUSY;
	}

	return 0;
}

static int vl53l4cd_start(const struct device *dev)
{
	enum VL53L4CD_Error ret;
	struct vl53l4cd_data *data = dev->data;

	ret = VL53L4CD_StartRanging(&(data->vl53l4cd));
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("[%s] Error while starting sensor. Returned %d.", dev->name, ret);
		return -EBUSY;
	}

	data->started = true;

	return 0;
}

static int vl53l4cd_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	uint8_t p_is_data_ready;

	__ASSERT_NO_MSG((chan == SENSOR_CHAN_DISTANCE) || (chan == SENSOR_CHAN_PROX));

	if (data->started == false) {
		ret = vl53l4cd_start(dev);
		if (ret < 0) {
			return ret;
		}
	}

	ret = VL53L4CD_CheckForDataReady(&data->vl53l4cd, &p_is_data_ready);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("Error checking for data ready from sensor. Returned %d.", ret);
		return -EIO;
	}
	if (p_is_data_ready == VL53L4CD_DATA_NOT_READY) {
		LOG_ERR("[%s] Data is not ready. Returned %d.", dev->name, ret);
		return -ENODATA;
	}

	ret = vl53l4cd_read_sensor(data);
	if (ret < 0) {
		LOG_ERR("[%s] Could not perform measurement; error = %d.", dev->name, ret);
		return ret;
	}

	return 0;
}

static int vl53l4cd_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct vl53l4cd_data *data = dev->data;

	if (chan == SENSOR_CHAN_PROX) {
		if (data->result_data.distance_mm < CONFIG_VL53L4CD_PROXIMITY_THRESHOLD) {
			val->val1 = 1;
		} else {
			val->val1 = 0;
		}

		val->val2 = 0;
	} else if (chan == SENSOR_CHAN_DISTANCE) {
		val->val1 = data->result_data.distance_mm / 1000;
		val->val2 = (data->result_data.distance_mm % 1000) * 1000;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

/* TODO: check if we need to implement other apis. */
static const struct sensor_driver_api vl53l4cd_api = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
};

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
static void vl53l4cd_worker(struct k_work *work)
{
	struct vl53l4cd_data *data = CONTAINER_OF(work, struct vl53l4cd_data, work);
	int ret;

	ret = vl53l4cd_read_sensor(data);
	if (ret < 0) {
		LOG_ERR("Reading data from sensor failed. Returned %d.\n", ret);
	}
}

static void vl53l4cd_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	struct vl53l4cd_data *data = CONTAINER_OF(cb, struct vl53l4cd_data, gpio_cb);

	(void) k_work_submit(&data->work);
}

static int vl53l4cd_init_interrupt(const struct device *dev)
{
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;
	int ret;

	data->dev = dev; /* TODO: what is the role of this statement. */

	if (!gpio_is_ready_dt(&config->gpio1)) {
		LOG_ERR("[%s] GPIO %s is not ready.", dev->name, config->gpio1.port->name);
		return -ENODEV;
	}

	/* TODO: what is the use of GPIO_PULL_UP. */
	ret = gpio_pin_configure_dt(&config->gpio1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to configure GPIO as INPUT. Returned %d.", dev->name, ret);
		return -EIO;
	}

	gpio_init_callback(&data->gpio_cb, vl53l4cd_gpio_callback, BIT(config->gpio1.pin));

	ret = gpio_add_callback(config->gpio1.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to set GPIO callback. Returned %d.", dev->name, ret);
		return -EIO;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->gpio1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to config GPIO interrupt. Returned %d.", dev->name, ret);
		return -EIO;
	}

	data->work.handler = vl53l4cd_worker;

	return 0;
}
#endif

/* TODO: the following two functions haven't been tested. */
#ifdef CONFIG_VL53L4CD_XSHUT
static int vl53l4cd_sensor_power_on(const struct device *dev)
{
	const struct vl53l4cd_config *const config = dev->config;
	int ret;

	if (config->xshut.port) {
		ret = gpio_pin_set_dt(&config->xshut, VL53L4CD_XSHUT_ON);
		if (ret < 0) {
			LOG_ERR("[%s] Unable to set XSHUT GPIO. Returned %d.", dev->name, ret);
			return -EIO;
		}

		k_sleep(K_USEC(VL53L4CD_BOOT_TIME_US)); /* Wait for device to boot up. */
	}

	return 0;
}

static int vl53l4cd_sensor_power_off(const struct device *dev)
{
	const struct vl53l4cd_config *const config = dev->config;
	int ret;

	if (config->xshut.port) {
		ret = gpio_pin_set_dt(&config->xshut, VL53L4CD_XSHUT_OFF);
		if (ret < 0) {
			LOG_ERR("[%s] Unable to set XSHUT GPIO. Returned %d.", dev->name, ret);
			return -EIO;
		}
	}

	return 0;
}
#endif

static int vl53l4cd_init(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;
	uint16_t vl53l4cd_id = 0;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("[%s] I2C bus is not ready. Exiting.", dev->name);
		return -ENODEV;
	}

	k_sleep(K_USEC(VL53L4CD_BOOT_TIME_US)); /* Wait for device to boot up. */

	/* Initialize device's data structure (struct vl53l4cd_data). */
	data->started = false;
	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_dev_addr = VL53L4CD_INITIAL_ADDR;

#ifdef CONFIG_VL53L4CD_XSHUT
	if (config->xshut.port) {
		ret = gpio_pin_configure_dt(&config->xshut, GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("[%s] Unable to configure GPIO as output. Returned %d.", dev->name,
				ret);
			return -EIO;
		}
	}
#endif

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	if (config->gpio1.port) {
		ret = vl53l4cd_init_interrupt(dev);
		if (ret < 0) {
			LOG_ERR("[%s] Failed to initialize GPIO interrupt.", dev->name);
			return -EIO;
		}
	}
#endif

	/* Validate Chip ID (Model_ID and Module_Type). */
	ret = VL53L4CD_RdWord(&(data->vl53l4cd), VL53L4CD_REG_WHO_AM_I, &vl53l4cd_id);
	if ((ret != VL53L4CD_ERROR_NONE) || (vl53l4cd_id != VL53L4CD_CHIP_ID)) {
		LOG_ERR("[%s] Issue with device identification. Returned = %d.", dev->name, ret);
		return -ENOTSUP;
	}

	ret = VL53L4CD_SensorInit(&(data->vl53l4cd));
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_DBG("[%s] Failed to initialize the sensor.", dev->name);
		return -EBUSY;
	}

	ret = vl53l4cd_start(dev);
	if (ret < 0) {
		LOG_DBG("[%s] Failed to start the sensor.", dev->name);
		return -EBUSY;
	}

	LOG_DBG("[%s] Initialized successfully.", dev->name);

	return 0;
}

#define VL53L4CD_INIT(inst)								\
	struct vl53l4cd_data vl53l4cd_data_##inst = {					\
	};										\
	struct vl53l4cd_config vl53l4cd_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		IF_ENABLED(CONFIG_VL53L4CD_XSHUT, (					\
			.xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),	\
			)								\
		)									\
		IF_ENABLED(CONFIG_VL53L4CD_INTERRUPT_MODE, (				\
			.gpio1 = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),	\
			)								\
		)									\
	};										\
	DEVICE_DT_INST_DEFINE(								\
		inst,									\
		vl53l4cd_init,								\
		NULL,									\
		&vl53l4cd_data_##inst,							\
		&vl53l4cd_config_##inst,						\
		POST_KERNEL,								\
		CONFIG_SENSOR_INIT_PRIORITY,						\
		&vl53l4cd_api								\
	);

DT_INST_FOREACH_STATUS_OKAY(VL53L4CD_INIT);
