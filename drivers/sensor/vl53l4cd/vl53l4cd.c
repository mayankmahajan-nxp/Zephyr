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
#include <zephyr/drivers/gpio.h>

#include "VL53L4CD_api.h"
#include "platform.h"

LOG_MODULE_REGISTER(VL53L4CD, CONFIG_SENSOR_LOG_LEVEL);

#define VL53L4CD_INITIAL_ADDR		0x29
#define VL53L4CD_BOOT_TIME		K_USEC(1200) /* VL53L4CD firmware boot period = 1.2 ms */
#define VL53L4CD_REG_WHO_AM_I		0x010F
#define VL53L4CD_CHIP_ID		0xEBAA

struct vl53l4cd_data {
	// struct k_sem lock; /* TODO: implement spin lock for data. */
	bool started;
	VL53L4CD_Dev_t vl53l4cd;
	VL53L4CD_ResultsData_t result_data;

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

static int vl53l4cd_start(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;

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
		ret = -EINVAL;
		goto clear_interrupt;
	}

clear_interrupt:
	VL53L4CD_ClearInterrupt(&(data->vl53l4cd));

	return ret;
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

static const struct sensor_driver_api vl53l4cd_api = {
	.sample_fetch = vl53l4cd_sample_fetch,
	.channel_get = vl53l4cd_channel_get,
};

static VL53L4CD_Error_t vl53l4cd_read_sensor(struct vl53l4cd_data *data)
{
	VL53L4CD_Error_t ret;

	ret = VL53L4CD_GetResult(&data->vl53l4cd, &data->result_data);
	// ret = VL53L4CD_GetRangingMeasurementData(&data->vl53l4cd, &data->data);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("VL53L1_GetRangingMeasurementData return error (%d)", ret);
		return ret;
	}

	ret = VL53L4CD_ClearInterrupt(&data->vl53l4cd);
	// ret = VL53L4CD_ClearInterruptAndStartMeasurement(&data->vl53l4cd);
	if (ret != VL53L4CD_ERROR_NONE) {
		LOG_ERR("VL53L1_ClearInterruptAndStartMeasurement return error (%d)", ret);
		return ret;
	}

	return VL53L4CD_ERROR_NONE;
}

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
static void vl53l4cd_worker(struct k_work *work)
{
	struct vl53l4cd_data *data = CONTAINER_OF(work, struct vl53l4cd_data, work);

	vl53l4cd_read_sensor(data);
}

static void vl53l4cd_gpio_callback(const struct device *dev,
		struct gpio_callback *cb, uint32_t pins)
{
	LOG_ERR("callback arrived.");
	struct vl53l4cd_data *data = CONTAINER_OF(cb, struct vl53l4cd_data, gpio_cb);

	k_work_submit(&data->work);
}

static int vl53l4cd_init_interrupt(const struct device *dev)
{
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;
	int ret;

	data->dev = dev;

	if (!gpio_is_ready_dt(&config->gpio1)) {
		LOG_ERR("%s: device %s is not ready", dev->name, config->gpio1.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->gpio1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to configure GPIO interrupt", dev->name);
		return -EIO;
	}

	gpio_init_callback(&data->gpio_cb, vl53l4cd_gpio_callback, BIT(config->gpio1.pin));

	ret = gpio_add_callback(config->gpio1.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}

	data->work.handler = vl53l4cd_worker;

	return 0;
}
#endif

static int vl53l4cd_init(const struct device *dev)
{
	int ret;
	struct vl53l4cd_data *data = dev->data;
	const struct vl53l4cd_config *config = dev->config;
	uint16_t vl53l4cd_id = 0;

	k_sleep(VL53L4CD_BOOT_TIME);

	data->started = false;
	data->vl53l4cd.i2c_bus = config->i2c.bus;
	data->vl53l4cd.i2c_dev_addr = VL53L4CD_INITIAL_ADDR;

	ret = VL53L4CD_RdWord(&(data->vl53l4cd), VL53L4CD_REG_WHO_AM_I, &vl53l4cd_id);
	if ((ret < 0) || (vl53l4cd_id != VL53L4CD_CHIP_ID)) {
		LOG_ERR("[%s] issue with device identification.", dev->name);
		return -ENOTSUP;
	}

	ret = VL53L4CD_SensorInit(&(data->vl53l4cd));
	if (ret < 0) {
		return ret;
	}

	ret = vl53l4cd_start(dev);
	if (ret < 0) {
		return ret;
	}

	/* Configure gpio connected to VL53L4CD's XSHUT pin to
	 * allow deepest sleep mode
	 */
#ifdef CONFIG_VL53L4CD_XSHUT
		if (config->xshut.port) {
			ret = gpio_pin_configure_dt(&config->xshut, GPIO_OUTPUT);
			if (ret < 0) {
				LOG_ERR("[%s] Unable to configure GPIO as output", dev->name);
				return -EIO;
			}
		}
#endif

printk("start\n");
#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	if (config->gpio1.port) {
		printk("config->gpio1.\n");
		ret = vl53l4cd_init_interrupt(dev);
		if (ret < 0) {
			LOG_ERR("Failed to initialize interrupt!");
			return -EIO;
		}
		printk("error free.\n");
	}
#endif

#ifdef CONFIG_VL53L4CD_INTERRUPT_MODE
	ret = gpio_pin_interrupt_configure_dt(&config->gpio1, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret < 0) {
		LOG_ERR("[%s] Unable to config interrupt", dev->name);
		return -EIO;
	}
#endif

	/* Pull XSHUT high to start the sensor */
#ifdef CONFIG_VL53L4CD_XSHUT
	// const struct vl53l1x_config *const config = dev->config;

	if (config->xshut.port) {
		int gpio_ret = gpio_pin_set_dt(&config->xshut, 1);

		if (gpio_ret < 0) {
			LOG_ERR("[%s] Unable to set XSHUT gpio (error %d)", dev->name, gpio_ret);
			return -EIO;
		}
		/* Boot duration is 1.2 ms max */
		k_sleep(K_MSEC(2));
	}
#endif

	LOG_DBG("[%s] initialized successfully.", dev->name);

	return 0;
}

#define VL53L4CD_INIT(inst)								\
	struct vl53l4cd_data vl53l4cd_data_##inst = {					\
	};										\
	struct vl53l4cd_config vl53l4cd_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		IF_ENABLED(CONFIG_VL53L4CD_XSHUT,					\
			(.xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),))	\
		IF_ENABLED(CONFIG_VL53L4CD_INTERRUPT_MODE,				\
			(.gpio1 = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),))	\
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
