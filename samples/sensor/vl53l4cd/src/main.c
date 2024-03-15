/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#define VL53L4CD_SAMPLE_MEASUREMENT_DELAY_MS	1000

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(st_vl53l4cd);
	struct sensor_value value;
	int ret;

	if (!device_is_ready(dev)) {
		printk("[%s] sensor: device not ready.\n", dev->name);
		return -ENODEV;
	}

	while (1) {
		/* If interrupt mode is not used, then call sensor_sample_fetch for polling mode.*/
#ifndef CONFIG_VL53L4CD_INTERRUPT_MODE
		ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("[%s] sensor_sample_fetch failed. returned %d.\n", dev->name, ret);
			return ret;
		}
#endif

		ret = sensor_channel_get(dev, SENSOR_CHAN_PROX, &value);
		if (ret) {
			printk("[%s] sensor_channel_get failed. returned %d.\n", dev->name, ret);
			return ret;
		}
		printk("proximity = %d\n", value.val1);

		ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		if (ret) {
			printk("[%s] sensor_channel_get failed. returned %d.\n", dev->name, ret);
			return ret;
		}
		printf("distance = %.3f m\n", sensor_value_to_double(&value));

		k_sleep(K_MSEC(VL53L4CD_SAMPLE_MEASUREMENT_DELAY_MS));
	}

	return 0;
}
