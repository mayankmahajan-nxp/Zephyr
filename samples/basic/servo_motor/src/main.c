/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/drivers/sensor.h>
#include <stdio.h>

static struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo), max_pulse);

#define VL53L4CD_SAMPLE_MEASUREMENT_DELAY_MS	1000

#define STEP PWM_USEC(100)

enum direction {
	DOWN,
	UP,
};

int main(void)
{
	uint32_t pulse_width = min_pulse;
	enum direction dir = UP;
	int ret;

	printk("Servomotor control\n");

	if (!pwm_is_ready_dt(&servo)) {
		printk("Error: PWM device %s is not ready\n", servo.dev->name);
		return 0;
	}

	const struct device *const dev = DEVICE_DT_GET_ONE(st_vl53l4cd);
	struct sensor_value value;

	if (!device_is_ready(dev)) {
		printk("[%s] sensor: device not ready.\n", dev->name);
		return -ENODEV;
	}

	/* If CONFIG_VL53L4CD_RECONFIGURE_ADDRESS=y, sensor is started at the first transaction. */
#ifdef CONFIG_VL53L4CD_RECONFIGURE_ADDRESS
	ret = sensor_sample_fetch(dev);
	if (ret) {
		printk("[%s] sensor_channel_get failed. Returned %d.\n", dev->name, ret);
		return ret;
	}
#endif

	int i = 0;
	while (1) {
		servo.channel = 0;
		ret = pwm_set_pulse_dt(&servo, pulse_width);
		servo.channel = 3;
		ret = pwm_set_pulse_dt(&servo, pulse_width * 1.4);
		servo.channel = 4;
		ret = pwm_set_pulse_dt(&servo, pulse_width);
		servo.channel = 5;
		ret = pwm_set_pulse_dt(&servo, pulse_width * 1.2);
		if (i == 5) {
			break;
		}

		if (ret < 0) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		if (dir == DOWN) {
			if (pulse_width <= min_pulse) {
				dir = UP;
				pulse_width = min_pulse;
			} else {
				pulse_width -= STEP;
			}
		} else {
			pulse_width += STEP;

			if (pulse_width >= max_pulse) {
				dir = DOWN;
				pulse_width = max_pulse;
			}
		}
		++i;

		k_sleep(K_SECONDS(1));
	}

	k_sleep(K_SECONDS(4));

	bool kill = false;

	while (1) {
		/* If interrupt mode is used, then we don't need to call sensor_sample_fetch. */
#ifndef CONFIG_VL53L4CD_INTERRUPT_MODE
		ret = sensor_sample_fetch(dev);
		if (ret) {
			printk("[%s] sensor_sample_fetch failed. Returned %d.\n", dev->name, ret);
			return ret;
		}
#endif

		ret = sensor_channel_get(dev, SENSOR_CHAN_PROX, &value);
		if (ret) {
			printk("[%s] sensor_channel_get failed. Returned %d.\n", dev->name, ret);
			return ret;
		}
		printk("proximity = %d\n", value.val1);

		ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value);
		if (ret) {
			printk("[%s] sensor_channel_get failed. Returned %d.\n", dev->name, ret);
			return ret;
		}
		printf("distance = %.3f m\n", sensor_value_to_double(&value));

		double dist = sensor_value_to_double(&value);
		kill = (dist < 0.2) ? false : true;

		ret = (kill) ? pwm_set_pulse_dt(&servo, pulse_width) : pwm_set_pulse_dt(&servo, min_pulse);
	}

	return 0;
}
