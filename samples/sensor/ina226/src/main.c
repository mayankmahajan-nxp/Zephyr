/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Referred code from file: "zephyr/samples/sensor/ina219/src/main.c". */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>


int main(void)
{
	const struct device *const ina = DEVICE_DT_GET(DT_NODELABEL(ina226));
	struct sensor_value v_bus, v_shunt, power, current;
	int rc;

	if (!device_is_ready(ina)) {
		printf("Device %s is not ready.\n", ina->name);
		return 0;
	}

	while (true) {
		rc = sensor_sample_fetch(ina);
		if (rc) {
			printf("Could not fetch sensor data.\n");
			return 0;
		}

		sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &v_bus);
		sensor_channel_get(ina, SENSOR_CHAN_POWER, &power);
		sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current);
		sensor_channel_get(ina, SENSOR_CHAN_VSHUNT, &v_shunt);

		printf("Bus Voltage: %f [V] , "
			"Shunt Voltage: %f [V] , "
			"Power: %f [W] , "
			"Current: %f [A]\n",
			sensor_value_to_double(&v_bus),
			sensor_value_to_double(&v_shunt),
			sensor_value_to_double(&power),
			sensor_value_to_double(&current));

		k_sleep(K_MSEC(2000));
	}

	return 0;
}
