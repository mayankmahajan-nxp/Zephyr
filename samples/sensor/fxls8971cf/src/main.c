/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/sensor/fxls8971cf.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(fxls8971cf, CONFIG_FXLS8971CF_LOG_LEVEL);

static const struct device *fxls8971cf_dev;
static const struct fxls8971cf_dev_api *fxls8971cf_api;

void main(void)
{
	LOG_INF("temp: execution started.");

	fxls8971cf_dev = DEVICE_DT_GET(DT_NODELABEL(fxls8971cf));
	if (!device_is_ready(fxls8971cf_dev)) {
		LOG_ERR("Device %s is not ready. Returning.", fxls8971cf_dev->name);
		return;
	}

	fxls8971cf_api = fxls8971cf_dev->api;

	int ret = 0;
	while (true) {
		ret |= fxls8971cf_api->read_data(fxls8971cf_dev);
		ret |= fxls8971cf_api->display_data(fxls8971cf_dev);

		if (ret) {
			LOG_ERR("Error %d while reading data. Returning.", ret);
			return;
		}

        k_sleep(K_MSEC(100));
	}

	LOG_INF("temp: execution completed.");
}
