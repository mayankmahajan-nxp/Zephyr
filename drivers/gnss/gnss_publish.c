/*
 * Copyright (c) 2023 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/iterable_sections.h>

static struct k_spinlock lock;

#include "gnss_dump.h"
#include <zephyr/logging/log.h>
static char dump_buf[512];
static void gnss_dump_data_to_log(const struct device *dev, const struct gnss_data *data)
{
	if (gnss_dump_info(dump_buf, sizeof(dump_buf), &data->info) < 0) {
		return;
	}

	LOG_PRINTK("%s: %s\r\n", dev->name, dump_buf);

	if (gnss_dump_nav_data(dump_buf, sizeof(dump_buf), &data->nav_data) < 0) {
		return;
	}

	LOG_PRINTK("%s: %s\r\n", dev->name, dump_buf);

	if (gnss_dump_time(dump_buf, sizeof(dump_buf), &data->utc) < 0) {
		return;
	}

	LOG_PRINTK("%s: %s\r\n", dev->name, dump_buf);

	LOG_PRINTK("\n");
}

void gnss_publish_data(const struct device *dev, const struct gnss_data *data)
{
	gnss_dump_data_to_log(dev, data);
	K_SPINLOCK(&lock) {
		STRUCT_SECTION_FOREACH(gnss_data_callback, callback) {
			if (callback->dev == NULL || callback->dev == dev) {
				callback->callback(dev, data);
			}
		}
	}
}

#if CONFIG_GNSS_SATELLITES
void gnss_publish_satellites(const struct device *dev, const struct gnss_satellite *satellites,
			     uint16_t size)
{
	K_SPINLOCK(&lock) {
		STRUCT_SECTION_FOREACH(gnss_satellites_callback, callback) {
			if (callback->dev == NULL || callback->dev == dev) {
				callback->callback(dev, satellites, size);
			}
		}
	}
}
#endif
