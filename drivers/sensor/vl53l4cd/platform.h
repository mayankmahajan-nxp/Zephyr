/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file was referred from the ULD API of VL53L4CD by STMicroelectronics.
 * Link: https://www.st.com/en/embedded-software/stsw-img026.html#get-software.
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_

#pragma once

#include <stdint.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

/**
 * @struct VL53L4CD_Dev
 * @brief  Generic PAL device type that links API and platform abstraction layer.
 */

struct VL53L4CD_Dev {
	const struct device *i2c_bus;
	uint8_t i2c_dev_addr;
};

/**
 *  @brief Driver error type
 */

enum VL53L4CD_Error {
	VL53L4CD_ERROR_NONE = 0U,
	VL53L4CD_ERROR_XTALK_FAILED = 253U,
	VL53L4CD_ERROR_INVALID_ARGUMENT = 254U,
	VL53L4CD_ERROR_TIMEOUT = 255U,
};

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

/* #define VL53L4CD_I2C_FAST_MODE_PLUS */

/**
 * @brief Read 32 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t *value);

/**
 * @brief Read 16 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t *value);

/**
 * @brief Read 8 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_RdByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t *value);

/**
 * @brief Write 8 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */

enum VL53L4CD_Error VL53L4CD_WrDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */

enum VL53L4CD_Error VL53L4CD_PollingDelay(struct VL53L4CD_Dev *dev, uint32_t TimeMs);

#endif  /* ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_ */
