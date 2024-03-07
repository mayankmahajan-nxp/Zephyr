/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file was referred from the ULD API of VL53L4CD by STMicroelectronics.
 * Link: https://www.st.com/en/embedded-software/stsw-img026.html#get-software.
 */

#include "platform.h"

/* TODO: create a generic read function for the following three read functions for code reusability.
 * The three read functions will BSWAP_n after getting data. Similarly for three write functions.
 */

enum VL53L4CD_Error VL53L4CD_RdDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);

	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	*value = BSWAP_32(*value);

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_RdWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);

	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	*value = BSWAP_16(*value);

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_RdByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t *value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(*value);

	reg_addr = BSWAP_16(reg_addr);
	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &reg_addr, register_addr_sz,
			     value, value_sz);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrByte(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint8_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, 3, dev->i2c_dev_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint16_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	value = BSWAP_16(value);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, buf_size, dev->i2c_dev_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_WrDWord(struct VL53L4CD_Dev *dev, uint16_t reg_addr, uint32_t value)
{
	int ret;
	uint8_t register_addr_sz = sizeof(reg_addr), value_sz = sizeof(value);
	uint8_t buf_size = register_addr_sz + value_sz, buf[buf_size];

	reg_addr = BSWAP_16(reg_addr);
	value = BSWAP_32(value);
	memcpy(buf, &reg_addr, register_addr_sz);
	memcpy(buf + register_addr_sz, &value, value_sz);

	ret = i2c_write(dev->i2c_bus, buf, buf_size, dev->i2c_dev_addr);
	if (ret < 0) {
		return VL53L4CD_ERROR_XTALK_FAILED;
	}

	return VL53L4CD_ERROR_NONE;
}

enum VL53L4CD_Error VL53L4CD_PollingDelay(struct VL53L4CD_Dev *dev, uint32_t TimeMs)
{
	int ret;

	ret = k_sleep(K_MSEC(TimeMs));
	if (ret != 0) {
		return VL53L4CD_ERROR_TIMEOUT;
	}

	return VL53L4CD_ERROR_NONE;
}
