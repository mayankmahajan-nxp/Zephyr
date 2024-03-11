/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L4CD Ultra Lite Driver and : dual licensed, either
 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L4CD Ultra Lite Driver may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

#ifndef ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_
#define ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

/**
 * @struct VL53L4CD_Dev_t
 * @brief  Generic PAL device type that does link between API and platform
 * abstraction layer
 *
 */
typedef struct {
	const struct device *i2c_bus;
	uint8_t i2c_dev_addr;
} VL53L4CD_Dev_t;

/**
 * @brief Declare the device Handle as a pointer of the structure VL53L4CD_Dev_t
 *
 */
typedef VL53L4CD_Dev_t *VL53L4CD_DEV;

/**
* VL53L4CD device instance.
*/

typedef uint16_t Dev_t;

/**
 * @brief Error instance.
 */
typedef uint8_t VL53L4CD_Error;

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

// #define VL53L4CD_I2C_FAST_MODE_PLUS

/**
 * @brief Read 32 bits through I2C.
 */

uint8_t VL53L4CD_RdDWord(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint32_t *data);

/**
 * @brief Read 16 bits through I2C.
 */

uint8_t VL53L4CD_RdWord(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint16_t *data);

/**
 * @brief Read 8 bits through I2C.
 */

uint8_t VL53L4CD_RdByte(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint8_t *data);

/**
 * @brief Write 8 bits through I2C.
 */

uint8_t VL53L4CD_WrByte(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint8_t data);

/**
 * @brief Write 16 bits through I2C.
 */

uint8_t VL53L4CD_WrWord(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint16_t data);

/**
 * @brief Write 32 bits through I2C.
 */

uint8_t VL53L4CD_WrDWord(VL53L4CD_Dev_t *dev, uint16_t register_addr, uint32_t data);

/**
 * @brief Wait during N milliseconds.
 */

uint8_t WaitMs(VL53L4CD_Dev_t *dev, uint32_t TimeMs);

#endif	// ZEPHYR_DRIVERS_SENSOR_VL53L4CD_VL53L4CD_PLATFORM_H_
