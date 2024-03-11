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

#include "vl53l4cd_platform.h"
#include <zephyr/sys/byteorder.h>

uint8_t VL53L4CD_RdDWord(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 255;

	int ret;

	RegisterAdress = BSWAP_16(RegisterAdress);
	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &RegisterAdress, 2, (uint8_t *) value, 4);
	if (ret < 0) {
		printk("i2c_write_read failed (%d)\n", ret);
	}
	*value = BSWAP_32(*value);

	return status;
}

uint8_t VL53L4CD_RdWord(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 255;

	int ret;

	RegisterAdress = BSWAP_16(RegisterAdress);
	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &RegisterAdress, 2, (uint8_t *) value, 2);
	if (ret < 0) {
		printk("i2c_write_read failed (%d)\n", ret);
	}
	*value = BSWAP_16(*value);

	return status;
}

uint8_t VL53L4CD_RdByte(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 255;

	int ret;

	RegisterAdress = BSWAP_16(RegisterAdress);
	ret = i2c_write_read(dev->i2c_bus, dev->i2c_dev_addr, &RegisterAdress, 2, value, 1);
	if (ret < 0) {
		printk("i2c_write_read failed (%d)\n", ret);
	}

	return status;
}

uint8_t VL53L4CD_WrByte(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t status = 255;

	int ret;

	RegisterAdress = BSWAP_16(RegisterAdress);
	uint8_t buf[3];
	memcpy(buf, &RegisterAdress, 2);
	memcpy(buf + 2, &value, 1);
	ret = i2c_write(dev->i2c_bus, buf, 3, dev->i2c_dev_addr);

	return status;
}

uint8_t VL53L4CD_WrWord(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t status = 255;

	int ret;

	value = BSWAP_16(value);
	RegisterAdress = BSWAP_16(RegisterAdress);
	uint8_t buf[4];
	memcpy(buf, &RegisterAdress, 2);
	memcpy(buf + 2, &value, 2);
	ret = i2c_write(dev->i2c_bus, buf, 4, dev->i2c_dev_addr);

	return status;
}

uint8_t VL53L4CD_WrDWord(VL53L4CD_Dev_t *dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t status = 255;

	int ret;

	value = BSWAP_32(value);
	RegisterAdress = BSWAP_16(RegisterAdress);
	uint8_t buf[6];
	memcpy(buf, &RegisterAdress, 2);
	memcpy(buf + 2, &value, 4);
	ret = i2c_write(dev->i2c_bus, buf, 6, dev->i2c_dev_addr);

	return status;
}

uint8_t VL53L4CD_PollingDelay(VL53L4CD_Dev_t *dev, uint32_t TimeMs)
{
	uint8_t status = 255;
	k_sleep(K_MSEC(TimeMs));
	return status;
}
