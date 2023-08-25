/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FXLS8971CF_HEADERS
#define FXLS8971CF_HEADERS

#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>

#define status_success                  0
#define status_failure                  1

typedef int32_t status_t;

#define FXLS8971CF_ACCEL_RESOLUTION_BITS          12
#define FXLS8971CF_SPI_MAX_FREQUENCY_HZ           4000000U
#define TEMP_OFFSET                               25
#define INITIAL_MEASURE_DELAY_NS                  528500
#define INCREMENTAL_MEASURE_DELAY_NS              312500

/*
 *  STATUS Register
 */
#define INT_STATUS_REG                  0x00

	/* Content for STATUS register */
	#define SRC_DRDY_MASK                 BIT(7)
	#define SRC_OVF_MASK                  BIT(6)
	#define SRC_ORIENT_MASK               BIT(2)
	#define SRC_ASLP_MASK                 BIT(1)
	#define BOOT_MODE_MASK                BIT(0)

/*
 *  TEMPERATURE Value Register
 */
#define TEMP_OUT_REG                    0x01

/*
 *  VECTOR Magnitude Result Registers
 */
#define VECM_LSB_REG                    0x02
#define VECM_MSB_REG                    0x03

/*
 *  XYZ Data Registers
 */
#define OUT_X_LSB_REG                   0x04
#define OUT_X_MSB_REG                   0x05
#define OUT_Y_LSB_REG                   0x06
#define OUT_Y_MSB_REG                   0x07
#define OUT_Z_LSB_REG                   0x08
#define OUT_Z_MSB_REG                   0x09

/* XYZ Data Buffer Status Register */
#define BUF_STATUS_REG                  0x0B

/* XYZ Sample Data Registers */
#define BUF_X_LSB_REG                   0x0C
#define BUF_X_MSB_REG                   0x0D
#define BUF_Y_LSB_REG                   0x0E
#define BUF_Y_MSB_REG                   0x0F
#define BUF_Z_LSB_REG                   0x10
#define BUF_Z_MSB_REG                   0x11

/* PRODUCT Revision Number Register */
#define PROD_REV_REG                    0x12

/* WHO_AM_I Device ID Register */
#define WHO_AM_I_REG                    0x75

	/* Content for WHO_AM_I Device ID Register */
	#define FXLS8971CF_WHO_AM_I_DEVICE_ID           0x47

/*
 *  CURRENT Device Operating Mode Register
 */
#define SYS_MODE_REG 0x14

	/* Content for SYS_MODE register */
	#define SYS_MODE_MASK 0b11

		/* Content for SYS_MODE_MASK */
		#define STANDBY_MODE_MASK 0b00
		#define WAKE_MODE_MASK 0b01
		#define SLEEP_MODE_MASK 0b10
		#define EXT_TRIG_MODE_MASK 0b11

/* CONFIG Registers */
#define SENS_CONFIG1_REG                0x15U
#define SENS_CONFIG2_REG                0x16
#define SENS_CONFIG3_REG                0x17
#define SENS_CONFIG4_REG                0x18
#define SENS_CONFIG5_REG                0x19

	/* Content for CONFIG1 register */
	#define ACTIVE_MODE_MASK              0x01U
	#define SPI_M_MASK                    BIT(3)

	#define FSR_SHIFT                     1
	#define FSR_2G_MASK                   0b00 << FSR_SHIFT
	#define FSR_4G_MASK                   0b01 << FSR_SHIFT
	#define FSR_8G_MASK                   0b10 << FSR_SHIFT
	#define FSR_16G_MASK                  0b11 << FSR_SHIFT

	/* Content for CONFIG2 register */
	#define CONFIG2_REG_RESET             0
	#define LE_BE_MASK                    BIT(3)
	#define AINC_TEMP_MASK                BIT(1)

	#define WAKE_PM_SHIFT                            6
	#define WAKE_PM_LOW_POWER_MODE                   0b00 << WAKE_PM_SHIFT
	#define WAKE_PM_HIGH_PERFORMANCE_MODE            0b01 << WAKE_PM_SHIFT
	#define WAKE_PM_FLEXIBLE_PERFORMANCE_MODE        0b10 << WAKE_PM_SHIFT
	#define SLEEP_PM_SHIFT                           4
	#define SLEEP_PM_LOW_POWER_MODE                  0b00 << SLEEP_PM_SHIFT
	#define SLEEP_PM_HIGH_PERFORMANCE_MODE           0b01 << SLEEP_PM_SHIFT
	#define SLEEP_PM_FLEXIBLE_PERFORMANCE_MODE       0b10 << SLEEP_PM_SHIFT

	/* Content for CONFIG4 register */
	#define CONFIG4_REG_RESET             1

/*
 *  MODE Registers
 */
#define WAKE_IDLE_LSB_REG               0x1A
#define WAKE_IDLE_MSB_REG               0x1B
#define SLEEP_IDLE_LSB_REG              0x1C
#define SLEEP_IDLE_MSB_REG              0x1D
#define ASLP_COUNT_LSB_REG              0x1E
#define ASLP_COUNT_MSB_REG              0x1F

	/* Content for ASLP_COUNT register */
	#define ASLP_COUNT_REG_RESET          0

/*
 *  INTERRUPT Registers
 */
#define INT_EN_REG      0x20
#define INT_PIN_SEL_REG 0x21

/* XYZ Offset Correction Registers */
#define OFF_X_REG 0x22
#define OFF_Y_REG 0x23
#define OFF_Z_REG 0x24

/* BUFFER Config Registers */
#define BUF_CONFIG1_REG 0x26
#define BUF_CONFIG2_REG 0x27

/* Time needed to enter modes. */
#define T_BOOT_1_MS 1
	// Time needed to enter "Standby"          mode after POR or soft reset.
#define T_BOOT_2_US 17100
	// Time needed to enter "Motion Detection" mode after POR or soft reset

/*! @brief fxls8971cf raw sensor accel data structure. */
typedef struct _fxls8971cf_raw_sensor_data { /* Data in unsigned format. */
	uint8_t accelXLSB;   /* 8-bit X-axis LSB output acceleration data. */
	uint8_t accelXMSB;   /* 4-bit X-axis MSB output acceleration data. */
	uint8_t accelYLSB;   /* 8-bit Y-axis LSB output acceleration data. */
	uint8_t accelYMSB;   /* 4-bit Y-axis MSB output acceleration data. */
	uint8_t accelZLSB;   /* 8-bit Z-axis LSB output acceleration data. */
	uint8_t accelZMSB;   /* 4-bit Z-axis MSB output acceleration data. */
} fxls8971cf_raw_sensor_data_t;

/*! @brief fxls8971cf synthesized accel data structure. */
typedef struct _fxls8971cf_synthesized_data { /* Data in 2's complement format. */
	int16_t accelX;      /* Synthesized 12-bit X-axis output acceleration data. */
	int16_t accelY;      /* Synthesized 12-bit Y-axis output acceleration data. */
	int16_t accelZ;      /* Synthesized 12-bit Z-axis output acceleration data. */
} fxls8971cf_synthesized_data_t;

/*! @brief fxls8971cf device data structure. */
typedef struct _fxls8971cf_dev_data {
	fxls8971cf_raw_sensor_data_t raw_sensor_data;
	fxls8971cf_synthesized_data_t synthesized_data;
	int8_t temp_celsius;

	struct k_sem lock;
} fxls8971cf_dev_data_t;

/*! @brief fxls8971cf device config structure. */
typedef struct _fxls8971cf_dev_config {
	const struct device *spi_dev;
	const struct spi_config spi_cfg;
} fxls8971cf_dev_config;

typedef status_t (*fxls8971cf_api_read_data)(const struct device *dev);
typedef status_t (*fxls8971cf_api_display_data)(const struct device *dev);

/*! @brief fxls8971cf device api. */
__subsystem struct fxls8971cf_dev_api {
	fxls8971cf_api_read_data read_data;
	fxls8971cf_api_display_data display_data;
};

#endif // FXLS8971CF_HEADERS
