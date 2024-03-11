# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
	vl53l4cd.c
	VL53L4CD_api.c
	platform.c
	VL53L4CD_calibration.c
)
