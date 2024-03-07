# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
	vl53l4cd.c
	VL53L4CD_ULD/VL53L4CD_ULD_Driver/vl53l4cd_api.c
	VL53L4CD_ULD/VL53L4CD_ULD_Driver/vl53l4cd_calibration.c
	VL53L4CD_ULD/Platform/platform.c
)
