# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

# Referred from file "zephyr/samples/sensor/ina219/CMakeLists.txt".

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ina226)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
