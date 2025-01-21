# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
zephyr_include_directories(./)
zephyr_include_directories(dts/riscv/mikron)
# zephyr_include_directories(hal)

add_subdirectory(drivers)
# add_subdirectory(hal)
# add_subdirectory(soc)
