# Copyright (c) 2025 Excave.ru
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
zephyr_include_directories(./)
zephyr_include_directories(soc/mikron/mik32)
zephyr_include_directories(dts/riscv/mikron)

add_subdirectory(drivers)

