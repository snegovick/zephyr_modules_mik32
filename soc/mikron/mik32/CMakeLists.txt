# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

#zephyr_sources(entry.S)
#zephyr_sources(soc.c)

zephyr_include_directories(.)

#zephyr_linker_sources(ROM_START SORT_KEY 0x0 eeprom.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
