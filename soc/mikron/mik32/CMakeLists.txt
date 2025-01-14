# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

#zephyr_sources(reset.S)
zephyr_sources(soc.c)

zephyr_include_directories(.)

#zephyr_linker_sources(ROM_START SORT_KEY 0x0 init.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
#set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/spifi.ld CACHE INTERNAL "")
