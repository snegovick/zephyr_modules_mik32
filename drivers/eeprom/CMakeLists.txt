# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/eeprom.h)

#zephyr_library()

zephyr_library_sources_ifdef(CONFIG_EEPROM_MIK32_8K_BLK eeprom_mikron_8k_blk.c)
