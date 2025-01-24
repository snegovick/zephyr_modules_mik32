# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/i2c.h)

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_I2C_MIK32		i2c_mik32.c)
