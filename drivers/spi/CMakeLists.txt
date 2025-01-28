# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/spi.h)

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_SPI_MIK32 spi_mik32.c)
