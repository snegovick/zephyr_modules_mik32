# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MEMC_MIKRON_SPIFI memc_mikron_spifi.c)
