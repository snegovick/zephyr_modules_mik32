# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_MIK32 clock_control_mik32.c)
