/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_MIK32_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_MIK32_H_

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the MIK32 clock controller.
 *
 * There is a single clock controller in the MIK32: pmgr (PowerManager as per docs).
 * The device can be used without checking for it to be ready since it has no initialization
 * code subject to failures.
 */

#define MIK32_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(pmgr))

#endif /*ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_MIK32_H_*/
