/*
 * Copyright (c) 2024 Excave
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * MIK32 SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_MIK32_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_MIK32_COMMON_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <mik32/mik32v2.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/** @brief Type for MIK32 pin.
 *
 * Bits (AF model):
 * - 0-12: MIK32_PINMUX_AF bit field.
 * - 13-25: Reserved.
 * - 26-31: Pin configuration bit field.
 *
 */
typedef uint32_t pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			       \
	(DT_PROP_BY_IDX(node_id, prop, idx) |				       \
	 ((MIK32_PUPD_PULLUP * DT_PROP(node_id, bias_pull_up))		       \
	  << MIK32_PUPD_POS) |						       \
	 ((MIK32_PUPD_PULLDOWN * DT_PROP(node_id, bias_pull_down))	       \
	  << MIK32_PUPD_POS) |						       \
	 (DT_ENUM_IDX(node_id, drive_strength) << MIK32_STRENGTH_POS)),

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		       \
				DT_FOREACH_PROP_ELEM, pinmux,		       \
				Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

/** No pull-up/down */
#define MIK32_PUPD_NONE 0U
/** Pull-up */
#define MIK32_PUPD_PULLUP 1U
/** Pull-down */
#define MIK32_PUPD_PULLDOWN 2U

/** Pin direction */
#define MIK32_DIR_NONE 0x0U
#define MIK32_DIR_OUT 0x1U
#define MIK32_DIR_IN 0x2U

/** Drive strength 2ma */
#define MIK32_STRENGTH_2MA 0x0U
/** 4ma */
#define MIK32_STRENGTH_4MA 0x1U
/** 8ma */
#define MIK32_STRENGTH_8MA 0x2U
/** @} */

/**
 * @name MIK32 pin configuration bit field mask and positions.
 * @anchor MIK32_PINCFG
 *
 * Fields:
 *
 * - 31..30: Pull-up/down
 * - 29..28: Direction
 * - 27..26: Load strength
 *
 * @{
 */

/** PUPD field mask. */
#define MIK32_PUPD_MSK 0x3U
/** PUPD field position. */
#define MIK32_PUPD_POS 30U

/** DIR field mask. */
#define MIK32_DIR_MSK 0x3U
/** DIR field position. */
#define MIK32_DIR_POS 28U

/** Drive strength field mask. */
#define MIK32_STRENGTH_MSK 0x3U
/** Drive strength field position. */
#define MIK32_STRENGTH_POS 26U

#define MIK32_PINMUX_DIR_OUT (MIK32_DIR_OUT << MIK32_DIR_POS)
#define MIK32_PINMUX_DIR_IN (MIK32_DIR_IN << MIK32_DIR_POS)

#define MIK32_PINMUX_STRENGTH_2MA (MIK32_STRENGTH_2MA << MIK32_STRENGTH_POS)
#define MIK32_PINMUX_STRENGTH_4MA (MIK32_STRENGTH_4MA << MIK32_STRENGTH_POS)
#define MIK32_PINMUX_STRENGTH_8MA (MIK32_STRENGTH_8MA << MIK32_STRENGTH_POS)

/** @} */

/**
 * Obtain PUPD field from pinctrl_soc_pin_t configuration.
 *
 * @param pincfg pinctrl_soc_pin_t bit field value.
 */
#define MIK32_PUPD_GET(pincfg) \
	(((pincfg) >> MIK32_PUPD_POS) & MIK32_PUPD_MSK)

/**
 * Obtain STRENGTH field from pinctrl_soc_pin_t configuration.
 *
 * @param pincfg pinctrl_soc_pin_t bit field value.
 */
#define MIK32_STRENGTH_GET(pincfg) \
	(((pincfg) >> MIK32_STRENGTH_POS) & MIK32_STRENGTH_MSK)

#define MIK32_DIR_GET(pincfg) \
	(((pincfg) >> MIK32_DIR_POS) & MIK32_DIR_MSK)


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_SOC_MIK32_COMMON_H_ */
