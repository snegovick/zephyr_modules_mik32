/*
 * Copyright (c) 2025 Excave.ru
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MIK32_PINCFG_H
#define __MIK32_PINCFG_H

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

#endif/*__MIK32_PINCFG_H*/
