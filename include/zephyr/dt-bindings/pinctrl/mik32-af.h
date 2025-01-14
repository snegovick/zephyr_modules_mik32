/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DT_BINDINGS_PINCTRL_MIK32_AF_H_
#define DT_BINDINGS_PINCTRL_MIK32_AF_H_

/**
 * @name MIK32 AFs
 * @{
 */

/** AF0 */
#define MIK32_AF0 0U
/** AF1 */
#define MIK32_AF1 1U
/** AF2 */
#define MIK32_AF2 2U
/** ANALOG */
#define MIK32_ANALOG 3U

/**
 * @name MIK32 pinmux bit field mask and positions.
 * @{
 */

/** Port field mask. */
#define MIK32_PORT_MSK 0x3U
/** Port field position. */
#define MIK32_PORT_POS 0U
/** Pin field mask. */
#define MIK32_PIN_MSK 0xFU
/** Pin field position. */
#define MIK32_PIN_POS 4U
/** AF field mask. */
#define MIK32_AF_MSK 0x3U
/** AF field position. */
#define MIK32_AF_POS 8U

/** @} */

/**
 * Obtain port field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define MIK32_PORT_GET(pinmux) \
	(((pinmux) >> MIK32_PORT_POS) & MIK32_PORT_MSK)

/**
 * Obtain pin field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define MIK32_PIN_GET(pinmux) \
	(((pinmux) >> MIK32_PIN_POS) & MIK32_PIN_MSK)

/**
 * Obtain AF field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define MIK32_AF_GET(pinmux) \
	(((pinmux) >> MIK32_AF_POS) & MIK32_AF_MSK)

/**
 * @brief Remap configuration bit field.
 * 
 * Fields:
 *
 * - 0..3: port
 * - 4..7: pin
 * - 8..12: af
 * 
 * @param port Port ('A'..'C')
 * @param pin Pin (0..15)
 * @param af Alternate function (ANALOG, AFx, x=0..2)
 */

#define MIK32_PINMUX_AF(port, pin, af)					\
	(((((port) - 'A') & MIK32_PORT_MSK) << MIK32_PORT_POS) |	\
	 (((pin) & MIK32_PIN_MSK) << MIK32_PIN_POS) |			\
	 (((MIK32_ ## af) & MIK32_AF_MSK) << MIK32_AF_POS))

#endif /* DT_BINDINGS_PINCTRL_MIK32_AF_H_ */
