/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_MIK32_GPIO_IRQ_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_MIK32_GPIO_IRQ_H_

#include <stdint.h>

#include <zephyr/sys/util_macro.h>

/**
 * @name Trigger modes.
 * @anchor MIK32_GPIO_IRQ_TRIG
 * @{
 */

/** No trigger */
#define MIK32_GPIO_IRQ_TRIG_NONE 0U
/** Trigger on rising edge */
#define MIK32_GPIO_IRQ_TRIG_RISING BIT(0)
#define MIK32_GPIO_IRQ_TRIG_HIGH BIT(0)
/** Trigger on falling edge */
#define MIK32_GPIO_IRQ_TRIG_FALLING BIT(1)
#define MIK32_GPIO_IRQ_TRIG_LOW BIT(1)
/** Trigger on rising and falling edge */
#define MIK32_GPIO_IRQ_TRIG_BOTH (MIK32_GPIO_IRQ_TRIG_RISING | MIK32_GPIO_IRQ_TRIG_FALLING)

/** @} */

/** Callback for GPIO IRQ interrupt. */
typedef void (*mik32_gpio_irq_cb_t)(uint8_t line, void *user);

/**
 * @brief Enable GPIO interrupt for the given line.
 *
 * @param line IRQ line.
 */
void mik32_gpio_irq_enable(uint8_t mux_line);

/**
 * @brief Disable GPIO interrupt for the given line.
 *
 * @param line IRQ line.
 */
void mik32_gpio_irq_disable(uint8_t mux_line);

/**
 * @brief Configure GPIO interrupt trigger mode for the given line.
 *
 * @param line IRQ line.
 * @param trigger Trigger mode (see @ref MIK32_GPIO_IRQ_TRIG).
 */
void mik32_gpio_irq_trigger_edge(uint8_t line, uint8_t trigger);
void mik32_gpio_irq_trigger_level(uint8_t line, uint8_t trigger);

/**
 * @brief Configure GPIO interrupt callback.
 *
 * @param line IRQ line.
 * @param cb Callback (NULL to disable).
 * @param user User data (optional).
 *
 * @retval 0 On success.
 * @retval -EALREADY If callback is already set and @p cb is not NULL.
 */
int mik32_gpio_irq_configure(uint8_t line, mik32_gpio_irq_cb_t cb, void *user);
int mik32_gpio_pin_to_mux_line(uint8_t port, uint8_t pin, uint8_t *muxline, uint8_t *muxval);
void mik32_set_irq_mux_line(uint8_t mux_line, uint8_t mux_val);
void mik32_clear_irq_mux_line(uint8_t mux_line);

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_MIK32_GPIO_IRQ_H_ */
