/*
 * Copyright (c) 2024 Escave
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_mik32_gpio_irq

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util_macro.h>

//#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <zephyr/drivers/interrupt_controller/mik32_gpio_irq.h>
#include <zephyr/drivers/clock_control/mik32.h>
//#include <hal/mik32/shared/include/mik32_memory_map.h>
//#include <hal/mik32/shared/periphery/epic.h>
#include <soc/mikron/mik32/soc.h>
#include <soc/mikron/mik32/soc_gpio.h>

#include <zephyr/soc/mik32_memory_map.h>
#include <zephyr/soc/mik32_irq.h>

/** Unsupported line indicator */
#define GPIO_IRQ_NOTSUP 0xFFU

/** @brief IRQ line interrupt callback. */
struct mik32_cb_data {
	/** Callback function */
	mik32_gpio_irq_cb_t cb;
	/** User data. */
	void *user;
};

/** GPIO IRQ driver data. */
struct mik32_gpio_irq_data {
	/** Array of callbacks. */
	struct mik32_cb_data cbs[MIK32_NUM_IRQ_MUX_LINES];
	uint32_t irq_line_mux;
};

__unused static void mik32_gpio_irq_isr(const void *isr_data)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct mik32_gpio_irq_data *data = dev->data;

	uint32_t stat = MIK32_GPIO_IRQ_INTERRUPT;

	for (int i = 0; i <= MIK32_NUM_IRQ_MUX_LINES; i ++) {
		if ((stat & BIT(i)) != 0) {
			if (data->cbs[i].cb != NULL) {
				data->cbs[i].cb(i, data->cbs[i].user);
			}
		}
	}
}

void mik32_set_irq_mux_line(uint8_t mux_line, uint8_t mux_val) {
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct mik32_gpio_irq_data *data = dev->data;
	data->irq_line_mux &= ~(0xf << (mux_line * 4));
	data->irq_line_mux |= (mux_val << (mux_line * 4));
	MIK32_GPIO_IRQ_LINE_MUX = data->irq_line_mux;
}

void mik32_clear_irq_mux_line(uint8_t mux_line) {
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct mik32_gpio_irq_data *data = dev->data;
	data->irq_line_mux &= ~(0xf << (mux_line * 4));
	MIK32_GPIO_IRQ_LINE_MUX = data->irq_line_mux;
}

int mik32_gpio_pin_to_mux_line(uint8_t port, uint8_t pin, uint8_t *muxline, uint8_t *muxval) {
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct mik32_gpio_irq_data *data = dev->data;
	if (((port < 2) && (pin > 15)) || ((port == 2) && (pin > 7)) || (port > 2)) {
		return -ENOTSUP;
	}
	unsigned int offset = pin + (port * 16);
	unsigned int mux_val = offset / 8;
	unsigned int mux_line = offset % 8;

	if ((data->irq_line_mux & (0xf << (mux_line * 4))) == 0) {
		*muxval = mux_val;
		*muxline = mux_line;
		return 0;
	}
	// Primary mux line is busy, check secondary
	mux_val += 5;
	mux_line = (mux_line >= 4 ? mux_line - 4 : mux_line + 4);
	if ((data->irq_line_mux & (0xf << (mux_line * 4))) == 0) {
		*muxval = mux_val;
		*muxline = mux_line;
		return 0;
	}
	return -ERANGE;
}

void mik32_gpio_irq_enable(uint8_t mux_line)
{
	__ASSERT_NO_MSG(mux_line < MIK32_NUM_IRQ_MUX_LINES);

	MIK32_GPIO_IRQ_ENABLE_SET = (1 << mux_line);
}

void mik32_gpio_irq_disable(uint8_t mux_line)
{
	__ASSERT_NO_MSG(mux_line < MIK32_NUM_IRQ_MUX_LINES);
	
	MIK32_GPIO_IRQ_ENABLE_CLEAR = (1 << mux_line);
	MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
	MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
}

void mik32_gpio_irq_trigger_edge(uint8_t mux_line, uint8_t trigger)
{
	__ASSERT_NO_MSG(mux_line < MIK32_NUM_IRQ_MUX_LINES);
	MIK32_GPIO_IRQ_EDGE = (1 << mux_line);

	switch (trigger) {
	case MIK32_GPIO_IRQ_TRIG_LOW:
		MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
		break;
	case MIK32_GPIO_IRQ_TRIG_HIGH:
		MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL_SET = (1 << mux_line);
		break;
	case MIK32_GPIO_IRQ_TRIG_BOTH:
		MIK32_GPIO_IRQ_ANY_EDGE_SET = (1 << mux_line);
		break;
	default:
		return;
	}
}

void mik32_gpio_irq_trigger_level(uint8_t mux_line, uint8_t trigger)
{
	MIK32_GPIO_IRQ_LEVEL = (1 << mux_line);
	switch (trigger) {
	case MIK32_GPIO_IRQ_TRIG_LOW:
		MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
		break;
	case MIK32_GPIO_IRQ_TRIG_HIGH:
		MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL_SET = (1 << mux_line);
		break;
	case MIK32_GPIO_IRQ_TRIG_BOTH:
		MIK32_GPIO_IRQ_ANY_EDGE_SET = (1 << mux_line);
		break;
	default:
		return;
	}
}

int mik32_gpio_irq_configure(uint8_t line, mik32_gpio_irq_cb_t cb, void *user)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct mik32_gpio_irq_data *data = dev->data;

	__ASSERT_NO_MSG(line < MIK32_NUM_IRQ_MUX_LINES);

	if ((data->cbs[line].cb != NULL) && (cb != NULL)) {
		return -EALREADY;
	}

	data->cbs[line].cb = cb;
	data->cbs[line].user = user;

	return 0;
}

static int mik32_gpio_irq_init(const struct device *dev)
{

	IRQ_CONNECT(EPIC_GPIO_IRQ_INDEX,
		    0,
		    mik32_gpio_irq_isr, 0, 0);
	return 0;
}

static struct mik32_gpio_irq_data data = {
	.irq_line_mux = 0,
};

DEVICE_DT_INST_DEFINE(0, mik32_gpio_irq_init, NULL, &data, NULL, PRE_KERNEL_1,
		      CONFIG_INTC_INIT_PRIORITY, NULL);
