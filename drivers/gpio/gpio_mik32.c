/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_mik32_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <hal/mik32/peripherals/Include/mik32_hal_gpio.h>
#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <include/drivers/clock_control/mik32.h>
#include <soc/mikron/mik32/soc.h>

#define GPIO_IRQ_NODE DT_NODELABEL(gpio_irq)

struct gpio_mik32_config {
	struct gpio_driver_config common;
	uint32_t reg;
	uint16_t clkid;
	uint16_t clkid_gpio_irq;
	struct reset_dt_spec reset;
};

struct gpio_mik32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	uint32_t irq_line_mux;
};

/**
 * @brief EXTI ISR callback.
 *
 * @param line EXTI line (equals to GPIO pin number).
 * @param arg GPIO port instance.
 */
static void gpio_mik32_isr(uint8_t line, void *arg)
{
	const struct device *dev = arg;
	struct gpio_mik32_data *data = dev->data;

	gpio_fire_callbacks(&data->callbacks, dev, BIT(line));
}

static inline int gpio_mik32_configure(const struct device *port, gpio_pin_t pin,
				       gpio_flags_t flags)
{
	const struct gpio_mik32_config *config = port->config;

	uint32_t pupd;
	uint32_t ds;
	uint32_t cfg;
	uint32_t dirin;
	uint32_t dirout;

	pupd = MIK32_PAD_PUPD(config->reg);
	cfg = MIK32_PAD_CFG(config->reg);
	ds = MIK32_PAD_DS(config->reg);
	dirin = MIK32_GPIO_DIRIN(config->reg);
	dirout = MIK32_GPIO_DIROUT(config->reg);

	if ((flags & GPIO_OUTPUT) != 0U) {
		// set mode GPIO
		cfg &= ~(0x3 << (pin * 2));
		// set direction output
		dirout |= (1 << pin);

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			GPIO_STATE(config->reg) = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			GPIO_CLEAR(config->reg) = BIT(pin);
		}
	} else if ((flags & GPIO_INPUT) != 0U) {
		// set mode GPIO
		cfg &= ~(0x3 << (pin * 2));
		// set direction input
		dirin |= (1 << pin);
	} else {
		// set mode ANALOG
		cfg |= (0x3 << pin*2);
		// set direction input
		dirin |= (1 << pin);
	}

	if ((flags & GPIO_PULL_UP) != 0U) {
		pupd &= ~(0x3 << (pin * 2));
		pupd |= (0x1 << (pin * 2));
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		pupd &= ~(0x3 << (pin * 2));
		pupd |= (0x2 << (pin * 2));
	} else {
		pupd &= ~(0x3 << (pin * 2));
	}

	MIK32_PAD_PUPD(config->reg) = pupd;
	MIK32_PAD_CFG(config->reg) = cfg;
	MIK32_GPIO_DIRIN(config->reg) = dirin;
	MIK32_GPIO_DIROUT(config->reg) = dirout;

	return 0;
}

static int gpio_mik32_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_mik32_config *config = port->config;

	*value = MIK32_GPIO_OUTPUT(config->reg);

	return 0;
}

static int gpio_mik32_port_set_masked_raw(const struct device *port,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_mik32_config *config = port->config;

	MIK32_GPIO_CLEAR(config->reg) = ((~value & mask));
	MIK32_GPIO_STATE(config->reg) = (value & mask);

	return 0;
}

static int gpio_mik32_port_set_bits_raw(const struct device *port,
				       gpio_port_pins_t pins)
{
	const struct gpio_mik32_config *config = port->config;

	MIK32_GPIO_STATE(config->reg) = value;

	return 0;
}

static int gpio_mik32_port_clear_bits_raw(const struct device *port,
					 gpio_port_pins_t pins)
{
	const struct gpio_mik32_config *config = port->config;

	MIK32_GPIO_CLEAR(config->reg) = pins;

	return 0;
}

static int gpio_mik32_port_toggle_bits(const struct device *port,
				      gpio_port_pins_t pins)
{
	const struct gpio_mik32_config *config = port->config;

	uint32_t state = MIK32_GPIO_STATE(config->reg);

	MIK32_GPIO_CLEAR(config->reg) = (state & pins);
	MIK32_GPIO_STATE(config->reg) = (state ^ pins);

	return 0;
}

static int __gpio_pin_to_mux_line(struct gpio_mik32_data *data, uint8_t port, uint8_t pin, uint8_t *muxline, uint8_t *muxval) {
	if (((port < 2) && (pin > 15)) || ((port == 2) && (pin > 7)) || (port > 2)) {
		return -ENOTSUP;
	}
	unsigned int offset = pin + (port * 16);
	unsigned int mux_val = offset / 8;
	unsigned int mux_line = offset % 8;

	if (data->irq_line_mux & (0xf << (mux_line * 4)) == 0) {
		*muxval = mux_val;
		*muxline = mux_line;
		return 0;
	}
	// Primary mux line is busy, check secondary
	mux_val += 5;
	mux_line = (mux_line >= 4 ? mux_line - 4 : mux_line + 4);
	if (data->irq_line_mux & (0xf << (mux_line * 4)) == 0) {
		*muxval = mux_val;
		*muxline = mux_line;
		return 0;
	}
	return -ERANGE;
}

static int __set_irq_mux_line(struct gpio_mik32_data *data, uint8_t mux_line, uint8_t mux_val) {
	data->irq_line_mux &= ~(0xf << (mux_line * 4));
	data->irq_line_mux |= (mux_val << (mux_line * 4));
	MIK32_GPIO_IRQ_LINE_MUX = data->irq_line_mux;
}

static int __clear_irq_mux_line(struct gpio_mik32_data *data, uint8_t mux_line) {
	data->irq_line_mux &= ~(0xf << (mux_line * 4));
	MIK32_GPIO_IRQ_LINE_MUX = data->irq_line_mux;
}

static int gpio_mik32_pin_interrupt_configure(const struct device *port,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	uint8_t mux_line = 0;
	uint8_t mux_val = 0;
	unsigned int gpion = MIK32_GPIO_NUMBER(port->reg);
	struct gpio_mik32_data *data = port->data;
	int ret = __gpio_pin_to_mux_line(data, gpion, pin, &mux_line, &mux_val);
	if (ret < 0) {
		return ret;
	}
	if (mode == GPIO_INT_MODE_DISABLED) {
		__clear_irq_mux_line(data, mux_line);
		MIK32_GPIO_IRQ_ENABLE_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL = (1 << mux_line);
		MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
		MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		__set_irq_mux_line(data, mux_line, mux_val);
		MIK32_GPIO_IRQ_EDGE = (1 << mux_line);

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
			MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
			break;
		case GPIO_INT_TRIG_HIGH:
			GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
			GPIO_IRQ_LEVEL_SET = (1 << mux_line);
			break;
		case GPIO_INT_TRIG_BOTH:
			GPIO_IRQ_ANY_EDGE_SET = (1 << mux_line);
			break;
		default:
			return -ENOTSUP;
		}

		MIK32_GPIO_IRQ_ENABLE_SET = (1 << mux_line);
		HAL_EPIC_MaskEdgeSet(HAL_EPIC_GPIO_IRQ_MASK);
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		__set_irq_mux_line(data, mux_line, mux_val);
		MIK32_GPIO_IRQ_LEVEL = (1 << mux_line);

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
			MIK32_GPIO_IRQ_LEVEL_CLEAR = (1 << mux_line);
			break;
		case GPIO_INT_TRIG_HIGH:
			MIK32_GPIO_IRQ_ANY_EDGE_CLEAR = (1 << mux_line);
			MIK32_GPIO_IRQ_LEVEL_SET = (1 << mux_line);
			break;
		case GPIO_INT_TRIG_BOTH:
			MIK32_GPIO_IRQ_ANY_EDGE_SET = (1 << mux_line);
			break;
		default:
			return -ENOTSUP;
		}

		MIK32_GPIO_IRQ_ENABLE_SET = (1 << mux_line);
		HAL_EPIC_MaskLevelSet(HAL_EPIC_GPIO_IRQ_MASK);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_mik32_manage_callback(const struct device *dev,
				     struct gpio_callback *callback, bool set)
{
	struct gpio_mik32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static DEVICE_API(gpio, gpio_mik32_api) = {
	.pin_configure = gpio_mik32_configure,
	.port_get_raw = gpio_mik32_port_get_raw,
	.port_set_masked_raw = gpio_mik32_port_set_masked_raw,
	.port_set_bits_raw = gpio_mik32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mik32_port_clear_bits_raw,
	.port_toggle_bits = gpio_mik32_port_toggle_bits,
	.pin_interrupt_configure = gpio_mik32_pin_interrupt_configure,
	.manage_callback = gpio_mik32_manage_callback,
};

static int gpio_mik32_init(const struct device *port)
{
	const struct gpio_mik32_config *config = port->config;
	//struct gpio_mik32_data *data = port->data;

	(void)clock_control_on(MIK32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&config->clkid);
	(void)clock_control_on(MIK32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&config->clkid_gpio_irq);

	return 0;
}

#define GPIO_MIK32_DEFINE(n)						\
	static const struct gpio_mik32_config gpio_mik32_config##n = {	\
		.common = {						\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n), \
		},							\
		.reg = DT_INST_REG_ADDR(n),				\
		.clkid = DT_INST_CLOCKS_CELL(n, id),			\
		.clkid_gpio_irq = DT_CLOCKS_CELL(GPIO_IRQ_NODE, id),	\
	};								\
									\
	static struct gpio_mik32_data gpio_mik32_data##n = {		\
		.irq_line_mux = 0,					\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, gpio_mik32_init, NULL, &gpio_mik32_data##n, \
			      &gpio_mik32_config##n, PRE_KERNEL_1,	\
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_mik32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MIK32_DEFINE)
