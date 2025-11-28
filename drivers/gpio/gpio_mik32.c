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

//#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <zephyr/drivers/clock_control/mik32.h>
#include <zephyr/drivers/interrupt_controller/mik32_gpio_irq.h>
//#include <hal/mik32/shared/include/mik32_memory_map.h>
//#include <hal/mik32/shared/periphery/epic.h>
#include <soc/mikron/mik32/soc.h>
#include <soc/mikron/mik32/soc_gpio.h>

#include <zephyr/soc/mik32_irq.h>
#include <zephyr/soc/mik32_memory_map.h>

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
	//uint32_t ds;
	uint32_t cfg;
	uint32_t dirin;
	uint32_t dirout;

	pupd = MIK32_PAD_PUPD(config->reg);
	cfg = MIK32_PAD_CFG(config->reg);
	//ds = MIK32_PAD_DS(config->reg);
	dirin = 0;//MIK32_GPIO_DIRIN(config->reg);
	dirout = 0;//MIK32_GPIO_DIROUT(config->reg);

	if ((flags & GPIO_OUTPUT) != 0U) {
		// set mode GPIO
		cfg &= ~(0x3 << (pin * 2));
		// set direction output
		dirout |= BIT(pin);

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			MIK32_GPIO_STATE(config->reg) = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			MIK32_GPIO_CLEAR(config->reg) = BIT(pin);
		}
	} else if ((flags & GPIO_INPUT) != 0U) {
		// set mode GPIO
		cfg &= ~(0x3 << (pin * 2));
		// set direction input
		dirin |= BIT(pin);
	} else {
		// set mode ANALOG
		cfg |= (0x3 << (pin * 2));
		// set direction input
		dirin |= BIT(pin);
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

	MIK32_GPIO_STATE(config->reg) = pins;

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

static int gpio_mik32_pin_interrupt_configure(const struct device *port,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
#if IS_ENABLED(CONFIG_MIK32_GPIO_IRQ)
	uint8_t mux_line = 0;
	uint8_t mux_val = 0;
	const struct gpio_mik32_config *config = port->config;
	//struct gpio_mik32_data *data = port->data;
	unsigned int gpion = MIK32_GPIO_NUMBER(config->reg);
	int ret = mik32_gpio_pin_to_mux_line(gpion, pin, &mux_line, &mux_val);
	if (ret < 0) {
		return ret;
	}
	if (mode == GPIO_INT_MODE_DISABLED) {
		mik32_gpio_irq_disable(mux_line);
		mik32_clear_irq_mux_line(mux_line);
		(void)mik32_gpio_irq_configure(mux_line, NULL, NULL);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		mik32_set_irq_mux_line(mux_line, mux_val);
		(void)mik32_gpio_irq_configure(mux_line, gpio_mik32_isr, (void *)port);

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_FALLING);
			break;
		case GPIO_INT_TRIG_HIGH:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_RISING);
			break;
		case GPIO_INT_TRIG_BOTH:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_BOTH);
			break;
		default:
			return -ENOTSUP;
		}

		mik32_gpio_irq_enable(mux_line);
		HAL_EPIC_MaskEdgeSet(HAL_EPIC_GPIO_IRQ_MASK);
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		mik32_set_irq_mux_line(mux_line, mux_val);
		(void)mik32_gpio_irq_configure(mux_line, gpio_mik32_isr, (void *)port);

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_LOW);
			break;
		case GPIO_INT_TRIG_HIGH:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_HIGH);
			break;
		case GPIO_INT_TRIG_BOTH:
			mik32_gpio_irq_trigger_level(mux_line, MIK32_GPIO_IRQ_TRIG_BOTH);
			break;
		default:
			return -ENOTSUP;
		}
		mik32_gpio_irq_enable(mux_line);
		HAL_EPIC_MaskLevelSet(HAL_EPIC_GPIO_IRQ_MASK);
	} else {
		return -ENOTSUP;
	}

	return 0;
#else
	return -ENOTSUP;
#endif
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
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, gpio_mik32_init, NULL, &gpio_mik32_data##n, \
			      &gpio_mik32_config##n, PRE_KERNEL_1,	\
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_mik32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MIK32_DEFINE)
