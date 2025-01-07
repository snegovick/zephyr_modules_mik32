/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <drivers/clock_control/mik32.h>
#include <zephyr/drivers/pinctrl.h>

#include <soc/mikron/mik32/soc.h>
#include <hal/mik32/peripherals/Include/mik32_hal_gpio.h>

/** Utility macro that expands to the GPIO port address if it exists */
#define MIK32_PORT_ADDR_OR_NONE(nodelabel)				\
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		\
		   (DT_REG_ADDR(DT_NODELABEL(nodelabel)),), ())

/** Utility macro that expands to the GPIO clock id if it exists */
#define MIK32_PORT_CLOCK_ID_OR_NONE(nodelabel)				\
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		\
		   (DT_CLOCKS_CELL(DT_NODELABEL(nodelabel), id),), ())

/* BUILD_ASSERT((MIK32_PUPD_NONE == GPIO_PUPD_NONE) && */
/* 	     (MIK32_PUPD_PULLUP == GPIO_PUPD_PULLUP) && */
/* 	     (MIK32_PUPD_PULLDOWN == GPIO_PUPD_PULLDOWN), */
/* 	     "pinctrl pull-up/down definitions != HAL definitions"); */

/* BUILD_ASSERT((MIK32_OTYPE_PP == GPIO_OTYPE_PP) && */
/* 	     (MIK32_OTYPE_OD == GPIO_OTYPE_OD), */
/* 	     "pinctrl output type definitions != HAL definitions"); */

/** Utility macro that expands to the GPIO port address if it exists */
#define MIK32_PORT_ADDR_OR_NONE(nodelabel)				       \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		       \
		   (DT_REG_ADDR(DT_NODELABEL(nodelabel)),), ())

/** Utility macro that expands to the GPIO clock id if it exists */
#define MIK32_PORT_CLOCK_ID_OR_NONE(nodelabel)				       \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),		       \
		   (DT_CLOCKS_CELL(DT_NODELABEL(nodelabel), id),), ())

/** MIK32 port addresses */
static const uint32_t mik32_port_addrs[] = {
	MIK32_PORT_ADDR_OR_NONE(gpioa)
	MIK32_PORT_ADDR_OR_NONE(gpiob)
	MIK32_PORT_ADDR_OR_NONE(gpioc)
};

/** MIK32 port clock identifiers */
static const uint16_t mik32_port_clkids[] = {
	MIK32_PORT_CLOCK_ID_OR_NONE(gpioa)
	MIK32_PORT_CLOCK_ID_OR_NONE(gpiob)
	MIK32_PORT_CLOCK_ID_OR_NONE(gpioc)
};

/**
 * @brief Configure a pin.
 *
 * @param pin The pin to configure.
 */
static void pinctrl_configure_pin(pinctrl_soc_pin_t pin)
{
	uint8_t port_idx;
	uint32_t port, pin_num, af;
	uint16_t clkid;
	uint16_t strength;

	port_idx = MIK32_PORT_GET(pin);
	__ASSERT_NO_MSG(port_idx < ARRAY_SIZE(mik32_port_addrs));

	clkid = mik32_port_clkids[port_idx];
	port = mik32_port_addrs[port_idx];
	pin_num = BIT(MIK32_PIN_GET(pin));
	af = MIK32_AF_GET(pin);
	strength = MIK32_STRENGTH_GET(pin);

	(void)clock_control_on(MIK32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clkid);


	uint32_t pupd;
	uint32_t ds;
	uint32_t cfg;
	uint32_t dirin;
	uint32_t dirout;

	pupd = MIK32_PAD_PUPD(port);
	cfg = MIK32_PAD_CFG(port);
	ds = MIK32_PAD_DS(port);
	dirin = MIK32_GPIO_DIRIN(port);
	dirout = MIK32_GPIO_DIROUT(port);

	if (af != MIK32_ANALOG) {
		// clear mode
		cfg &= ~(0x3 << (pin_num * 2));
		// set af
		cfg |= (af << (pin_num * 2));
	} else {
		// set mode ANALOG
		cfg |= (0x3 << (pin_num * 2));
		// set direction input
		dirin |= (1 << pin_num);
	}

	switch (MIK32_PUPD_GET(pin)) {
	case MIK32_PUPD_PULLUP:
		pupd &= ~(0x3 << (pin_num * 2));
		pupd |= (0x1 << (pin_num * 2));
		break;
	case MIK32_PUPD_PULLDOWN:
		pupd &= ~(0x3 << (pin_num * 2));
		pupd |= (0x2 << (pin_num * 2));
		break;
	case MIK32_PUPD_NONE:
	default:
		pupd &= ~(0x3 << (pin_num * 2));
		break;
	}

	switch (MIK32_STRENGTH_GET(pin)) {
	case MIK32_STRENGTH_4MA:
		ds &= ~(0x3 << (pin_num * 2));
		ds |= (0x1 << (pin_num * 2));
		break;
	case MIK32_STRENGTH_8MA:
		ds &= ~(0x3 << (pin_num * 2));
		ds |= (0x2 << (pin_num * 2));
		break;
	case MIK32_STRENGTH_2MA:
	default:
		ds &= ~(0x3 << (pin_num * 2));
		break;
	}

	MIK32_PAD_PUPD(port) = pupd;
	MIK32_PAD_CFG(port) = cfg;
	MIK32_PAD_DS(port) = cfg;
	MIK32_GPIO_DIRIN(port) = dirin;
	MIK32_GPIO_DIROUT(port) = dirout;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins[i]);
	}

	return 0;
}
