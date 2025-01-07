/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <soc.h>

#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mikron_spifi, CONFIG_MEMC_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(mikron_spifi)
#define DT_DRV_COMPAT mikron_spifi
#else
#error "No compatible SPIFI devicetree node found"
#endif

struct memc_mikron_spifi_config {
	const struct pinctrl_dev_config *pcfg;
	SPIFI_CONFIG_TypeDef *spifi_instance;
};

static int memc_mikron_spifi_init(const struct device *dev)
{
	const struct memc_mikron_spifi_config *config = dev->config;

	/* int r; */
	/* const struct device *clk; */

	/* /\* configure pinmux *\/ */
	/* r = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT); */
	/* if (r < 0) { */
	/* 	LOG_ERR("SPIFI pinctrl setup failed (%d)", r); */
	/* 	return r; */
	/* } */
	return 0;
}

//PINCTRL_DT_INST_DEFINE(0);

static const struct memc_mikron_spifi_config config = {
	.spifi_instance = SPIFI_CONFIG,
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0, memc_mikron_spifi_init, NULL, NULL,
		      &config, POST_KERNEL, CONFIG_MEMC_INIT_PRIORITY, NULL);
