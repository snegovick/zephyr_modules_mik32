/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_eeprom_8k_blk

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <hal/mik32/peripherals/Include/mik32_hal_eeprom.h>

LOG_MODULE_REGISTER(eeprom_8k_blk, CONFIG_EEPROM_LOG_LEVEL);

struct eeprom_8k_blk_config {
	size_t size;
};

struct eeprom_8k_blk_data {
	struct k_mutex lock_mtx;
  HAL_EEPROM_HandleTypeDef *eeprom;
};

static int eeprom_8k_blk_read(const struct device *dev, off_t offset,
				void *buf,
				size_t len)
{
	const struct eeprom_8k_blk_config *config = dev->config;
	struct eeprom_8k_blk_data * const data = dev->data;

	if (len == 0) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock_mtx, K_FOREVER);
	//pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	/* EEPROM HW READ */
  HAL_EEPROM_Read(data->eeprom, offset, buf, len, HAL_EEPROM_TIMEOUT);
	//pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	k_mutex_unlock(&data->lock_mtx);

	return 0;
}

static int eeprom_8k_blk_write(const struct device *dev, off_t offset,
				const void *buf, size_t len)
{
	const struct eeprom_8k_blk_config *config = dev->config;
	struct eeprom_8k_blk_data * const data = dev->data;

	if (len == 0) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock_mtx, K_FOREVER);
	//pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

  HAL_EEPROM_Write(data->eeprom, offset, buf, len, HAL_EEPROM_WRITE_ALL, HAL_EEPROM_TIMEOUT);
	//pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	k_mutex_unlock(&data->lock_mtx);

	return 0;
}

static size_t eeprom_8k_blk_size(const struct device *dev)
{
	const struct eeprom_8k_blk_config *config = dev->config;

	return config->size;
}

static int eeprom_8k_blk_init(const struct device *dev)
{
	const struct eeprom_8k_blk_config *config = dev->config;
	struct eeprom_8k_blk_data * const data = dev->data;

	k_mutex_init(&data->lock_mtx);

  data->eeprom = EEPROM_REGS;
  HAL_EEPROM_Init(data->eeprom);

	return 0;
}

static DEVICE_API(eeprom, eeprom_8k_blk_api) = {
	.read = eeprom_8k_blk_read,
	.write = eeprom_8k_blk_write,
	.size = eeprom_8k_blk_size,
};

static const struct eeprom_8k_blk_config eeprom_config = {
	.size = DT_INST_PROP(0, size),
};

static struct eeprom_8k_blk_data eeprom_data;

DEVICE_DT_INST_DEFINE(0, &eeprom_8k_blk_init, PM_DEVICE_DT_INST_GET(0), &eeprom_data,
		    &eeprom_config, POST_KERNEL,
		    CONFIG_EEPROM_INIT_PRIORITY, &eeprom_8k_blk_api);
