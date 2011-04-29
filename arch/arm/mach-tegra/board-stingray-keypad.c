/*
 * arch/arm/mach-tegra/board-stingray-keypad.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "board-stingray.h"
#include "gpio-names.h"

static struct gpio_event_direct_entry stingray_keypad_keys_map[] = {
	{
		.code	= KEY_VOLUMEUP,
		.gpio	= TEGRA_GPIO_PR0,
	},
	{
		.code	= KEY_VOLUMEDOWN,
		.gpio	= TEGRA_GPIO_PR1,
	},
};

static struct gpio_event_input_info stingray_keypad_keys_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = stingray_keypad_keys_map,
	.keymap_size = ARRAY_SIZE(stingray_keypad_keys_map),
	.info.no_suspend = false,
};

static struct gpio_event_info *stingray_keypad_info[] = {
	&stingray_keypad_keys_info.info,
};

static struct gpio_event_platform_data stingray_keypad_data = {
	.name = "stingray-keypad",
	.info = stingray_keypad_info,
	.info_count = ARRAY_SIZE(stingray_keypad_info)
};

static struct platform_device stingray_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &stingray_keypad_data,
	},
};

int stingray_log_reset(void)
{
	pr_warn("Hard reset buttons pushed\n");
	return 0;
}

static struct keyreset_platform_data stingray_reset_keys_pdata = {
	.reset_fn = stingray_log_reset,
	.keys_down = {
		KEY_END,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device stingray_keyreset_device = {
	.name	= KEYRESET_NAME,
	.dev	= {
		.platform_data = &stingray_reset_keys_pdata,
	},
};


int __init stingray_keypad_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PR0);
	tegra_gpio_enable(TEGRA_GPIO_PR1);
	tegra_gpio_enable(TEGRA_GPIO_PQ0);
	gpio_request(TEGRA_GPIO_PQ0, "keypad-col");
	gpio_direction_output(TEGRA_GPIO_PQ0, 0);
	platform_device_register(&stingray_keyreset_device);
	return platform_device_register(&stingray_keypad_device);
}
