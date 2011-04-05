/*
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307  USA
 */
#ifndef __LINUX_WRIGLEY_CTRL_H__
#define __LINUX_WRIGLEY_CTRL_H__

enum wrigley_status {
	WRIGLEY_STATUS_UNDEFINED = 0,
	WRIGLEY_STATUS_OFF,
	WRIGLEY_STATUS_PWRUP,
	WRIGLEY_STATUS_NORMAL,
	WRIGLEY_STATUS_FLASH,
	WRIGLEY_STATUS_PWRDN,
	WRIGLEY_STATUS_RESETTING,

	WRIGLEY_STATUS_MAX = WRIGLEY_STATUS_RESETTING,
};

struct wrigley_ctrl_platform_data {
	unsigned int gpio_disable;
	unsigned int gpio_reset;
	unsigned int gpio_force_flash;

	/* optional callback to handle permanent shutdown of the radio */
	void (*handle_radio_off)(enum wrigley_status);
};
#endif /* __LINUX_WRIGLEY_CTRL_H__ */
