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
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/radio_ctrl/radio_class.h>
#include <linux/radio_ctrl/wrigley_ctrl.h>

#define GPIO_MAX_NAME 30

/* How long, in jiffies, does it take for the modem to restart. */
#define RESTART_DELAY        (2*HZ) /* jiffies */
#define PWRUP_DELAY_MS       100
#define PWRUP_FLASH_DELAY_MS 2000
#define PWRDN_DELAY_HRD_MS   1000   /* hard power-off time */
#define PWRDN_DELAY_GRC_MS   25000  /* graceful shutdown time */
#define PWRUP_DELAY_OS_MS    2000   /* time until cards os is running */

static const char *wrigley_status_str[] = {
	[WRIGLEY_STATUS_UNDEFINED] = "undefined",
	[WRIGLEY_STATUS_OFF] = "off",
	[WRIGLEY_STATUS_PWRUP] = "powering_up",
	[WRIGLEY_STATUS_NORMAL] = "normal",
	[WRIGLEY_STATUS_FLASH] = "flash",
	[WRIGLEY_STATUS_RESETTING] = "resetting",
	[WRIGLEY_STATUS_PWRDN] = "powering_down",
};

struct wrigley_info {
	unsigned int disable_gpio;
	char disable_name[GPIO_MAX_NAME];

	unsigned int flash_gpio;
	char flash_name[GPIO_MAX_NAME];

	struct completion pwrup_complete;
	struct completion pwrdn_complete;
	unsigned int reset_gpio;
	char reset_name[GPIO_MAX_NAME];
	struct delayed_work work;
	/* optional callback for cases where modem cannot recover */
	void (*handle_radio_off)(enum wrigley_status);

	bool boot_flash;
	bool allow_reboot;
	enum wrigley_status status;

	struct radio_dev rdev;
	struct mutex sysfs_lock;
};

#define wrigley_set_status(info, new_status) \
	do { \
		pr_debug("%s: status: %s (%d) -> %s (%d) \n", __func__, \
			wrigley_status_str[info->status], info->status, \
			wrigley_status_str[new_status], new_status); \
		info->status = new_status; \
	} while (0)

static inline enum wrigley_status wrigley_get_on_status(
		struct wrigley_info *info) {
	return info->boot_flash ?  WRIGLEY_STATUS_FLASH : WRIGLEY_STATUS_NORMAL;
}

static ssize_t wrigley_status_show(struct radio_dev *rdev, char *buff)
{
	struct wrigley_info *info =
		container_of(rdev, struct wrigley_info, rdev);

	pr_debug("%s: wrigley_status = %s (%d) reset = %d\n", __func__,
		wrigley_status_str[info->status], info->status,
		gpio_get_value(info->reset_gpio));
	if (info->status > WRIGLEY_STATUS_MAX)
		wrigley_set_status(info, WRIGLEY_STATUS_UNDEFINED);

	return snprintf(buff, RADIO_STATUS_MAX_LENGTH, "%s\n",
		wrigley_status_str[info->status]);
}

/* hard poweroff */
static ssize_t wrigley_do_poweroff(struct wrigley_info *info)
{
	pr_info("%s: hard poweroff\n", __func__);
	INIT_COMPLETION(info->pwrdn_complete);
	wrigley_set_status(info, WRIGLEY_STATUS_PWRDN);
	disable_irq(gpio_to_irq(info->reset_gpio));
	gpio_direction_output(info->disable_gpio, 0);
	gpio_direction_output(info->flash_gpio, 1);
	gpio_direction_output(info->reset_gpio, 0);
	msleep(10);
	gpio_set_value(info->reset_gpio, 1);
	enable_irq(gpio_to_irq(info->reset_gpio));
	gpio_direction_input(info->reset_gpio);
	gpio_set_value(info->flash_gpio, 0);

	if (wait_for_completion_timeout(&info->pwrdn_complete,
		msecs_to_jiffies(PWRDN_DELAY_HRD_MS)) == 0) {
		pr_err("%s: timeout powering off wrigley\n", __func__);
		if (gpio_get_value(info->reset_gpio) != 0)
			return -1;
	}

	wrigley_set_status(info, WRIGLEY_STATUS_OFF);
	pr_debug("%s: wrigley is off\n", __func__);
	return 0;
}

/* graceful shutdown */
static ssize_t wrigley_do_shutdown(struct wrigley_info *info)
{
	pr_info("%s: graceful shutdown\n", __func__);
	INIT_COMPLETION(info->pwrdn_complete);
	if (info->status == WRIGLEY_STATUS_OFF) {
		pr_err("%s: already off\n", __func__);
		return -1;
	}

	gpio_direction_output(info->flash_gpio, 0);
	wrigley_set_status(info, WRIGLEY_STATUS_PWRDN);
	gpio_direction_output(info->disable_gpio, 0);

	if (wait_for_completion_timeout(&info->pwrdn_complete,
		msecs_to_jiffies(PWRDN_DELAY_GRC_MS)) == 0) {
		pr_err("%s: timeout shutting down wrigley\n", __func__);
		return wrigley_do_poweroff(info);
	}

	pr_debug("%s: wrigley is off\n", __func__);
	return 0;
}

static ssize_t wrigley_do_powerup(struct wrigley_info *info)
{
	enum wrigley_status status = info->status;

	pr_info("%s: power-up\n", __func__);
	if (status == WRIGLEY_STATUS_NORMAL || status == WRIGLEY_STATUS_FLASH) {
		pr_err("%s: already on\n", __func__);
		return -1;
	}

	/* power on in normal or flash mode */
	if (info->boot_flash)
		gpio_direction_output(info->flash_gpio, 1);
	else
		gpio_direction_output(info->flash_gpio, 0);

	/* set disable high to actually power on the card */
	pr_debug("%s: set disable high\n", __func__);
	gpio_direction_output(info->disable_gpio, 1);
	wrigley_set_status(info, WRIGLEY_STATUS_PWRUP);

	if (info->boot_flash) {
		/* The reset gpio is disconnected when the flash gpio is set.
		 * Therefore the flash gpio must be returned to 0 after giving
		 * the card time to see it was asserted. */
		msleep(PWRUP_FLASH_DELAY_MS);
		gpio_direction_output(info->flash_gpio, 0);
	}

	if (wait_for_completion_timeout(&info->pwrup_complete,
		msecs_to_jiffies(PWRUP_DELAY_MS)) == 0) {
		if (gpio_get_value(info->reset_gpio) == 0) {
			pr_err("%s: timeout starting wrigley\n", __func__);
			return -1;
		}
	}
	/* The reset line shows that the hardware has turned on.
	 * Delay here so that the device's os has a chance to start
	 * running before returning to caller */
	msleep(PWRUP_DELAY_OS_MS);

	pr_debug("%s: started wrigley in %s mode\n",
		__func__, info->boot_flash ? "flash" : "normal" );

	return 0;
}

static ssize_t wrigley_set_flash_mode(struct wrigley_info *info, bool enable)
{
	pr_debug("%s: set boot state to %d\n", __func__, enable);
	info->boot_flash = enable;
	return 0;
}

/* primary interface from sysfs driver */
static ssize_t wrigley_command(struct radio_dev *rdev, char *cmd)
{
	int status;
	struct wrigley_info *info =
		container_of(rdev, struct wrigley_info, rdev);

	pr_info("%s: user command = %s\n", __func__, cmd);
	mutex_lock(&info->sysfs_lock);
	if (strcmp(cmd, "shutdown") == 0) {
		status = wrigley_do_shutdown(info);
	} else if (strcmp(cmd, "poweroff") == 0) {
		status = wrigley_do_poweroff(info);
	} else if (strcmp(cmd, "powerup") == 0) {
		status = wrigley_do_powerup(info);
	} else if (strcmp(cmd, "bootmode_normal") == 0) {
		status = wrigley_set_flash_mode(info, 0);
	} else if (strcmp(cmd, "bootmode_flash") == 0) {
		status = wrigley_set_flash_mode(info, 1);
	} else if (strcmp(cmd, "allow_reboot_on") == 0) {
		info->allow_reboot = 1;
		status = 0;
	} else if (strcmp(cmd, "allow_reboot_off") == 0) {
		info->allow_reboot = 0;
		status = 0;
	} else {
		pr_err("%s: command %s not supported\n", __func__, cmd);
		status = -EINVAL;
	}
	mutex_unlock(&info->sysfs_lock);

	return status;
}

/* Delayed work procedure to set the device to OFF.  Additionally, some
 * devices may need to detect the device powering off without restarting.
 * This indicates a low battery condition.  If the board has a special
 * purpose function for handling that, invoke it here.
 */
static void wrigley_detect_off(struct work_struct *work)
{
	struct wrigley_info *info =  container_of(work,
		struct wrigley_info, work.work);

	pr_debug("%s: set device off\n", __func__);
	if (info->handle_radio_off)
		info->handle_radio_off(info->status);
	wrigley_set_status(info, WRIGLEY_STATUS_OFF);

	if (info->rdev.dev) {
		pr_debug("%s: sending uevent\n", __func__);
		kobject_uevent(&info->rdev.dev->kobj, KOBJ_CHANGE);
	}
}

/* Notify userspace that the device is changing state */
static irqreturn_t wrigley_reset_fn(int irq, void *data)
{
	struct wrigley_info *info = (struct wrigley_info *) data;

	if (info->rdev.dev) {
		pr_debug("%s: sending uevent\n", __func__);
		kobject_uevent(&info->rdev.dev->kobj, KOBJ_CHANGE);
	}
	return IRQ_HANDLED;
}

/* When the reset line goes low, wrigley is either restarting
 * or resetting.  If the device is resetting, the line will go
 * high within RESTART_DELAY.  If the line stays low, then the device
 * is off.
 */
static irqreturn_t wrigley_reset_isr(int irq, void *data)
{
	struct wrigley_info *info = (struct wrigley_info *) data;

	cancel_delayed_work(&info->work);
	if (gpio_get_value(info->reset_gpio)) {
		pr_debug("%s: rising edge irq (%d)\n", __func__, irq);
		if (info->status == WRIGLEY_STATUS_PWRUP) {
			pr_debug("%s: powerup complete (%d)\n", __func__, irq);
			wrigley_set_status(info, wrigley_get_on_status(info));
			complete(&info->pwrup_complete);
		} else if (info->status == WRIGLEY_STATUS_PWRDN) {
			/* a reset will drive the line high before reenabling
			   the irq.  We can consume this irq */
			pr_debug("%s: powering down - ignore (%d)\n",
				__func__, irq);
			return IRQ_HANDLED;
		} else {
			pr_debug("%s: ascync powerup (%d)\n", __func__, irq);
			wrigley_set_status(info, wrigley_get_on_status(info));
		}
	} else {
		pr_debug("%s: falling edge irq (%d)\n", __func__, irq);
		if (info->status == WRIGLEY_STATUS_PWRDN) {
			pr_debug("%s: shutdown complete\n", __func__);
			wrigley_set_status(info, WRIGLEY_STATUS_OFF);
			complete(&info->pwrdn_complete);
		} else {
			pr_info("%s: LTE data-card powered off.\n",
				__func__);
			/* data-card will restart by default, it is simplier for
			   user space if off means off and return IRQ_HANDLED so
			   user-space will only see the new startup or the full
			   powerdown. In one case, when a secure fuse is being
			   blown on the card, we need to allow reboot */
			if (likely(!info->allow_reboot))
				gpio_direction_output(info->disable_gpio, 0);
			else
				pr_info("%s: allow card to reboot.", __func__);

			wrigley_set_status(info, WRIGLEY_STATUS_RESETTING);
			schedule_delayed_work(&info->work, RESTART_DELAY);
			return IRQ_HANDLED;
		}
	}
	return IRQ_WAKE_THREAD;
}

static int __devinit wrigley_probe(struct platform_device *pdev)
{
	struct wrigley_ctrl_platform_data *pdata = pdev->dev.platform_data;
	struct wrigley_info *info;
	int reset_irq, err = 0;

	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	info = kzalloc(sizeof(struct wrigley_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto err_exit;
	}

	platform_set_drvdata(pdev, info);

	/* setup radio_class device */
	info->rdev.name = dev_name(&pdev->dev);
	info->rdev.status = wrigley_status_show;
	info->rdev.command = wrigley_command;

	mutex_init(&info->sysfs_lock);

	/* disable */
	pr_debug("%s: setup wrigley_disable\n", __func__);
	info->disable_gpio = pdata->gpio_disable;
	snprintf(info->disable_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "disable");
	err = gpio_request(info->disable_gpio, info->disable_name);
	if (err) {
		pr_err("%s: err_disable\n", __func__);
		goto err_disable;
	}
	gpio_export(info->disable_gpio, false);

	/* reset */
	pr_debug("%s: setup wrigley_reset\n", __func__);

	INIT_DELAYED_WORK(&info->work, wrigley_detect_off);
	info->handle_radio_off = pdata->handle_radio_off;
	init_completion(&info->pwrup_complete);
	init_completion(&info->pwrdn_complete);
	info->reset_gpio = pdata->gpio_reset;
	snprintf(info->reset_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "reset");
	err = gpio_request(info->reset_gpio, info->reset_name);
	if (err) {
		pr_err("%s: err requesting reset gpio\n", __func__);
		goto err_reset;
	}
	gpio_direction_input(info->reset_gpio);
	reset_irq = gpio_to_irq(info->reset_gpio);
	err = request_threaded_irq(reset_irq, wrigley_reset_isr,
		wrigley_reset_fn, IRQ_TYPE_EDGE_BOTH, info->reset_name,
		info);
	if (err) {
		pr_err("%s: request irq (%d) %s failed\n",
			__func__, reset_irq, info->reset_name);
		gpio_free(info->reset_gpio);
		goto err_reset;
	}
	gpio_export(info->reset_gpio, false);

	/* force_flash */
	pr_debug("%s: setup wrigley_force_flash\n", __func__);
	info->flash_gpio = pdata->gpio_force_flash;
	snprintf(info->flash_name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "flash");
	err = gpio_request(info->flash_gpio, info->flash_name);
	if (err) {
		pr_err("%s: error requesting flash gpio\n", __func__);
		goto err_flash;
	}
	gpio_export(info->flash_gpio, false);

	/* try to determine the boot up mode of the device */
	info->boot_flash = !!gpio_get_value(info->flash_gpio);
	if (gpio_get_value(info->reset_gpio))
		wrigley_set_status(info, wrigley_get_on_status(info));
	else
		wrigley_set_status(info, WRIGLEY_STATUS_OFF);

	pr_debug("%s: initial status = %s\n", __func__,
		wrigley_status_str[info->status]);

	err = radio_dev_register(&info->rdev);
	if (err) {
		pr_err("%s: failed to register radio device\n", __func__);
		goto err_dev_register;
	}

	return 0;

err_dev_register:
	gpio_free(info->flash_gpio);
err_flash:
	free_irq(reset_irq, info);
	gpio_free(info->reset_gpio);
err_reset:
	gpio_free(info->disable_gpio);
err_disable:
	platform_set_drvdata(pdev, NULL);
	kfree(info);
err_exit:
	return err;
}

static void __devexit wrigley_shutdown(struct platform_device *pdev)
{
	struct wrigley_info *info = platform_get_drvdata(pdev);
	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	(void) wrigley_do_poweroff(info);
}

static int __devexit wrigley_remove(struct platform_device *pdev)
{
	struct wrigley_info *info = platform_get_drvdata(pdev);

	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	cancel_delayed_work_sync(&info->work);
	radio_dev_unregister(&info->rdev);

	gpio_free(info->flash_gpio);
	free_irq(gpio_to_irq(info->reset_gpio), info);
	gpio_free(info->reset_gpio);
	gpio_free(info->disable_gpio);

	platform_set_drvdata(pdev, NULL);
	kfree(info);

	return 0;
}

static struct platform_driver wrigley_driver = {
	.probe = wrigley_probe,
	.remove = __devexit_p(wrigley_remove),
	.shutdown = __devexit_p(wrigley_shutdown),
	.driver = {
		.name = "wrigley",
		.owner = THIS_MODULE,
	},
};

static int __init wrigley_init(void)
{
	pr_info("%s: initializing %s\n", __func__, wrigley_driver.driver.name);
	return platform_driver_register(&wrigley_driver);
}

static void __exit wrigley_exit(void)
{
	pr_info("%s: exiting %s\n", __func__, wrigley_driver.driver.name);
	return platform_driver_unregister(&wrigley_driver);
}

module_init(wrigley_init);
module_exit(wrigley_exit);

MODULE_AUTHOR("Jim Wylder <james.wylder@motorola.com>");
MODULE_DESCRIPTION("Wrigley Modem Control");
MODULE_LICENSE("GPL");
