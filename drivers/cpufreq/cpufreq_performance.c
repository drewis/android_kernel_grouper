/*
 *  linux/drivers/cpufreq/cpufreq_performance.c
 *
 *  Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/init.h>

#include <linux/pm_qos_params.h>
static struct pm_qos_request_list perf_core_lock_min;
static struct pm_qos_request_list perf_core_lock_max;


static int cpufreq_governor_performance(struct cpufreq_policy *policy,
					unsigned int event)
{
	switch (event) {
	case CPUFREQ_GOV_START:
		pm_qos_update_request(&perf_core_lock_min,
			(s32)4);
		pm_qos_update_request(&perf_core_lock_max,
			(s32)4);
		break;
	case CPUFREQ_GOV_STOP:
		pm_qos_update_request(&perf_core_lock_min,
			(s32)PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);
		pm_qos_update_request(&perf_core_lock_max,
			(s32)PM_QOS_MAX_ONLINE_CPUS_DEFAULT_VALUE);
		break;
	case CPUFREQ_GOV_LIMITS:
		pr_debug("setting to %u kHz because of event %u\n",
						policy->max, event);
		__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_CPU_FREQ_GOV_PERFORMANCE_MODULE
static
#endif
struct cpufreq_governor cpufreq_gov_performance = {
	.name		= "performance",
	.governor	= cpufreq_governor_performance,
	.owner		= THIS_MODULE,
};


static int __init cpufreq_gov_performance_init(void)
{
	pm_qos_add_request(&perf_core_lock_min, PM_QOS_MIN_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&perf_core_lock_max, PM_QOS_MAX_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	return cpufreq_register_governor(&cpufreq_gov_performance);
}


static void __exit cpufreq_gov_performance_exit(void)
{
	pm_qos_remove_request(&perf_core_lock_min);
	pm_qos_remove_request(&perf_core_lock_max);
	cpufreq_unregister_governor(&cpufreq_gov_performance);
}


MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>");
MODULE_DESCRIPTION("CPUfreq policy governor 'performance'");
MODULE_LICENSE("GPL");

fs_initcall(cpufreq_gov_performance_init);
module_exit(cpufreq_gov_performance_exit);
