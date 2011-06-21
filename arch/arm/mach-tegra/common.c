/*
 * arch/arm/mach-tegra/common.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/memblock.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/system.h>

#include <mach/iomap.h>
#include <mach/dma.h>
#include <mach/powergate.h>
#include <mach/system.h>

#include "apbio.h"
#include "board.h"
#include "clock.h"
#include "fuse.h"

#define MC_SECURITY_CFG2 0x7c

#define AHB_ARBITRATION_PRIORITY_CTRL		0x4
#define   AHB_PRIORITY_WEIGHT(x)	(((x) & 0x7) << 29)
#define   PRIORITY_SELECT_USB BIT(6)
#define   PRIORITY_SELECT_USB2 BIT(18)
#define   PRIORITY_SELECT_USB3 BIT(17)

#define AHB_GIZMO_AHB_MEM		0xc
#define   ENB_FAST_REARBITRATE BIT(2)
#define   DONT_SPLIT_AHB_WR     BIT(7)

#define AHB_GIZMO_USB		0x1c
#define AHB_GIZMO_USB2		0x78
#define AHB_GIZMO_USB3		0x7c
#define   IMMEDIATE	BIT(18)

#define AHB_MEM_PREFETCH_CFG3	0xe0
#define AHB_MEM_PREFETCH_CFG4	0xe4
#define AHB_MEM_PREFETCH_CFG1	0xec
#define AHB_MEM_PREFETCH_CFG2	0xf0
#define   PREFETCH_ENB	BIT(31)
#define   MST_ID(x)	(((x) & 0x1f) << 26)
#define   AHBDMA_MST_ID	MST_ID(5)
#define   USB_MST_ID	MST_ID(6)
#define   USB2_MST_ID	MST_ID(18)
#define   USB3_MST_ID	MST_ID(17)
#define   ADDR_BNDRY(x)	(((x) & 0xf) << 21)
#define   INACTIVITY_TIMEOUT(x)	(((x) & 0xffff) << 0)

unsigned long tegra_bootloader_fb_start;
unsigned long tegra_bootloader_fb_size;
unsigned long tegra_fb_start;
unsigned long tegra_fb_size;
unsigned long tegra_fb2_start;
unsigned long tegra_fb2_size;
unsigned long tegra_carveout_start;
unsigned long tegra_carveout_size;
unsigned long tegra_lp0_vec_start;
unsigned long tegra_lp0_vec_size;
unsigned long tegra_grhost_aperture;

void (*tegra_reset)(char mode, const char *cmd);

static __initdata struct tegra_clk_init_table common_clk_init_table[] = {
	/* set up clocks that should always be on */
	/* name		parent		rate		enabled */
	{ "clk_m",	NULL,		0,		true },
	{ "pll_p",	"clk_m",	216000000,	true },
	{ "pll_p_out1",	"pll_p",	28800000,	true },
	{ "pll_p_out2",	"pll_p",	48000000,	true },
	{ "pll_p_out3",	"pll_p",	72000000,	true },
	{ "pll_m_out1",	"pll_m",	120000000,	true },
	{ "sclk",	"pll_m_out1",	120000000,	true },
	{ "hclk",	"sclk",		120000000,	true },
	{ "pclk",	"hclk",		60000000,	true },
	{ "pll_x",	NULL,		0,		true },
	{ "cpu",	NULL,		0,		true },
	{ "emc",	NULL,		0,		true },
	{ "csite",	NULL,		0,		true },
	{ "timer", 	NULL,		0,		true },
	{ "kfuse",	NULL,		0,		true },
	{ "rtc",	NULL,		0,		true },

	/* set frequencies of some device clocks */
	{ "pll_u",	"clk_m",	480000000,	false },
	{ "sdmmc1",	"pll_p",	48000000,	false},
	{ "sdmmc2",	"pll_p",	48000000,	false},
	{ "sdmmc3",	"pll_p",	48000000,	false},
	{ "sdmmc4",	"pll_p",	48000000,	false},
	{ NULL,		NULL,		0,		0},
};

void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem *p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;

	writel(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x441, p + L2X0_DATA_LATENCY_CTRL);
	writel(2, p + L2X0_PWR_CTRL);

	l2x0_init(p, 0x6C480001, 0x8200c3fe);
#endif

}

static void __init tegra_init_power(void)
{
	tegra_powergate_power_off(TEGRA_POWERGATE_MPE);
	tegra_powergate_power_off(TEGRA_POWERGATE_3D);
}

static inline unsigned long gizmo_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}

static inline void gizmo_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}

static void __init tegra_init_ahb_gizmo_settings(void)
{
	unsigned long val;

	val = gizmo_readl(AHB_GIZMO_AHB_MEM);
	val |= ENB_FAST_REARBITRATE | IMMEDIATE | DONT_SPLIT_AHB_WR;
	gizmo_writel(val, AHB_GIZMO_AHB_MEM);

	val = gizmo_readl(AHB_GIZMO_USB);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB);

	val = gizmo_readl(AHB_GIZMO_USB2);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB2);

	val = gizmo_readl(AHB_GIZMO_USB3);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB3);

	val = gizmo_readl(AHB_ARBITRATION_PRIORITY_CTRL);
	val |= PRIORITY_SELECT_USB | PRIORITY_SELECT_USB2 | PRIORITY_SELECT_USB3
				| AHB_PRIORITY_WEIGHT(7);
	gizmo_writel(val, AHB_ARBITRATION_PRIORITY_CTRL);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG1);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | AHBDMA_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG1);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG2);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG2);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG3);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB3_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG3);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG4);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB2_MST_ID | ADDR_BNDRY(0xc) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG4);
}


static bool console_flushed;

static void tegra_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	pr_emerg("Restarting %s\n", linux_banner);
	if (!try_acquire_console_sem()) {
		release_console_sem();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (try_acquire_console_sem())
		pr_emerg("tegra_restart: Console was locked! Busting\n");
	else
		pr_emerg("tegra_restart: Console was locked!\n");
	release_console_sem();
}

static void tegra_pm_restart(char mode, const char *cmd)
{
	tegra_pm_flush_console();
	arm_machine_restart(mode, cmd);
}

void __init tegra_common_init(void)
{
	arm_pm_restart = tegra_pm_restart;
	tegra_init_fuse();
	tegra_init_clock();
	tegra_clk_init_from_table(common_clk_init_table);
	tegra_init_power();
	tegra_init_cache();
	tegra_dma_init();
	tegra_init_apb_dma();
	tegra_init_ahb_gizmo_settings();
}

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_lp0_vec_arg(char *options)
{
	char *p = options;

	tegra_lp0_vec_size = memparse(p, &p);
	if (*p == '@')
		tegra_lp0_vec_start = memparse(p+1, &p);

	return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);

/*
 * Tegra has a protected aperture that prevents access by most non-CPU
 * memory masters to addresses above the aperture value.  Enabling it
 * secures the CPU's memory from the GPU, except through the GART.
 */
void __init tegra_protected_aperture_init(unsigned long aperture)
{
#ifndef CONFIG_NVMAP_ALLOW_SYSMEM
	void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);
	pr_info("Enabling Tegra protected aperture at 0x%08lx\n", aperture);
	writel(aperture, mc_base + MC_SECURITY_CFG2);
#else
	pr_err("Tegra protected aperture disabled because nvmap is using "
		"system memory\n");
#endif
}

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;

	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);

	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}

	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy_toio(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}

		for (i = 0; i < size; i+= 4)
			writel(readl(from_io + i), to_io + i);

		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);


	tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
	if (memblock_remove(tegra_carveout_start, carveout_size))
		pr_err("Failed to remove carveout %08lx@%08lx from memory "
			"map\n",
			tegra_carveout_start, carveout_size);
	else
		tegra_carveout_size = carveout_size;

	tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
	if (memblock_remove(tegra_fb2_start, fb2_size))
		pr_err("Failed to remove second framebuffer %08lx@%08lx from "
			"memory map\n",
			tegra_fb2_start, fb2_size);
	else
		tegra_fb2_size = fb2_size;

	tegra_fb_start = memblock_end_of_DRAM() - fb_size;
	if (memblock_remove(tegra_fb_start, fb_size))
		pr_err("Failed to remove framebuffer %08lx@%08lx from memory "
			"map\n",
			tegra_fb_start, fb_size);
	else
		tegra_fb_size = fb_size;

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);

	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:         %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_start + tegra_lp0_vec_size - 1,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1,
		tegra_fb_start,
		tegra_fb_start + tegra_fb_size - 1,
		tegra_fb2_start,
		tegra_fb2_start + tegra_fb2_size - 1,
		tegra_carveout_start,
		tegra_carveout_start + tegra_carveout_size - 1);
}
