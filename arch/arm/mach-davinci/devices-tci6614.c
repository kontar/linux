/*
 * Texas Instruments TCI6614 SoC devices
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/hwspinlock.h>

#include <asm/pmu.h>

#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/tci6614.h>
#include <mach/time.h>

/* Base addresses for on-chip devices */
#define TCI6614_TPCC_BASE			0x01c00000
#define TCI6614_TPTC0_BASE			0x01c10000
#define TCI6614_TPTC1_BASE			0x01c10400
#define TCI6614_WDOG_BASE			0x02280000
#define TCI6614_SEM_BASE			0x02640000

/* TCI6614 specific EDMA3 information */
#define EDMA_TCI6614_NUM_DMACH		64
#define EDMA_TCI6614_NUM_TCC		64
#define EDMA_TCI6614_NUM_PARAMENTRY	128
#define EDMA_TCI6614_NUM_EVQUE		2
#define EDMA_TCI6614_NUM_TC		2
#define EDMA_TCI6614_CHMAP_EXIST	0
#define EDMA_TCI6614_NUM_REGIONS	4
#define TCI6614_DMACH2EVENT_MAP0	0x3C0CE000u
#define TCI6614_DMACH2EVENT_MAP1	0x000FFFFFu


static struct resource wdt_resources[] = {
	{
		.start	= TCI6614_WDOG_BASE,
		.end	= TCI6614_WDOG_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device tci6614_wdt_device = {
	.name		= "watchdog",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(wdt_resources),
	.resource	= wdt_resources,
};

static struct hwspinlock_pdata sem_data = {
	.base_id	= 0,
};

static struct resource sem_resources[] = {
	{
		.start	= TCI6614_SEM_BASE,
		.end	= TCI6614_SEM_BASE + 0x7ff,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device tci6614_sem_device = {
	.name			= "keystone_hwspinlock",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(sem_resources),
	.resource		= sem_resources,
	.dev.platform_data	= &sem_data,
};

static struct resource pmu_resources[] = {
	{
		.start	= IRQ_TCI6614_BENCH,
		.end	= IRQ_TCI6614_BENCH,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tci6614_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= pmu_resources,
	.num_resources	= ARRAY_SIZE(pmu_resources),
};

void __init tci6614_devices_init(struct tci6614_device_info *info)
{
	platform_device_register(&tci6614_wdt_device);
	platform_device_register(&tci6614_sem_device);
	platform_device_register(&tci6614_pmu_device);
}
