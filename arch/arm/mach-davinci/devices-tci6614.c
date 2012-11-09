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
#include <mach/edma.h>
#include <mach/tci6614.h>
#include <mach/time.h>

/* Base addresses for on-chip devices */
#define TCI6614_TPCC_BASE			0x01c00000
#define TCI6614_TPTC0_BASE			0x01c10000
#define TCI6614_TPTC1_BASE			0x01c10400
#define TCI6614_WDOG_BASE			0x02280000
#define TCI6614_I2C_BASE			0x02530000
#define TCI6614_SPI_BASE			0x20BF0000
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

static struct resource tci6614_i2c_resources[] = {
	{
		.start	= TCI6614_I2C_BASE,
		.end	= TCI6614_I2C_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TCI6614_I2CINT,
		.end	= IRQ_TCI6614_I2CINT,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tci6614_i2c_device = {
	.name		= "i2c_davinci",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(tci6614_i2c_resources),
	.resource	= tci6614_i2c_resources,
};

static int __init tci6614_register_i2c(struct davinci_i2c_platform_data *pdata)
{
	struct platform_device *pdev = &tci6614_i2c_device;

	pdev->dev.platform_data = pdata;
	return platform_device_register(pdev);
}

static struct resource spi_resources[] = {
	[0] = {
		.start	= TCI6614_SPI_BASE,
		.end	= TCI6614_SPI_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_TCI6614_SPIINT0,
		.end	= IRQ_TCI6614_SPIINT0,
		.flags	= IORESOURCE_IRQ,
	},
/* Not using DMA */
};

static struct platform_device tci6614_spi_device = {
	.name		= "spi_davinci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(spi_resources),
	.resource	= spi_resources,
};

static int __init tci6614_register_spi(struct davinci_spi_platform_data *pdata)
{
	struct platform_device *pdev = &tci6614_spi_device;

	pdev->dev.platform_data = pdata;
	return platform_device_register(pdev);
}

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

	if (info->i2c_config)
		tci6614_register_i2c(info->i2c_config);

	if (info->spi_config)
		tci6614_register_spi(info->spi_config);
}
