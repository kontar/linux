/*
 * KEYSTONE Remote Processor driver
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Contact: Sajesh Kumar Saran <sajesh@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/klist.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include "remoteproc_internal.h"


#define ADDR_MAP_TABLE_LENGTH 3
#define L2_RAM_ADDR_MAP_INDEX 0
#define MSMC_ADDR_MAP_INDEX 1
#define DDR3_ADDR_MAP_INDEX 2

/**
 * struct local_addr_map - keystone remote processor local address map entry
 */
struct local_addr_map {
	phys_addr_t global_addr;
	phys_addr_t local_addr;
	int length;
};

/**
 * struct keystone_rproc - keystone remote processor state
 * @rproc: rproc handle
 */
struct keystone_rproc {
	struct rproc	*rproc;
	void __iomem	*ipc_int_gen;
	void __iomem	*boot_magic_addr;
	struct local_addr_map addr_map_table[ADDR_MAP_TABLE_LENGTH];
};

static int keystone_create_carveout(struct rproc *rproc, int index)
{
	struct rproc_mem_entry *carveout;
	struct keystone_rproc *kproc = rproc->priv;

	carveout = devm_kzalloc(rproc->dev, sizeof(struct rproc_mem_entry),
			GFP_KERNEL);
	if (!carveout) {
		dev_err(rproc->dev, "devm_kzalloc carveout failed\n");
		return -ENOMEM;
	}

	carveout->da = kproc->addr_map_table[index].global_addr;
	carveout->len = kproc->addr_map_table[index].length;
	carveout->va = devm_ioremap_nocache(rproc->dev, carveout->da,
			carveout->len);
	if (!carveout->va) {
		dev_err(rproc->dev, "failed to ioremap 0x%x\n",
				(int) carveout->va);
		return -EBUSY;
	}

	list_add_tail(&carveout->node, &rproc->carveouts);

	return 0;
}

/* Send an IPC interrupt to the remote DSP core */
static void keystone_rproc_kick(struct rproc *rproc, int vqid)
{
	struct keystone_rproc *kproc = (struct keystone_rproc *) rproc->priv;

	__raw_writel(1, kproc->ipc_int_gen);
}

/*
 * In Keystone the carveout stores the virtual to phycical mappings for the
 * DSP memory sections. The DSP memory sections needs to lie in this sections
 * for kernel to copy.
 */
static void *keystone_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct keystone_rproc *kproc = (struct keystone_rproc *) rproc->priv;
	struct rproc_mem_entry *carveout;
	void *ptr = NULL;
	int offset, i;
	struct local_addr_map *addr_map = &kproc->addr_map_table[0];

	for (i = 0; i < ADDR_MAP_TABLE_LENGTH; i++) {
		if (addr_map->local_addr <= da &&
		    addr_map->local_addr + addr_map->length > da) {
			offset = da - addr_map->local_addr;
			da = addr_map->global_addr + offset;
			break;
		}
		addr_map++;
	}

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		offset = da - carveout->da;

		/* Check if da is in the memory range */
		if (carveout->da > da || carveout->da + carveout->len < da)
			continue;

		/* Check if the section length fits to the carveout segment */
		if (da + len > carveout->da + carveout->len)
			break;

		ptr = carveout->va + offset;

		break;
	}

	return ptr;
}

/*
 * Power up the remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int keystone_rproc_start(struct rproc *rproc)
{
	struct keystone_rproc *kproc = (struct keystone_rproc *) rproc->priv;

	int ret = 0;
	__raw_writel(rproc->bootaddr, kproc->boot_magic_addr);

	keystone_rproc_kick(rproc, -1);

	return ret;
}

static int keystone_rproc_stop(struct rproc *rproc)
{
	struct keystone_rproc *kproc = (struct keystone_rproc *) rproc->priv;

	if (kproc->ipc_int_gen)
		devm_iounmap(rproc->dev, kproc->ipc_int_gen);

	if (kproc->boot_magic_addr)
		devm_iounmap(rproc->dev, kproc->boot_magic_addr);

	return 0;
}

static int keystone_rproc_init_proc(struct rproc *rproc)
{
	int ret;

	ret = keystone_create_carveout(rproc, L2_RAM_ADDR_MAP_INDEX);

	if (ret < 0)
		return ret;

	ret = keystone_create_carveout(rproc, MSMC_ADDR_MAP_INDEX);
	if (ret < 0)
		return ret;

	ret = keystone_create_carveout(rproc, DDR3_ADDR_MAP_INDEX);
	if (ret < 0)
		return ret;

	return 0;
}

static void keystone_rproc_shutdown_proc(struct rproc *rproc)
{
	struct rproc_mem_entry *carveout, *tmp;
	list_for_each_entry_safe(carveout, tmp, &rproc->carveouts, node) {
		if (carveout->va)
			iounmap(carveout->va);
		devm_kfree(rproc->dev, carveout);
	}
}

static struct rproc_ops keystone_rproc_ops = {
	.init		= keystone_rproc_init_proc,
	.shutdown	= keystone_rproc_shutdown_proc,
	.start		= keystone_rproc_start,
	.stop		= keystone_rproc_stop,
	.kick		= keystone_rproc_kick,
	.da_to_va	= keystone_rproc_da_to_va,
};

static int __devinit keystone_rproc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct keystone_rproc *kproc;
	struct rproc *rproc;
	const char *core;
	const char *pfirmware;
	int ret;


	if (of_property_read_string(np, "core", &core)) {
		dev_err(&pdev->dev, "No name in dt bindings\n");
		return -ENODEV;
	}

	if (of_property_read_string(np, "firmware", &pfirmware)) {
		dev_err(&pdev->dev, "No firmware in dt bindings\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(&pdev->dev, core, &keystone_rproc_ops,
		pfirmware, sizeof(*kproc));
	if (!rproc) {
		dev_err(&pdev->dev, "rpoc_alloc failure\n");
		return -ENOMEM;
	}

	kproc = rproc->priv;
	kproc->rproc = rproc;

	if (of_property_read_u32_array(np, "addr_map",
	    (u32 *)kproc->addr_map_table, 9)) {
		dev_err(&pdev->dev, "No addr_map array  in dt bindings\n");
		return -ENODEV;
	}

	kproc->boot_magic_addr = of_devm_iomap(rproc->dev, 0);
	if (!kproc->boot_magic_addr) {
		dev_err(rproc->dev, "failed to iomap DSP boot register\n");
		return -ENOMEM;
	}

	kproc->ipc_int_gen = of_devm_iomap(rproc->dev, 1);
	if (!kproc->boot_magic_addr) {
		dev_err(rproc->dev, "failed to iomap DSP ipc interrupt register\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, rproc);

	ret = rproc_register(rproc);
	if (ret) {
		dev_err(rproc->dev, "rproc registration failure\n");
		goto free_rproc;
	}

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int __devexit keystone_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	return rproc_unregister(rproc);
}




/* Match table for of_platform binding */
static struct of_device_id __devinitdata keystone_rproc_of_match[] = {
	{ .compatible = "ti,keystone-rproc", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_rproc_of_match);


static struct platform_driver keystone_rproc_driver = {
	.probe = keystone_rproc_probe,
	.remove = __devexit_p(keystone_rproc_remove),
	.driver = {
		.name = "keystone-rproc",
		.owner = THIS_MODULE,
		.of_match_table = keystone_rproc_of_match,
	},
};

/* most of the below will go when module_platform_driver is merged */
static int __init keystone_rproc_init(void)
{
	return platform_driver_register(&keystone_rproc_driver);
}
module_init(keystone_rproc_init);

static void __exit keystone_rproc_exit(void)
{
	platform_driver_unregister(&keystone_rproc_driver);
}
module_exit(keystone_rproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Keystone Remote Processor control driver");
MODULE_AUTHOR("Sajesh Kumar Saran <sajesh@ti.com>");
