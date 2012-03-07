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


/**
 * struct local_addr_map - keystone remote processor local address map entry
 */
struct local_addr_map {
	phys_addr_t global_addr;
	phys_addr_t local_addr;
	int length;
	struct list_head node;
};

/**
 * struct keystone_rproc - keystone remote processor state
 * @rproc: rproc handle
 */
struct keystone_rproc {
	struct rproc	*rproc;
	void __iomem	*ipc_int_gen;
	void __iomem	*boot_magic_addr;
	struct list_head addr_map;
};

/* Send an IPC interrupt to the remote DSP core */
static void keystone_rproc_kick(struct rproc *rproc, int vqid)
{
	struct keystone_rproc *kproc = (struct keystone_rproc *) rproc->priv;

	__raw_writel(1, kproc->ipc_int_gen);
}

static inline u64 keystone_rproc_get_global_addr(struct rproc *rproc, u64 da,
						 int length)
{
	struct keystone_rproc *kproc = rproc->priv;
	int offset;
	struct local_addr_map *padd_map;

	list_for_each_entry(padd_map, &kproc->addr_map, node) {
		if (padd_map->local_addr <= da &&
			padd_map->local_addr + padd_map->length > da) {
			offset = da - padd_map->local_addr;
			if (offset + length > padd_map->length)
				return 0;
			return padd_map->global_addr + offset;
		}
	}
	return 0;
}

static int keystone_rproc_load_seg(struct rproc *rproc, const u8 *elf_data)
{
	struct device *dev = rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	void *va;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 filesz = phdr->p_filesz;
		u32 memsz = phdr->p_memsz;
		u32 da = phdr->p_paddr;
		u32 gda;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
			phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		gda = keystone_rproc_get_global_addr(rproc, da, memsz);
		if (!gda) {
			dev_err(dev, "bad phdr da 0x%x len %d\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		va = devm_ioremap_nocache(rproc->dev, gda, memsz);
		if (!va) {
			dev_err(dev, "failed to ioremap 0x%x: %d\n",
				da, (int) va);
			return -EBUSY;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			memcpy(va, elf_data + phdr->p_offset, filesz);

		/*
		 * Zero out remaining memory for this segment.
		 *
		 * This isn't strictly required since dma_alloc_coherent already
		 * did this for us. albeit harmless, we may consider removing
		 * this.
		 */
		if (memsz > filesz)
			memset(va + filesz, 0, memsz - filesz);

		devm_iounmap(rproc->dev, va);
	}

	return ret;
}

static void *keystone_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	u32 gda;
	void *va;

	gda = keystone_rproc_get_global_addr(rproc, da, len);
	if (!gda) {
		dev_err(rproc->dev, "bad phdr da 0x%x len %d\n", (int)da, len);
		return 0;
	}

	va = devm_ioremap_nocache(rproc->dev, gda, len);
	if (!va) {
		dev_err(rproc->dev, "failed to ioremap 0x%x: %d\n",
			(int)da, (int) va);
		return 0;
	}

	return va;
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
	return 0;
}

static void keystone_rproc_shutdown_proc(struct rproc *rproc)
{

}

static struct rproc_ops keystone_rproc_ops = {
	.init		= keystone_rproc_init_proc,
	.shutdown	= keystone_rproc_shutdown_proc,
	.start		= keystone_rproc_start,
	.stop		= keystone_rproc_stop,
	.kick		= keystone_rproc_kick,
	.load_seg	= keystone_rproc_load_seg,
	.da_to_va	= keystone_rproc_da_to_va,
};

static int __devinit keystone_rproc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct keystone_rproc *kproc;
	struct rproc *rproc;
	const char *core;
	const char *pfirmware;
	int ret, len, i, num_entries;
	u32 *paddr_map;
	struct local_addr_map *pmapentry;

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

	INIT_LIST_HEAD(&kproc->addr_map);

	if (!of_get_property(np, "addr-map", &len)) {
		dev_err(&pdev->dev, "No addr-map array in dt bindings\n");
		return -ENODEV;
	}

	paddr_map = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!paddr_map) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	len = len / sizeof(u32);
	if ((len % 3) != 0) {
		dev_err(&pdev->dev, "invalid address map in dt binding\n");
		return -EINVAL;
	}
	num_entries = len / 3;
	if (of_property_read_u32_array(np, "addr-map",
				       (u32 *)paddr_map, len)) {
		dev_err(&pdev->dev, "No addr-map array  in dt bindings\n");
		return -ENODEV;
	}

	for (i = 0; i < num_entries; i++) {
		pmapentry  = devm_kzalloc(&pdev->dev,
						sizeof(*pmapentry), GFP_KERNEL);
		if (!pmapentry) {
			dev_err(rproc->dev, "devm_kzalloc mapping failed\n");
			return -ENOMEM;
		}
		pmapentry->global_addr = *paddr_map++;
		pmapentry->local_addr = *paddr_map++;
		pmapentry->length = *paddr_map++;
		list_add_tail(&pmapentry->node, &kproc->addr_map);
	}

	kproc->boot_magic_addr = of_devm_iomap(rproc->dev, 0);
	if (!kproc->boot_magic_addr) {
		dev_err(rproc->dev, "failed to iomap DSP boot register\n");
		return -ENOMEM;
	}

	kproc->ipc_int_gen = of_devm_iomap(rproc->dev, 1);
	if (!kproc->ipc_int_gen) {
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
