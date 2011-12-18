/*
 * OMAP Remote Processor driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Mark Grosen <mgrosen@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Hari Kanigeri <h-kanigeri2@ti.com>
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/iommu.h>

#include <plat/mailbox.h>
#include <plat/remoteproc.h>

#include "omap_remoteproc.h"
#include "remoteproc_internal.h"

/**
 * struct omap_rproc - omap remote processor state
 * @mbox: omap mailbox handle
 * @nb: notifier block that will be invoked on inbound mailbox messages
 * @rproc: rproc handle
 * @domain: iommu domain
 */
struct omap_rproc {
	struct omap_mbox *mbox;
	struct notifier_block nb;
	struct rproc *rproc;
	struct iommu_domain *domain;
};

/*
 * This is the IOMMU fault handler we register with the IOMMU API
 * (when relevant; not all remote processors access memory through
 * an IOMMU).
 *
 * IOMMU core will invoke this handler whenever the remote processor
 * will try to access an unmapped device address.
 *
 * Currently this is mostly a stub, but it will be later used to trigger
 * the recovery of the remote processor.
 */
static int omap_rproc_iommu_fault(struct iommu_domain *domain, struct device *dev,
		unsigned long iova, int flags)
{
	dev_err(dev, "iommu fault: da 0x%lx flags 0x%x\n", iova, flags);

	/*
	 * Let the iommu core know we're not really handling this fault;
	 * we just plan to use this as a recovery trigger.
	 */
	return -ENOSYS;
}

static int omap_rproc_enable_iommu(struct rproc *rproc)
{
	struct iommu_domain *domain;
	struct device *dev = rproc->dev;
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	/*
	 * We currently use iommu_present() to decide if an IOMMU
	 * setup is needed.
	 *
	 * This works for simple cases, but will easily fail with
	 * platforms that do have an IOMMU, but not for this specific
	 * rproc.
	 *
	 * This will be easily solved by introducing hw capabilities
	 * that will be set by the remoteproc driver.
	 */
	if (!iommu_present(dev->bus)) {
		dev_err(dev, "iommu not found\n");
		return -ENODEV;
	}

	domain = iommu_domain_alloc(dev->bus);
	if (!domain) {
		dev_err(dev, "can't alloc iommu domain\n");
		return -ENOMEM;
	}

	iommu_set_fault_handler(domain, omap_rproc_iommu_fault);

	ret = iommu_attach_device(domain, dev);
	if (ret) {
		dev_err(dev, "can't attach iommu device: %d\n", ret);
		goto free_domain;
	}

	oproc->domain = domain;

	return 0;

free_domain:
	iommu_domain_free(domain);
	return ret;
}

static int omap_rproc_init_proc(struct rproc *rproc)
{
	int ret;

	ret = omap_rproc_enable_iommu(rproc);
	return ret;
}

static void omap_rproc_disable_iommu(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct iommu_domain *domain = oproc->domain;
	struct device *dev = rproc->dev;

	if (!domain)
		return;

	iommu_detach_device(domain, dev);
	iommu_domain_free(domain);

	return;
}

static void omap_rproc_shutdown_proc(struct rproc *rproc)
{
	omap_rproc_disable_iommu(rproc);
}

/**
 * omap_rproc_mbox_callback() - inbound mailbox message handler
 * @this: notifier block
 * @index: unused
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we also have some out-of-band values
 * that indicates different events. Those values are deliberately very
 * big so they don't coincide with virtqueue indices.
 */
static int omap_rproc_mbox_callback(struct notifier_block *this,
					unsigned long index, void *data)
{
	mbox_msg_t msg = (mbox_msg_t) data;
	struct omap_rproc *oproc = container_of(this, struct omap_rproc, nb);
	struct device *dev = oproc->rproc->dev;
	const char *name = oproc->rproc->name;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_CRASH:
		/* just log this for now. later, we'll also do recovery */
		dev_err(dev, "omap rproc %s crashed\n", name);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	default:
		/* ignore vq indices which are too large to be valid */
		if (msg >= 2) {
			dev_warn(dev, "invalid mbox msg: 0x%x\n", msg);
			break;
		}

		/*
		 * At this point, 'msg' contains the index of the vring
		 * which was just triggered.
		 */
		if (rproc_vq_interrupt(oproc->rproc, msg) == IRQ_NONE)
			dev_dbg(dev, "no message was found in vqid %d\n", msg);
	}

	return NOTIFY_DONE;
}

/* kick a virtqueue */
static void omap_rproc_kick(struct rproc *rproc, int vqid)
{
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	/* send the index of the triggered virtqueue in the mailbox payload */
	ret = omap_mbox_msg_send(oproc->mbox, vqid);
	if (ret)
		dev_err(rproc->dev, "omap_mbox_msg_send failed: %d\n", ret);
}

/*
 * Power up the remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int omap_rproc_start(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct platform_device *pdev = to_platform_device(rproc->dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	int ret;

	oproc->nb.notifier_call = omap_rproc_mbox_callback;

	/* every omap rproc is assigned a mailbox instance for messaging */
	oproc->mbox = omap_mbox_get(pdata->mbox_name, &oproc->nb);
	if (IS_ERR(oproc->mbox)) {
		ret = PTR_ERR(oproc->mbox);
		dev_err(rproc->dev, "omap_mbox_get failed: %d\n", ret);
		return ret;
	}

	/*
	 * Ping the remote processor. this is only for sanity-sake;
	 * there is no functional effect whatsoever.
	 *
	 * Note that the reply will _not_ arrive immediately: this message
	 * will wait in the mailbox fifo until the remote processor is booted.
	 */
	ret = omap_mbox_msg_send(oproc->mbox, RP_MBOX_ECHO_REQUEST);
	if (ret) {
		dev_err(rproc->dev, "omap_mbox_get failed: %d\n", ret);
		goto put_mbox;
	}

	ret = pdata->device_enable(pdev);
	if (ret) {
		dev_err(rproc->dev, "omap_device_enable failed: %d\n", ret);
		goto put_mbox;
	}

	return 0;

put_mbox:
	omap_mbox_put(oproc->mbox, &oproc->nb);
	return ret;
}

/* power off the remote processor */
static int omap_rproc_stop(struct rproc *rproc)
{
	struct platform_device *pdev = to_platform_device(rproc->dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	ret = pdata->device_shutdown(pdev);
	if (ret)
		return ret;

	omap_mbox_put(oproc->mbox, &oproc->nb);

	return 0;
}

/*
 * Some remote processors will ask us to allocate them physically contiguous
 * memory regions (which we call "carveouts"), and map them to specific
 * device addresses (which are hardcoded in the firmware).
 *
 * They may then ask us to copy objects into specific device addresses (e.g.
 * code/data sections) or expose us certain symbols in other device address
 * (e.g. their trace buffer).
 *
 * This function is an internal helper with which we can go over the allocated
 * carveouts and translate specific device address to kernel virtual addresses
 * so we can access the referenced memory.
 *
 * Note: phys_to_virt(iommu_iova_to_phys(oproc->domain, da)) will work too,
 * but only on kernel direct mapped RAM memory. Instead, we're just using
 * here the output of the DMA API, which should be more correct.
 */
static void *omap_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct rproc_mem_entry *carveout;
	void *ptr = NULL;

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		int offset = da - carveout->da;

		/* try next carveout if da is too small */
		if (offset < 0)
			continue;

		/* try next carveout if da is too large */
		if (offset + len > carveout->len)
			continue;

		ptr = carveout->va + offset;

		break;
	}

	return ptr;
}

static int omap_rproc_map(struct rproc *rproc, unsigned long iova,
			  phys_addr_t paddr, size_t size, int prot)
{
	struct omap_rproc *oproc = rproc->priv;

	return iommu_map(oproc->domain, iova, paddr, size, prot);
}

static int omap_rproc_unmap(struct rproc *rproc, unsigned long iova,
			    size_t size)
{
	struct omap_rproc *oproc = rproc->priv;

	return iommu_unmap(oproc->domain, iova, size);
}

static struct rproc_ops omap_rproc_ops = {
	.init		= omap_rproc_init_proc,
	.shutdown	= omap_rproc_shutdown_proc,
	.start		= omap_rproc_start,
	.stop		= omap_rproc_stop,
	.kick		= omap_rproc_kick,
	.map		= omap_rproc_map,
	.unmap		= omap_rproc_unmap,
	.da_to_va	= omap_rproc_da_to_va,
};

static int __devinit omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc;
	struct rproc *rproc;
	int ret;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(pdev->dev.parent, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(&pdev->dev, pdata->name, &omap_rproc_ops,
				pdata->firmware, sizeof(*oproc));
	if (!rproc)
		return -ENOMEM;

	oproc = rproc->priv;
	oproc->rproc = rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_register(rproc);
	if (ret)
		goto free_rproc;

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int __devexit omap_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	return rproc_unregister(rproc);
}

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = __devexit_p(omap_rproc_remove),
	.driver = {
		.name = "omap-rproc",
		.owner = THIS_MODULE,
	},
};

/* most of the below will go when module_platform_driver is merged */
static int __init omap_rproc_init(void)
{
	return platform_driver_register(&omap_rproc_driver);
}
module_init(omap_rproc_init);

static void __exit omap_rproc_exit(void)
{
	platform_driver_unregister(&omap_rproc_driver);
}
module_exit(omap_rproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
