/*
 * phy-bcm-kona-ctrl.c - Broadcom Kona USB Control Driver
 *
 * Based on phy-omap-control.c
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 * Copyright (C) 2013 Linaro Limited
 * Matt Porter <matt.porter@linaro.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>

#include "bcm-kona-usb.h"

#define OTGCTL_OTGSTAT2		(1 << 31)
#define OTGCTL_OTGSTAT1		(1 << 30)
#define OTGCTL_PRST_N_SW	(1 << 11)
#define OTGCTL_HRESET_N		(1 << 10)
#define OTGCTL_UTMI_LINE_STATE1	(1 << 9)
#define OTGCTL_UTMI_LINE_STATE0	(1 << 8)

#define P1CTL_SOFT_RESET	(1 << 1)
#define P1CTL_NON_DRIVING	(1 << 0)

struct bcm_kona_usb_ctrl_regs {
	u32 ctrl;
	u32 cfg;
	u32 p1ctl;
	u32 status;
	u32 bc_cfg;
	u32 tp_in;
	u32 tp_out;
	u32 phy_ctrl;
	u32 usbreg;
	u32 usbproben;
};

struct bcm_kona_ctrl_usb {
	struct device *dev;
	struct bcm_kona_usb_ctrl_regs *regs;
};

static struct bcm_kona_ctrl_usb *ctrl_usb;

/**
 * bcm_kona_ctrl_dev - returns the device pointer for this control device
 *
 * This API should be called to get the device pointer for the Kona USB
 * control device. This device pointer should be used when calling the
 * exported bcm_kona_ctrl_usb_phy_power() API.
 */
struct device *bcm_kona_get_ctrl_dev(void)
{
	if (!ctrl_usb)
		return ERR_PTR(-ENODEV);

	return ctrl_usb->dev;
}
EXPORT_SYMBOL_GPL(bcm_kona_get_ctrl_dev);

/**
 * bcm_kona_ctrl_usb_phy_power - power on/off the phy using control module reg
 * @dev: the control module device
 * @on: 0 or 1, based on powering on or off the PHY
 */
void bcm_kona_ctrl_usb_phy_power(struct device *dev, int on)
{
	struct bcm_kona_ctrl_usb *ctrl = dev_get_drvdata(dev);
	u32 val;

	val = readl(&ctrl->regs->ctrl);
	if (on) {
		/* Configure and power PHY */
		val &= ~(OTGCTL_OTGSTAT2 | OTGCTL_OTGSTAT1 |
			 OTGCTL_UTMI_LINE_STATE1 | OTGCTL_UTMI_LINE_STATE0);
		val |= OTGCTL_PRST_N_SW | OTGCTL_HRESET_N;
		writel(val, &ctrl->regs->ctrl);

		/* Soft reset PHY */
		val = readl(&ctrl->regs->p1ctl);
		val &= ~P1CTL_NON_DRIVING;
		val |= P1CTL_SOFT_RESET;
		writel(val, &ctrl->regs->p1ctl);
		writel(val & ~P1CTL_SOFT_RESET, &ctrl->regs->p1ctl);
		/* Reset needs to be asserted for 2ms */
		mdelay(2);
		writel(val | P1CTL_SOFT_RESET, &ctrl->regs->p1ctl);
	} else {
		val &= ~(OTGCTL_PRST_N_SW | OTGCTL_HRESET_N);
		writel(val, &ctrl->regs->ctrl);
	}
}
EXPORT_SYMBOL_GPL(bcm_kona_ctrl_usb_phy_power);

static int bcm_kona_ctrl_usb_probe(struct platform_device *pdev)
{
	struct resource	*res;

	ctrl_usb = devm_kzalloc(&pdev->dev, sizeof(*ctrl_usb), GFP_KERNEL);
	if (!ctrl_usb) {
		dev_err(&pdev->dev, "unable to alloc memory for control usb\n");
		return -ENOMEM;
	}

	ctrl_usb->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctrl_usb->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctrl_usb->regs))
		return PTR_ERR(ctrl_usb->regs);

	dev_set_drvdata(ctrl_usb->dev, ctrl_usb);

	return 0;
}

static const struct of_device_id bcm_kona_ctrl_usb_id_table[] = {
	{ .compatible = "brcm,kona-ctrl-usb" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm_kona_ctrl_usb_id_table);

static struct platform_driver bcm_kona_ctrl_usb_driver = {
	.probe		= bcm_kona_ctrl_usb_probe,
	.driver		= {
		.name	= "bcm-kona-ctrl-usb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(bcm_kona_ctrl_usb_id_table),
	},
};

module_platform_driver(bcm_kona_ctrl_usb_driver);

MODULE_ALIAS("platform:bcm-kona-ctrl-usb");
MODULE_AUTHOR("Matt Porter");
MODULE_DESCRIPTION("Broadcom Kona USB Control Driver");
MODULE_LICENSE("GPL v2");
