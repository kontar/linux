/*
 * phy-bcm-kona-control.c - Broadcom Kona USB2 Phy Driver
 *
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
#include <linux/of.h>
#include <linux/platform_device.h>

#include "bcm-kona-usb.h"

static int bcm_kona_phy_init(struct usb_phy *uphy)
{
	struct bcm_kona_usb *phy = dev_get_drvdata(uphy->dev);

	bcm_kona_ctrl_usb_phy_power(phy->ctrl_dev, 1);

	return 0;
}

static void bcm_kona_phy_shutdown(struct usb_phy *uphy)
{
	struct bcm_kona_usb *phy = dev_get_drvdata(uphy->dev);

	bcm_kona_ctrl_usb_phy_power(phy->ctrl_dev, 0);
}

static int bcm_kona_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm_kona_usb *phy;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->ctrl_dev = bcm_kona_get_ctrl_dev();
	if (IS_ERR(phy->ctrl_dev)) {
		dev_dbg(&pdev->dev, "Failed to get control device\n");
		return -ENODEV;
	}

	phy->dev		= dev;
	phy->phy.dev		= phy->dev;
	phy->phy.label		= "bcm-kona-usb2";
	phy->phy.init		= bcm_kona_phy_init;
	phy->phy.shutdown	= bcm_kona_phy_shutdown;
	phy->phy.type		= USB_PHY_TYPE_USB2;

	platform_set_drvdata(pdev, phy);

	usb_add_phy_dev(&phy->phy);

	return 0;
}

static int bcm_kona_usb2_remove(struct platform_device *pdev)
{
	struct bcm_kona_usb *phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);

	return 0;
}

static const struct of_device_id bcm_kona_usb2_dt_ids[] = {
	{ .compatible = "brcm,kona-usb2" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bcm_kona_usb2_dt_ids);

static struct platform_driver bcm_kona_usb2_driver = {
	.probe		= bcm_kona_usb2_probe,
	.remove		= bcm_kona_usb2_remove,
	.driver		= {
		.name	= "bcm-kona-usb2",
		.owner	= THIS_MODULE,
		.of_match_table = bcm_kona_usb2_dt_ids,
	},
};

module_platform_driver(bcm_kona_usb2_driver);

MODULE_ALIAS("platform:bcm-kona-usb2");
MODULE_AUTHOR("Matt Porter");
MODULE_DESCRIPTION("BCM Kona USB 2.0 PHY driver");
MODULE_LICENSE("GPL v2");
