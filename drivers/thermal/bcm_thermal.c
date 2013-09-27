/*
 * Copyright 2013 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2,
 * as published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php,
 * or by writing to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/**
*  Broadcom Thermal Management Unit - bcm_tmu
*/
#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/thermal.h>
#include <linux/of.h>

/* From TMON Register Database */
#define TMON_TEMP_VAL_OFFSET			0x0000001c
#define TMON_TEMP_VAL_TEMP_VAL_SHIFT		0
#define TMON_TEMP_VAL_TEMP_VAL_MASK		0x000003ff

/* Broadcom Thermal Zone Device Structure */
struct bcm_thermal_zone_priv {
	char name[THERMAL_NAME_LENGTH];
	void __iomem *base;
};

/* Temperature conversion function for TMON block */
static long raw_to_mcelsius(u32 raw)
{
	/*
	 * According to Broadcom internal Analog Module Specification
	 * the formula for converting TMON block output to temperature in
	 * degree Celsius is:
	 *	T = 428 - (0.561 * raw)
	 * Note: the valid operating range for the TMON block is -40C to 125C
	 */
	return 428000 - (561 * (long)raw);
}

/* Get temperature callback function for thermal zone */
static int bcm_get_temp(struct thermal_zone_device *thermal,
			unsigned long *temp)
{
	u32 raw;
	long mcelsius;
	struct bcm_thermal_zone_priv *priv = thermal->devdata;

	if (!priv) {
		pr_err("%s: thermal zone number %d devdata not initialized.\n",
			__func__, thermal->id);
		return -EINVAL;
	}

	raw = (readl(priv->base + TMON_TEMP_VAL_OFFSET)
		& TMON_TEMP_VAL_TEMP_VAL_MASK) >> TMON_TEMP_VAL_TEMP_VAL_SHIFT;

	pr_debug("%s: thermal zone number %d raw temp 0x%x\n", __func__,
		thermal->id, raw);

	mcelsius = raw_to_mcelsius(raw);

	/*
	 * Since 'mcelsius' might be negative, we need to limit it to smallest
	 * unsigned value before returning it to thermal framework.
	 */
	if (mcelsius < 0)
		*temp = 0;
	else
		*temp = mcelsius;

	pr_debug("%s: thermal zone number %d final temp %d\n", __func__,
		thermal->id, (int) *temp);

	return 0;
}

/* Operation callback functions for thermal zone */
static struct thermal_zone_device_ops bcm_dev_ops = {
	.get_temp = bcm_get_temp,
};

static const struct of_device_id bcm_tmu_match_table[] = {
	{ .compatible = "brcm,kona-thermal" },
	{},
};
MODULE_DEVICE_TABLE(of, bcm_tmu_match_table);

static int bcm_tmu_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *thermal = NULL;
	struct bcm_thermal_zone_priv *priv;
	struct resource *res;
	const char *str;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Failed to malloc priv.\n");
		return -ENOMEM;
	}

	/* Obtain the tmu name from device tree file */
	if (of_property_read_string(pdev->dev.of_node, "thermal-name",
			&str) == 0) {
		strlcpy(priv->name, str, sizeof(priv->name));
	} else {
		dev_err(&pdev->dev, "Failed to get thermal-name from DT.\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	thermal = thermal_zone_device_register(priv->name, 0, 0, priv,
						&bcm_dev_ops, NULL, 0, 0);
	if (IS_ERR(thermal)) {
		dev_err(&pdev->dev,
			"Failed to register Broadcom thermal zone device.\n");
		return PTR_ERR(thermal);
	}

	platform_set_drvdata(pdev, thermal);

	dev_info(&pdev->dev, "Broadcom Thermal Monitor Initialized.\n");

	return 0;
}

static int bcm_tmu_remove(struct platform_device *pdev)
{
	struct thermal_zone_device *broadcom_thermal =
		platform_get_drvdata(pdev);

	thermal_zone_device_unregister(broadcom_thermal);

	dev_info(&pdev->dev, "Broadcom Thermal Monitor Uninitialized.\n");

	return 0;
}

static struct platform_driver bcm_tmu_driver = {
	.driver = {
		.name = "bcm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = bcm_tmu_match_table,
	},
	.probe = bcm_tmu_probe,
	.remove = bcm_tmu_remove,
};

module_platform_driver(bcm_tmu_driver);

MODULE_DESCRIPTION("Broadcom Thermal Driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bcm-thermal");
