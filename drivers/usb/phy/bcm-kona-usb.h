/*
 * bcm_kona_usb.h -- Broadcom Kona USB header file
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

#ifndef __BCM_KONA_USB_H
#define __BCM_KONA_USB_H

#include <linux/usb/phy.h>

struct bcm_kona_usb {
	struct usb_phy phy;
	struct device *dev;
	struct device *ctrl_dev;
};

extern struct device *bcm_kona_get_ctrl_dev(void);
extern void bcm_kona_ctrl_usb_phy_power(struct device *, int);

#endif /* __BCM_KONA_USB_H */
