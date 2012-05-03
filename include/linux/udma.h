/*
 * include/linux/ion.h
 *
 * Copyright (C) 2011 Google, Inc.
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

#ifndef _LINUX_UDMA_H
#define _LINUX_UDMA_H

#include <linux/types.h>
#include <linux/dmaengine.h>

struct udma_chan_data {
	unsigned long			 ring_virt;	/* in  */
	unsigned			 ring_size;	/* in  */
	unsigned			 num_desc;	/* in  */
	unsigned			 align;		/* in  */
	char				 name[64];	/* in  */
	int				 eventfd;	/* in  */
	enum dma_transfer_direction	 direction;	/* in  */

	int				 handle;	/* out */
};

#define UDMA_IOC_MAGIC		'I'
#define UDMA_IOC_ATTACH		_IOWR(UDMA_IOC_MAGIC, 0, struct udma_chan_data)
#define UDMA_IOC_DETACH		_IOW (UDMA_IOC_MAGIC, 1, int)
#define UDMA_IOC_KICK		_IOW (UDMA_IOC_MAGIC, 2, int)

#endif /* _LINUX_UDMA_H */
