/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com
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

#ifndef __MACH_KEYSTONE_DMA_H__
#define __MACH_KEYSTONE_DMA_H__

#include <linux/dmaengine.h>

#define DMA_HAS_PSINFO		BIT(31)
#define DMA_HAS_SWINFO		BIT(30)
#define DMA_HAS_TIMESTAMP	BIT(29)

#define DMA_GET_RX_FLOW		1000
#define DMA_GET_RX_QUEUE	1001

static inline int dma_get_rx_flow(struct dma_chan *chan) {
	return dmaengine_device_control(chan, DMA_GET_RX_FLOW, 0);
}

static inline int dma_get_rx_queue(struct dma_chan *chan) {
	return dmaengine_device_control(chan, DMA_GET_RX_QUEUE, 0);
}

#endif /* __MACH_KEYSTONE_DMA_H__ */

