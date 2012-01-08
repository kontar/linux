/*
 * Hardware queue framework
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
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

#ifndef __LINUX_HWQUEUE_H
#define __LINUX_HWQUEUE_H

#include <linux/err.h>
#include <linux/time.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/fcntl.h>

#define HWQUEUE_ANY			((unsigned)-1)
#define HWQUEUE_BYNAME			((unsigned)-2)

/* file fcntl flags repurposed for hwqueues */
#define O_HIGHTHROUGHPUT	O_DIRECT
#define O_LOWLATENCY		O_LARGEFILE

/**
 * struct hwqueue - a hardware queue handle
 *
 * This structure is empty because no part of the underlying implementation is
 * to be visible to users of this interface.
 */
struct hwqueue {
	/* empty */
};

/*
 * Hardware queue notifier callback prototype
 */
typedef void (*hwqueue_notify_fn)(void *arg);

struct hwqueue *hwqueue_open(const char *name, unsigned id, unsigned flags);

void hwqueue_close(struct hwqueue *queue);

struct hwqueue *devm_hwqueue_open(struct device *dev, const char *name,
				  unsigned id, unsigned flags);
void devm_hwqueue_close(struct device *dev, struct hwqueue *queue);

int hwqueue_get_id(struct hwqueue *queue);

int hwqueue_get_hw_id(struct hwqueue *queue);

int hwqueue_set_notifier(struct hwqueue *queue, hwqueue_notify_fn fn,
			 void *fn_arg);

bool hwqueue_is_empty(struct hwqueue *queue);

int hwqueue_flush(struct hwqueue *queue);

int hwqueue_push(struct hwqueue *queue, void *data, unsigned size);

void *hwqueue_pop(struct hwqueue *queue, unsigned *size,
		  struct timeval *timeout);

#endif /* __LINUX_HWQUEUE_H */
