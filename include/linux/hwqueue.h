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

/* hardware queue notifier callback prototype */
typedef void (*hwqueue_notify_fn)(void *arg);

struct hwqueue_stats {
	atomic_t	 pushes;
	atomic_t	 pops;
	atomic_t	 notifies;
};

struct hwqueue_instance;

struct hwqueue {
	int	 (*push)(struct hwqueue_instance *inst, void *data,
			 unsigned size);
	void	*(*pop)(struct hwqueue_instance *inst, unsigned *size);
	int	 (*flush)(struct hwqueue_instance *inst);
	int	 (*get_count)(struct hwqueue_instance *inst);

	struct hwqueue_instance	*inst;
	struct hwqueue_stats	 stats;
	hwqueue_notify_fn	 notifier_fn;
	void			*notifier_fn_arg;
	atomic_t		 notifier_enabled;
	struct rcu_head		 rcu;
	unsigned		 flags;
	struct list_head	 list;
};

struct hwqueue *hwqueue_open(const char *name, unsigned id, unsigned flags);

void hwqueue_close(struct hwqueue *queue);

struct hwqueue *devm_hwqueue_open(struct device *dev, const char *name,
				  unsigned id, unsigned flags);
void devm_hwqueue_close(struct device *dev, struct hwqueue *queue);

int hwqueue_get_id(struct hwqueue *queue);

int hwqueue_get_hw_id(struct hwqueue *queue);

int hwqueue_set_notifier(struct hwqueue *queue, hwqueue_notify_fn fn,
			 void *fn_arg);
int hwqueue_enable_notifier(struct hwqueue *queue);

int hwqueue_disable_notifier(struct hwqueue *queue);

void *__hwqueue_pop_slow(struct hwqueue_instance *inst, unsigned *size,
			 struct timeval *timeout);

/**
 * hwqueue_get_count() - poll a hardware queue and check if empty
 *			 and return number of elements in a queue
 * @qh	- hardware queue handle
 *
 * Returns 'true' if empty, and 'false' otherwise
 */
static inline int hwqueue_get_count(struct hwqueue *qh)
{
	if (unlikely(!qh->get_count))
		return -EINVAL;
	return qh->get_count(qh->inst);
}

/**
 * hwqueue_flush() - forcibly empty a queue if possible
 * @qh	- hardware queue handle
 *
 * Returns 0 on success, errno otherwise.  This may not be universally
 * supported on all hardware queue implementations.
 */
static inline int hwqueue_flush(struct hwqueue *qh)
{
	if (unlikely(!qh->flush))
		return -EINVAL;
	return qh->flush(qh->inst);
}

/**
 * hwqueue_push() - push data (or descriptor) to the tail of a queue
 * @qh	- hardware queue handle
 * @data	- data to push
 * @size	- size of data to push
 *
 * Returns 0 on success, errno otherwise.
 */
static inline int hwqueue_push(struct hwqueue *qh, void *data, unsigned size)
{
	if (unlikely(!qh->push))
		return -EINVAL;
	atomic_inc(&qh->stats.pushes);
	return qh->push(qh->inst, data, size);
}

/**
 * hwqueue_pop() - pop data (or descriptor) from the head of a queue
 * @qh	- hardware queue handle
 * @size	- hwqueue_pop fills this parameter on success
 * @timeout	- timeout value to use if blocking
 *
 * Returns a data/descriptor pointer on success.  Use IS_ERR() to identify
 * error values on return.
 */
static inline void *hwqueue_pop(struct hwqueue *qh, unsigned *size,
				struct timeval *timeout)
{
	void *data;

	if (unlikely(!qh->pop))
		return ERR_PTR(-EINVAL);
	atomic_inc(&qh->stats.pops);

	data = qh->pop(qh->inst, size);
	if (likely(data))
		return data;

	if (likely((qh->flags & O_NONBLOCK) ||
		   (timeout && !timeout->tv_sec && !timeout->tv_usec)))
		return NULL;

	return __hwqueue_pop_slow(qh->inst, size, timeout);
}

#endif /* __LINUX_HWQUEUE_H */
