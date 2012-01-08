/*
 * Hardware queues handle header
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Contact: Prabhu Kuttiyam <pkuttiyam@ti.com>
 *	    Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HWQUEUE_HWQUEUE_H
#define __HWQUEUE_HWQUEUE_H

#include <linux/device.h>
#include <linux/wait.h>
#include <linux/hwqueue.h>

struct hwqueue_handle;

struct hwqueue_stats {
	atomic_t	 pushes;
	atomic_t	 pops;
};

struct hwqueue_handle {
	struct hwqueue		 queue;
	unsigned		 flags;
	struct hwqueue_instance	*inst;
	struct list_head	 list;
	void			*creator;
	struct hwqueue_stats	 stats;
	hwqueue_notify_fn	 notifier_fn;
	void			*notifier_fn_arg;
	atomic_t		 notifier_calls;
};

struct hwqueue_instance {
	struct list_head	 handles;
	struct hwqueue_device	*hdev;
	struct timer_list	 poll_timer;
	wait_queue_head_t	 wait;
	void			*priv;
	void			*creator;
	char			 name[32];
	unsigned int		 num_notifiers;
};

struct hwqueue_device_ops {
	/*
	 * Return a match quotient (0 = best .. UINT_MAX-1) for a set of
	 * option flags.  Negative error values imply "do not allocate"
	 */
	int	 (*match)(struct hwqueue_instance *inst, unsigned flags);

	/* Initialize a queue inst when opened for the first time */
	int	 (*open)(struct hwqueue_instance *inst, unsigned flags);

	/* Close a queue inst when closed by the last user */
	void	 (*close)(struct hwqueue_instance *inst);

	/* Enable or disable notification */
	void	 (*set_notify)(struct hwqueue_instance *inst, bool enabled);

	/* Get a hardware identifier for a queue */
	int	 (*get_hw_id)(struct hwqueue_instance *inst);

	/* Push something into the queue */
	int	 (*push)(struct hwqueue_instance *inst, void *data,
			 unsigned size);

	/* Pop something from the queue */
	void	*(*pop)(struct hwqueue_instance *inst, unsigned *size);

	/* Poll a queue and check if it is empty */
	bool	 (*is_empty)(struct hwqueue_instance *inst);

	/* Flush a queue */
	int	 (*flush)(struct hwqueue_instance *inst);
};

struct hwqueue_device {
	struct list_head		 list;
	struct device			*dev;
	unsigned			 base_id;
	unsigned			 num_queues;
	unsigned			 priv_size;
	void				*instances;
	struct hwqueue_device_ops	*ops;
};

static inline int hwqueue_inst_to_id(struct hwqueue_instance *inst)
{
	struct hwqueue_device *hdev = inst->hdev;
	int offset = (void *)inst - hdev->instances;
	int inst_size = hdev->priv_size + sizeof(struct hwqueue_instance);

	BUG_ON(offset % inst_size);
	return offset / inst_size;
}

static inline struct hwqueue_instance *
hwqueue_id_to_inst(struct hwqueue_device *hdev, unsigned id)
{
	int inst_size = hdev->priv_size + sizeof(struct hwqueue_instance);

	return hdev->instances + (id * inst_size);
}

static inline void *hwqueue_inst_to_priv(struct hwqueue_instance *inst)
{
	return (void *)(inst + 1);
}

static inline struct hwqueue_handle *hwqueue_to_handle(struct hwqueue *q)
{
	if (IS_ERR_OR_NULL(q))
		return (struct hwqueue_handle *)q;
	return container_of(q, struct hwqueue_handle, queue);
}

static inline struct hwqueue *hwqueue_from_handle(struct hwqueue_handle *qh)
{
	if (IS_ERR_OR_NULL(qh))
		return (struct hwqueue *)qh;
	return &qh->queue;
}

int hwqueue_device_register(struct hwqueue_device *dev);
int hwqueue_device_unregister(struct hwqueue_device *dev);
void hwqueue_notify(struct hwqueue_instance *inst);
void hwqueue_set_poll(struct hwqueue_instance *inst, bool enabled);

#endif /* __HWQUEUE_HWQUEUE_H */
