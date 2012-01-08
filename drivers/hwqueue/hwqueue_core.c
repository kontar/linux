/*
 * Hardware queue framework
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

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "hwqueue_internal.h"

static LIST_HEAD(hwqueue_devices);
static DEFINE_MUTEX(hwqueue_devices_lock);
static DEFINE_SPINLOCK(hwqueue_notifier_lock);

#define for_each_handle(qh, inst)			\
	list_for_each_entry(qh, &inst->handles, list)

#define for_each_device(hdev)				\
	list_for_each_entry(hdev, &hwqueue_devices, list)

#define for_each_instance(id, inst, hdev)		\
	for (id = 0, inst = hdev->instances;		\
	     id < (hdev)->num_queues;			\
	     id++, inst = hwqueue_id_to_inst(hdev, id))

static void __hwqueue_poll(unsigned long data);

static int hwqueue_poll_interval = 100;

static inline bool hwqueue_is_busy(struct hwqueue_instance *inst)
{
	return !list_empty(&inst->handles);
}

static inline bool hwqueue_is_exclusive(struct hwqueue_instance *inst)
{
	struct hwqueue_handle *tmp;

	for_each_handle(tmp, inst) {
		if (tmp->flags & O_EXCL)
			return true;
	}
	return false;
}

static inline bool hwqueue_is_writable(struct hwqueue_handle *qh)
{
	unsigned acc = qh->flags & O_ACCMODE;
	return (acc == O_RDWR || acc == O_WRONLY);
}

static inline bool hwqueue_is_readable(struct hwqueue_handle *qh)
{
	unsigned acc = qh->flags & O_ACCMODE;
	return (acc == O_RDWR || acc == O_RDONLY);
}

static inline struct hwqueue_instance *hwqueue_find_by_id(int id)
{
	struct hwqueue_device *hdev;

	for_each_device(hdev) {
		if (hdev->base_id <= id &&
		    hdev->base_id + hdev->num_queues > id) {
			id -= hdev->base_id;
			return hwqueue_id_to_inst(hdev, id);
		}
	}
	return NULL;
}

int hwqueue_device_register(struct hwqueue_device *hdev)
{
	struct hwqueue_instance *inst;
	int id, size, ret = -EEXIST;
	struct hwqueue_device *b;

	if (!hdev->ops || !hdev->dev)
		return -EINVAL;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(b) {
		if (b->base_id + b->num_queues >= hdev->base_id &&
		    hdev->base_id + hdev->num_queues >= b->base_id) {
			dev_err(hdev->dev, "id collision with %s\n",
				dev_name(b->dev));
			goto unlock_ret;
		}
	}
	ret = -ENOMEM;
	size  = sizeof(struct hwqueue_instance) + hdev->priv_size;
	size *= hdev->num_queues;
	hdev->instances = kzalloc(size, GFP_KERNEL);
	if (!hdev->instances)
		goto unlock_ret;

	ret = 0;
	for_each_instance(id, inst, hdev) {
		inst->hdev = hdev;
		INIT_LIST_HEAD(&inst->handles);
		setup_timer(&inst->poll_timer, __hwqueue_poll,
			    (unsigned long)inst);
		init_waitqueue_head(&inst->wait);
	}

	list_add(&hdev->list, &hwqueue_devices);

	dev_info(hdev->dev, "registered queues %d-%d\n",
		 hdev->base_id, hdev->base_id + hdev->num_queues - 1);

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);
	return ret;
}
EXPORT_SYMBOL(hwqueue_device_register);

int hwqueue_device_unregister(struct hwqueue_device *hdev)
{
	struct hwqueue_instance *inst;
	int id, ret = -EBUSY;

	mutex_lock(&hwqueue_devices_lock);

	for_each_instance(id, inst, hdev) {
		if (hwqueue_is_busy(inst)) {
			dev_err(hdev->dev, "cannot unregister busy dev\n");
			goto unlock_ret;
		}
	}

	list_del(&hdev->list);
	dev_info(hdev->dev, "unregistered queues %d-%d\n",
		 hdev->base_id, hdev->base_id + hdev->num_queues);
	kfree(hdev->instances);
	ret = 0;

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);
	return ret;
}
EXPORT_SYMBOL(hwqueue_device_unregister);

static struct hwqueue *__hwqueue_open(struct hwqueue_instance *inst,
				      const char *name, unsigned flags,
				      void *caller)
{
	struct hwqueue_device *hdev = inst->hdev;
	struct hwqueue_handle *qh;
	int ret;

	if (!try_module_get(hdev->dev->driver->owner))
		return ERR_PTR(-ENODEV);

	qh = kzalloc(sizeof(struct hwqueue_handle), GFP_KERNEL);
	if (!qh) {
		module_put(hdev->dev->driver->owner);
		return ERR_PTR(-ENOMEM);
	}

	qh->flags = flags;
	qh->inst = inst;
	qh->creator = caller;

	/* first opener? */
	if (!hwqueue_is_busy(inst)) {
		strncpy(inst->name, name, sizeof(inst->name));
		inst->creator = caller;
		ret = hdev->ops->open(inst, flags);
		if (ret) {
			kfree(qh);
			module_put(hdev->dev->driver->owner);
			return ERR_PTR(ret);
		}
	}

	list_add(&qh->list, &inst->handles);

	return hwqueue_from_handle(qh);
}

static struct hwqueue *hwqueue_open_by_id(const char *name, unsigned id,
					  unsigned flags, void *caller)
{
	struct hwqueue_instance *inst;
	struct hwqueue_device *hdev;
	struct hwqueue *qh;
	int match;

	mutex_lock(&hwqueue_devices_lock);

	qh = ERR_PTR(-ENODEV);
	inst = hwqueue_find_by_id(id);
	if (!inst)
		goto unlock_ret;
	hdev = inst->hdev;


	qh = ERR_PTR(-EINVAL);
	match = hdev->ops->match(inst, flags);

	if (match < 0)
		goto unlock_ret;

	qh = ERR_PTR(-EBUSY);
	if (hwqueue_is_exclusive(inst))
		goto unlock_ret;

	qh = ERR_PTR(-EEXIST);
	if ((flags & O_CREAT) && hwqueue_is_busy(inst))
		goto unlock_ret;

	qh = __hwqueue_open(inst, name, flags, caller);

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);

	return qh;
}

static struct hwqueue *hwqueue_open_any(const char *name, unsigned flags,
					void *caller)
{
	struct hwqueue_device *hdev;
	int match = INT_MAX, _match, id;
	struct hwqueue_instance *inst = NULL, *_inst;
	struct hwqueue *qh = ERR_PTR(-ENODEV);

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		for_each_instance(id, _inst, hdev) {
			_match = hdev->ops->match(_inst, flags);
			if (_match < 0) /* match error */
				continue;
			if (_match >= match) /* match is no better */
				continue;
			if (hwqueue_is_exclusive(_inst))
				continue;
			if ((flags & O_CREAT) && hwqueue_is_busy(_inst))
				continue;

			match = _match;
			inst = _inst;

			if (!match) /* made for each other */
				break;
		}
	}

	if (inst)
		qh = __hwqueue_open(inst, name, flags, caller);

	mutex_unlock(&hwqueue_devices_lock);
	return qh;
}

static struct hwqueue *hwqueue_open_by_name(const char *name, unsigned flags,
					    void *caller)
{
	struct hwqueue_device *hdev;
	struct hwqueue_instance *inst;
	struct hwqueue *qh = ERR_PTR(-EINVAL);
	int id;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		for_each_instance(id, inst, hdev) {
			int match = hdev->ops->match(inst, flags);
			if (match < 0)
				continue;
			if (!hwqueue_is_busy(inst))
				continue;
			if (hwqueue_is_exclusive(inst))
				continue;
			if (strcmp(inst->name, name))
				continue;
			qh = __hwqueue_open(inst, name, flags, caller);
			goto unlock_ret;
		}
	}

unlock_ret:
	mutex_unlock(&hwqueue_devices_lock);
	return qh;
}

/**
 * hwqueue_open() - open a hardware queue
 * @name	- name to give the queue handle
 * @id		- desired queue number if any
 *		  HWQUEUE_ANY: allocate any free queue, implies O_CREAT
 *		  HWQUEUE_BYNAME: open existing queue by name, implies !O_CREAT
 * @flags	- the following flags are applicable to queues:
 *	O_EXCL		- insist on exclusive ownership - will fail if queue is
 *			  already open.  Subsequent attempts to open the same
 *			  queue will also fail. O_EXCL => O_CREAT here.
 *	O_CREAT		- insist that queue not be already open - will fail if
 *			  queue is already open. Subsequent attempts to open
 *			  the same queue may succeed. O_CREAT is implied if
 *			  queue id == HWQUEUE_ANY.
 *	O_RDONLY	- pop only access
 *	O_WRONLY	- push only access
 *	O_RDWR		- push and pop access
 *	O_NONBLOCK	- never block on pushes and pops
 *   In addition, the following "hints" to the driver/hardware may be passed
 *   in at open:
 *	O_HIGHTHROUGHPUT - hint high throughput usage
 *	O_LOWLATENCY	 - hint low latency usage
 *
 * Returns a handle to the open hardware queue if successful.  Use IS_ERR()
 * to check the returned value for error codes.
 */
struct hwqueue *hwqueue_open(const char *name, unsigned id, unsigned flags)
{
	struct hwqueue *qh = ERR_PTR(-EINVAL);
	void *caller = __builtin_return_address(0);

	if (flags & O_EXCL)
		flags |= O_CREAT;

	switch (id) {
	case HWQUEUE_ANY:
		qh = hwqueue_open_any(name, flags, caller);
		break;
	case HWQUEUE_BYNAME:
		if (WARN_ON(flags & (O_EXCL | O_CREAT)))
			break;
		qh = hwqueue_open_by_name(name, flags, caller);
		break;
	default:
		qh = hwqueue_open_by_id(name, id, flags, caller);
		break;
	}

	return qh;
}
EXPORT_SYMBOL(hwqueue_open);

static void devm_hwqueue_release(struct device *dev, void *res)
{
	hwqueue_close(*(struct hwqueue **)res);
}

struct hwqueue *devm_hwqueue_open(struct device *dev, const char *name,
				  unsigned id, unsigned flags)
{
	struct hwqueue **ptr, *queue;

	ptr = devres_alloc(devm_hwqueue_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return NULL;

	queue = hwqueue_open(name, id, flags);
	if (queue) {
		*ptr = queue;
		devres_add(dev, ptr);
	} else
		devres_free(ptr);

	return queue;
}
EXPORT_SYMBOL(devm_hwqueue_open);

/**
 * hwqueue_close() - close a hardware queue handle
 * @queue	- handle to close
 */
void hwqueue_close(struct hwqueue *queue)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	mutex_lock(&hwqueue_devices_lock);

	list_del(&qh->list);
	module_put(hdev->dev->driver->owner);

	if (!hwqueue_is_busy(inst))
		hdev->ops->close(inst);

	mutex_unlock(&hwqueue_devices_lock);
}
EXPORT_SYMBOL(hwqueue_close);

static int devm_hwqueue_match(struct device *dev, void *res, void *match_data)
{
	return *(void **)res == match_data;
}

void devm_hwqueue_close(struct device *dev, struct hwqueue *queue)
{
	WARN_ON(devres_destroy(dev, devm_hwqueue_release, devm_hwqueue_match,
			       (void *)queue));
	hwqueue_close(queue);
}
EXPORT_SYMBOL(devm_hwqueue_close);

/**
 * hwqueue_get_id() - get an ID number for an open queue.  This ID may be
 *		      passed to another part of the kernel, which then opens the
 *		      queue by ID.
 * @queue	- queue handle
 *
 * Returns queue id (>= 0) on success, negative return value is an error.
 */
int hwqueue_get_id(struct hwqueue *queue)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	unsigned base_id = inst->hdev->base_id;

	return base_id + hwqueue_inst_to_id(inst);
}
EXPORT_SYMBOL(hwqueue_get_id);

/**
 * hwqueue_get_hw_id() - get an ID number for an open queue.  This ID may be
 *			 passed to hardware modules as a part of
 *			 descriptor/buffer content.
 * @queue	- queue handle
 *
 * Returns queue id (>= 0) on success, negative return value is an error.
 */
int hwqueue_get_hw_id(struct hwqueue *queue)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	if (!hdev->ops->get_hw_id)
		return -EINVAL;

	return hdev->ops->get_hw_id(inst);
}
EXPORT_SYMBOL(hwqueue_get_hw_id);

/**
 * hwqueue_set_notifier() - Set a notifier callback to a queue handle.  This
 *			    notifier is called whenever the queue has
 *			    something to pop.
 * @queue	- hardware queue handle
 * @fn		- callback function
 * @fn_arg	- argument for the callback function
 *
 * Hardware queues can have multiple notifiers attached to them.
 * The underlying notification mechanism may vary from queue to queue.  For
 * example, some queues may issue notify callbacks on timer expiry, and some
 * may do so in interrupt context.  Notifier callbacks may be called from an
 * atomic context, and _must not_ block ever.
 *
 * Returns 0 on success, errno otherwise.
 */
int hwqueue_set_notifier(struct hwqueue *queue, hwqueue_notify_fn fn,
			 void *fn_arg)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;
	unsigned long flags;

	if (!hwqueue_is_readable(qh))
		return -EINVAL;

	spin_lock_irqsave(&hwqueue_notifier_lock, flags);

	/*
	 * Ensure that the underlying driver's set_notify is called only for
	 * the first/last notifier added/removed for a particular hwqueue
	 * instance
	 */
	if (fn && !qh->notifier_fn) {
		bool first = (!inst->num_notifiers++);
		if (first)
			hdev->ops->set_notify(inst, true);
	} else if (!fn && qh->notifier_fn) {
		bool last = (!--inst->num_notifiers);
		if (last)
			hdev->ops->set_notify(inst, false);
	}

	qh->notifier_fn = fn;
	qh->notifier_fn_arg = fn_arg;

	spin_unlock_irqrestore(&hwqueue_notifier_lock, flags);

	return 0;
}
EXPORT_SYMBOL(hwqueue_set_notifier);

/**
 * hwqueue_is_empty() - poll a hardware queue and check if empty
 * @queue	- hardware queue handle
 *
 * Returns 'true' if empty, and 'false' otherwise
 */
bool hwqueue_is_empty(struct hwqueue *queue)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	if (!hdev->ops->is_empty || !hwqueue_is_readable(qh))
		return false; /* assume non empty, cannot really check */

	return hdev->ops->is_empty(inst);
}
EXPORT_SYMBOL(hwqueue_is_empty);

/**
 * hwqueue_flush() - forcibly empty a queue if possible
 * @queue	- hardware queue handle
 *
 * Returns 0 on success, errno otherwise.  This may not be universally
 * supported on all hardware queue implementations.
 */
int hwqueue_flush(struct hwqueue *queue)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	if (!hdev->ops->flush || !hwqueue_is_writable(qh))
		return -EPERM;
	return hdev->ops->flush(inst);
}
EXPORT_SYMBOL(hwqueue_flush);

/**
 * hwqueue_push() - push data (or descriptor) to the tail of a queue
 * @queue	- hardware queue handle
 * @data	- data to push
 * @size	- size of data to push
 *
 * Returns 0 on success, errno otherwise.
 */
int hwqueue_push(struct hwqueue *queue, void *data, unsigned size)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;

	if (!hdev->ops->push || !hwqueue_is_writable(qh))
		return -EPERM;
	atomic_inc(&qh->stats.pushes);
	return hdev->ops->push(inst, data, size);
}
EXPORT_SYMBOL(hwqueue_push);

static void *__hwqueue_pop_slow(struct hwqueue_instance *inst,
				unsigned *size,
				struct timeval *timeout)
{
	struct hwqueue_device *hdev = inst->hdev;
	void *data = NULL;
	int ret;

	if (timeout) {
		unsigned long expires = timeval_to_jiffies(timeout);

		ret = wait_event_interruptible_timeout(inst->wait,
				(data = hdev->ops->pop(inst, size)),
				expires);
		if (ret < 0)
			return ERR_PTR(ret);
		if (!ret && !data)
			return ERR_PTR(-ETIMEDOUT);
		jiffies_to_timeval(ret, timeout);
	} else {
		ret = wait_event_interruptible(inst->wait,
				(data = hdev->ops->pop(inst, size)));
		if (ret < 0)
			return ERR_PTR(ret);
		if (WARN_ON(!ret && !data))
			return ERR_PTR(-EIO);
	}

	return data;
}

/**
 * hwqueue_pop() - pop data (or descriptor) from the head of a queue
 * @queue	- hardware queue handle
 * @size	- hwqueue_pop fills this parameter on success
 * @timeout	- timeout value to use if blocking
 *
 * Returns a data/descriptor pointer on success.  Use IS_ERR() to identify
 * error values on return.
 */
void *hwqueue_pop(struct hwqueue *queue, unsigned *size,
		  struct timeval *timeout)
{
	struct hwqueue_handle *qh = hwqueue_to_handle(queue);
	struct hwqueue_instance *inst = qh->inst;
	struct hwqueue_device *hdev = inst->hdev;
	void *data;

	if (!hdev->ops->pop || !hwqueue_is_readable(qh))
		return ERR_PTR(-EPERM);
	atomic_inc(&qh->stats.pops);

	data = hdev->ops->pop(inst, size);
	if (data)
		return data;

	if (timeout && !timeout->tv_sec && !timeout->tv_usec)
		return NULL;

	if (qh->flags & O_NONBLOCK)
		return NULL;

	return __hwqueue_pop_slow(inst, size, timeout);
}
EXPORT_SYMBOL(hwqueue_pop);

/**
 * hwqueue_notify() - notify users on data availability
 * @inst	- hardware queue instance
 *
 * Walk through the notifier list for a hardware queue instance and issue
 * callbacks.  This function is called by drivers when data is available on a
 * hardware queue, either when notified via interrupt or on timer poll.
 */
void hwqueue_notify(struct hwqueue_instance *inst)
{
	struct hwqueue_handle *qh;
	unsigned long flags;

	spin_lock_irqsave(&hwqueue_notifier_lock, flags);
	for_each_handle(qh, inst) {
		if (qh->notifier_fn) {
			atomic_inc(&qh->notifier_calls);
			qh->notifier_fn(qh->notifier_fn_arg);
		}
	}
	wake_up_interruptible(&inst->wait);
	spin_unlock_irqrestore(&hwqueue_notifier_lock, flags);
}
EXPORT_SYMBOL(hwqueue_notify);

static void __hwqueue_poll(unsigned long data)
{
	struct hwqueue_instance *inst = (struct hwqueue_instance *)data;
	struct hwqueue_handle *qh;

	/* nasty - is_empty should be per-inst and not per-qh */
	for_each_handle(qh, inst) {
		if (!hwqueue_is_empty(hwqueue_from_handle(qh)))
			hwqueue_notify(inst);
		break;
	}

	mod_timer(&inst->poll_timer, jiffies +
		  msecs_to_jiffies(hwqueue_poll_interval));
}

void hwqueue_set_poll(struct hwqueue_instance *inst, bool enabled)
{
	unsigned long expires;

	if (!enabled) {
		del_timer_sync(&inst->poll_timer);
		return;
	}

	expires = jiffies + msecs_to_jiffies(hwqueue_poll_interval);
	mod_timer(&inst->poll_timer, expires);
}
EXPORT_SYMBOL(hwqueue_set_poll);

static void hwqueue_debug_show_instance(struct seq_file *s,
					struct hwqueue_instance *inst)
{
	struct hwqueue_device *hdev = inst->hdev;
	struct hwqueue_handle *qh;

	if (!hwqueue_is_busy(inst))
		return;

	seq_printf(s, "\tqueue id %d (%s), creator %pS\n",
		   hdev->base_id + hwqueue_inst_to_id(inst), inst->name,
		   inst->creator);

	for_each_handle(qh, inst) {
		struct hwqueue *queue = hwqueue_from_handle(qh);

		seq_printf(s, "\t\thandle %p, qh %p, creator %pS\n",
			   hwqueue_from_handle(qh), qh, qh->creator);
		seq_printf(s, "\t\t\tpushes %d, pushfn %pS\n",
			   atomic_read(&qh->stats.pushes), hdev->ops->push);
		seq_printf(s, "\t\t\tpops %d, popfn %pS\n",
			   atomic_read(&qh->stats.pops), hdev->ops->pop);
		seq_printf(s, "\t\t\t%s, emptyfn %pS\n",
			   hwqueue_is_empty(queue) ?  "empty" : "not empty",
			   hdev->ops->is_empty);
		seq_printf(s, "\t\t\tnotifier fn %pS, fn_arg %p, calls %d\n",
			   qh->notifier_fn, qh->notifier_fn_arg,
			   atomic_read(&qh->notifier_calls));
	}
}

static int hwqueue_debug_show(struct seq_file *s, void *v)
{
	struct hwqueue_device *hdev;
	struct hwqueue_instance *inst;
	int id;

	mutex_lock(&hwqueue_devices_lock);

	for_each_device(hdev) {
		seq_printf(s, "hdev %s: %u-%u\n",
			   dev_name(hdev->dev), hdev->base_id,
			   hdev->base_id + hdev->num_queues - 1);

		for_each_instance(id, inst, hdev)
			hwqueue_debug_show_instance(s, inst);
	}

	mutex_unlock(&hwqueue_devices_lock);

	return 0;
}

static int hwqueue_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwqueue_debug_show, NULL);
}

static const struct file_operations hwqueue_debug_ops = {
	.open		= hwqueue_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init hwqueue_debug_init(void)
{
	debugfs_create_file("hwqueues", S_IFREG | S_IRUGO, NULL, NULL,
			    &hwqueue_debug_ops);
	return 0;
}
device_initcall(hwqueue_debug_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware queue interface");
