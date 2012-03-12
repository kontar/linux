/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Author: Cyril Chemparathy <cyril@ti.com>
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

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/hwqueue.h>
#include <linux/dmapool.h>
#include <linux/hwqueue.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <mach/keystone-dma.h>

#define BITS(x)			(BIT(x) - 1)

#define DMA_LOOPBACK		BIT(31)
#define DMA_ENABLE		BIT(31)
#define DMA_TEARDOWN		BIT(30)

#define CHAN_HAS_EPIB		BIT(30)
#define CHAN_HAS_PSINFO		BIT(29)

#define DMA_TIMEOUT		1000	/* msecs */

#define DESC_HAS_EPIB		BIT(31)
#define DESC_PSLEN_MASK		BITS(6)
#define DESC_PSLEN_SHIFT	24
#define DESC_PSFLAGS_MASK	BITS(4)
#define DESC_PSFLAGS_SHIFT	16
#define DESC_RETQ_MASK		BITS(14)
#define DESC_RETQ_SHIFT		0
#define DESC_FLOWTAG_MASK	BITS(8)
#define DESC_FLOWTAG_SHIFT	16
#define DESC_LEN_MASK		BITS(22)
#define DESC_TYPE_HOST		1
#define DESC_TYPE_SHIFT		26

#define DMA_DEFAULT_NUM_DESCS	128
#define DMA_DEFAULT_PRIORITY	DMA_PRIO_MED_L
#define DMA_DEFAULT_FLOWTAG	0

enum keystone_dma_tx_priority {
	DMA_PRIO_HIGH	= 0,
	DMA_PRIO_MED_H,
	DMA_PRIO_MED_L,
	DMA_PRIO_LOW,
};

struct reg_global {
	u32	revision;
	u32	perf_control;
	u32	emulation_control;
	u32	priority_control;
	u32	qm_base_address[4];
};

struct reg_chan {
	u32	control;
	u32	mode;
	u32	__rsvd[6];
};

struct reg_tx_sched {
	u32	prio;
};

struct reg_rx_flow {
	u32	control;
	u32	tags;
	u32	tag_sel;
	u32	fdq_sel[2];
	u32	thresh[3];
};

#define BUILD_CHECK_REGS()						\
	do {								\
		BUILD_BUG_ON(sizeof(struct reg_global)   != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_chan)     != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_rx_flow)  != 32);	\
		BUILD_BUG_ON(sizeof(struct reg_tx_sched) !=  4);	\
	} while (0)

struct keystone_hw_priv {
	struct keystone_dma_desc	*desc;
	unsigned long			 pad[3];
};

struct keystone_hw_desc {
	u32	desc_info;
	u32	tag_info;
	u32	packet_info;
	u32	buff_len;
	u32	buff;
	u32	next_desc;
	u32	orig_len;
	u32	orig_buff;
	u32	time_stamp;
	u32	swdata[3];
	u32	psdata[16];
	struct keystone_hw_priv priv;
};
#define DESC_MAX_SIZE	offsetof(struct keystone_hw_desc, priv)
#define DESC_MIN_SIZE	offsetof(struct keystone_hw_desc, psdata)

enum keystone_desc_state {
	DESC_STATE_INVALID,	/* invalid, no associated hw desc */
	DESC_STATE_FREE,	/* free, may be allocated at prep */
	DESC_STATE_USER_ALLOC,	/* inuse, alloced, not submitted  */
	DESC_STATE_ACTIVE,	/* inuse, enqueued to dma hw      */
	DESC_STATE_COMPLETE,	/* inuse, transfer completed      */
	DESC_STATE_USER_RETURN,	/* inuse, user callback pending   */
	__DESC_STATE_MAX,
};

enum keystone_chan_state {
	CHAN_STATE_ACTIVE,
	CHAN_STATE_PAUSED,
	CHAN_STATE_TEARDOWN,
	CHAN_STATE_INVALID
};

struct keystone_dma_desc {
	enum keystone_desc_state	 state;
	struct keystone_hw_desc		*hwdesc;
	unsigned			 orig_size;
	struct dma_async_tx_descriptor	 adesc;
	struct keystone_dma_chan	*chan;
	unsigned long			 options;
	unsigned			 sg_len;
	enum dma_status			 status;
	struct scatterlist		*sg;
	struct list_head		 list;
};

#define from_hwdesc(d)		((d)->priv.desc)
#define to_hwdesc(d)		((d)->hwdesc)
#define from_adesc(d)		container_of(d, struct keystone_dma_desc, adesc)
#define to_adesc(d)		(&(d)->adesc)
#define from_cookie(chan, c)	((chan)->descs + (c))
#define to_cookie(chan, d)	((d) - (chan)->descs)
#define desc_dev(d)		chan_dev((d)->chan)

struct keystone_dma_device {
	struct dma_device		 engine;
	bool				 big_endian, loopback, enable_all;
	struct reg_global __iomem	*reg_global;
	struct reg_chan __iomem		*reg_tx_chan;
	struct reg_rx_flow __iomem	*reg_rx_flow;
	struct reg_chan __iomem		*reg_rx_chan;
	struct reg_tx_sched __iomem	*reg_tx_sched;
	unsigned			 max_rx_chan, max_tx_chan;
	unsigned			 max_rx_flow;
	bool				 debug;
};
#define from_dma(dma)	container_of(dma, struct keystone_dma_device, engine)
#define to_dma(dma)	(&(dma)->engine)
#define dma_dev(dma)	((dma)->engine.dev)

struct keystone_dma_chan {
	spinlock_t			 lock;
	enum keystone_chan_state	 state;
	struct keystone_dma_device	*dma;
	struct dma_chan			 achan;
	struct hwqueue			*q_submit, *q_complete, *q_pool;
	struct keystone_dma_desc	*descs;
	struct list_head		 desc_list[__DESC_STATE_MAX];
	int				 qnum_submit, qnum_complete;

	/*registers */
	struct reg_chan __iomem		*reg_chan;
	struct reg_tx_sched __iomem	*reg_tx_sched;
	struct reg_rx_flow __iomem	*reg_rx_flow;

	/* configuration stuff */
	enum keystone_dma_tx_priority	 tx_priority;
	enum dma_transfer_direction	 direction;
	int				 qcfg_submit, qcfg_complete;
	unsigned			 channel, flow;
	const char			*qname_pool;
	u32				 num_descs;
	u32				 tag_info;
	bool				 debug;
};
#define from_achan(ch)	container_of(ch, struct keystone_dma_chan, achan)
#define to_achan(ch)	(&(ch)->achan)
#define chan_dev(ch)	(&to_achan(ch)->dev->device)
#define chan_id(ch)	(to_achan(ch)->chan_id)
#define chan_name(ch)	((ch)->achan.name)
#define chan_dbg(ch, format, arg...)				\
	do {							\
		if ((ch)->debug)				\
		dev_dbg(chan_dev(ch), format, ##arg);	\
	} while (0)
#define chan_vdbg(ch, format, arg...)				\
	do {							\
		if ((ch)->debug)				\
		dev_vdbg(chan_dev(ch), format, ##arg);	\
	} while (0)

#define for_all_descs(desc, chan)				\
	for ((desc) = (chan)->descs;				\
	     (desc) < (chan)->descs + (chan)->num_descs;	\
	     (desc)++)

#define first_free_desc(ch)					\
	list_first_entry(&(ch)->desc_list[DESC_STATE_FREE],	\
			 struct keystone_dma_desc, list)

#define first_complete_desc(ch)					\
	list_first_entry(&(ch)->desc_list[DESC_STATE_COMPLETE],	\
			 struct keystone_dma_desc, list)

#define desc_list_empty(ch, state)				\
	list_empty(&(ch)->desc_list[state])

/**
 * dev_to_dma_chan - convert a device pointer to the its sysfs container object
 * @dev - device node
 */
static inline struct dma_chan *dev_to_dma_chan(struct device *dev)
{
	struct dma_chan_dev *chan_dev;

	chan_dev = container_of(dev, typeof(*chan_dev), device);
	return chan_dev->chan;
}

static inline u32 desc_read(struct keystone_dma_device *dma, u32 *field)
{
	return (dma->big_endian) ?  be32_to_cpu(*field) : le32_to_cpu(*field);
}

static inline void desc_write(struct keystone_dma_device *dma,
			      u32 *field, u32 val)
{
	*field = (dma->big_endian) ? cpu_to_be32(val) : cpu_to_le32(val);
}

static const char *desc_state_str(struct keystone_dma_desc *desc)
{
	static const char * const state_str[] = {
		[DESC_STATE_INVALID]		= "invalid",
		[DESC_STATE_FREE]		= "free",
		[DESC_STATE_ACTIVE]		= "active",
		[DESC_STATE_USER_ALLOC]		= "user-alloc",
		[DESC_STATE_COMPLETE]		= "complete",
		[DESC_STATE_USER_RETURN]	= "user-return",
	};

	if (desc->state < 0 || desc->state >= ARRAY_SIZE(state_str))
		return state_str[DESC_STATE_INVALID];
	else
		return state_str[desc->state];
}

static inline void desc_set_state(struct keystone_dma_desc *desc,
				  enum keystone_desc_state state)
{
	struct keystone_dma_chan *chan = desc->chan;
	void *caller = __builtin_return_address(0);

	desc->state = state;
	chan_vdbg(desc->chan, "desc %p state set to %s [by %pS]\n",
		  desc, desc_state_str(desc), caller);
	if (state >= __DESC_STATE_MAX || state < 0)
		state = DESC_STATE_INVALID;
	list_del(&desc->list);
	list_add_tail(&desc->list, &chan->desc_list[state]);
	cpu_relax();
}

static const char *chan_state_str(struct keystone_dma_chan *chan)
{
	static const char * const state_str[] = {
		[CHAN_STATE_ACTIVE]	= "active",
		[CHAN_STATE_PAUSED]	= "paused",
		[CHAN_STATE_TEARDOWN]	= "teardown",
		[CHAN_STATE_INVALID]	= "invalid",
	};

	if (chan->state < 0 || chan->state >= ARRAY_SIZE(state_str))
		return state_str[CHAN_STATE_INVALID];
	else
		return state_str[chan->state];
}

static void chan_set_state(struct keystone_dma_chan *chan,
			   enum keystone_chan_state state)
{
	void *caller = __builtin_return_address(0);

	chan->state = state;
	chan_vdbg(chan, "channel state set to %s [by %pS]\n",
		  chan_state_str(chan), caller);
	cpu_relax();
}

static inline bool chan_is_alive(struct keystone_dma_chan *chan)
{
	return (chan->state == CHAN_STATE_PAUSED ||
		chan->state == CHAN_STATE_ACTIVE);
}

static void __hwdesc_dump(struct keystone_dma_chan *chan,
			  struct keystone_hw_desc *hwdesc)
{
	unsigned long *data = (unsigned long *)hwdesc;

	chan_vdbg(chan, "\tdesc_info: %x\n",	hwdesc->desc_info);
	chan_vdbg(chan, "\ttag_info: %x\n",	hwdesc->tag_info);
	chan_vdbg(chan, "\tpacket_info: %x\n",	hwdesc->packet_info);
	chan_vdbg(chan, "\tbuff_len: %x\n",	hwdesc->buff_len);
	chan_vdbg(chan, "\tbuff: %x\n",		hwdesc->buff);
	chan_vdbg(chan, "\tnext_desc: %x\n",	hwdesc->next_desc);
	chan_vdbg(chan, "\torig_len: %x\n",	hwdesc->orig_len);
	chan_vdbg(chan, "\torig_buff: %x\n",	hwdesc->orig_buff);
	chan_vdbg(chan, "\ttime_stamp: %x\n",    hwdesc->time_stamp);

	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x00], data[0x01], data[0x02], data[0x03],
		  data[0x04], data[0x05], data[0x06], data[0x07]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x08], data[0x09], data[0x0a], data[0x0b],
		  data[0x0c], data[0x0d], data[0x0e], data[0x0f]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x10], data[0x11], data[0x12], data[0x13],
		  data[0x14], data[0x15], data[0x16], data[0x17]);
	chan_vdbg(chan, "\t%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
		  data[0x18], data[0x19], data[0x1a], data[0x1b],
		  data[0x1c], data[0x1d], data[0x1e], data[0x1f]);
}

static void hwdesc_dump(struct keystone_dma_chan *chan,
			struct keystone_hw_desc *hwdesc, const char *why)
{
	chan_vdbg(chan, "descriptor dump (%s): desc %p\n", why, hwdesc);
	__hwdesc_dump(chan, hwdesc);
}

static inline void hwdesc_clear(struct keystone_hw_desc *hwdesc)
{
	memset(hwdesc, 0, DESC_MAX_SIZE);
}

static void desc_dump(struct keystone_dma_chan *chan,
		      struct keystone_dma_desc *desc, const char *why)
{
	struct keystone_hw_desc *hwdesc = to_hwdesc(desc);

	chan_vdbg(chan, "%s: state %s, desc %p\n", why,
		  desc_state_str(desc), hwdesc);
	__hwdesc_dump(chan, hwdesc);
}

static void chan_complete(struct keystone_dma_chan *chan,
			  struct hwqueue *queue,
			  enum dma_status status)
{
	struct keystone_hw_desc *hwdesc;
	struct keystone_dma_desc *desc;
	struct dma_async_tx_descriptor *adesc;
	struct scatterlist *sg;
	unsigned long flags;
	int i, len;
	u32 *data;

	spin_lock_irqsave(&chan->lock, flags);

	for (;;) {
		hwdesc = hwqueue_pop(queue, NULL, NULL);
		if (!hwdesc)
			break;

		chan_vdbg(chan, "popped desc %p from queue %d\n",
			  hwdesc, hwqueue_get_id(queue));
		hwdesc_dump(chan, hwdesc, "complete");

		desc = from_hwdesc(hwdesc);
		adesc = to_adesc(desc);

		WARN_ON(desc->state != DESC_STATE_ACTIVE);

		sg = desc->sg;

		if (desc->options & DMA_HAS_SWINFO) {
			len  = sg->length / sizeof(u32);
			data = sg_virt(sg);
			sg++;
			for (i = 0; i < len; i++)
				data[i] = desc_read(chan->dma,
						    &hwdesc->swdata[i]);
		}

		if (desc->options & DMA_HAS_PSINFO) {
			len  = sg->length / sizeof(u32);
			data = sg_virt(sg);
			sg++;
			for (i = 0; i < len; i++)
				data[i] = desc_read(chan->dma,
						    &hwdesc->psdata[i]);
		}

		if (desc->options & DMA_HAS_TIMESTAMP) {
			len  = sg->length / sizeof(u32);
			data = sg_virt(sg);
			sg++;
			for (i = 0; i < len; i++)
				data[i] = desc_read(chan->dma,
						    &hwdesc->time_stamp);
		}

#ifdef DEBUG
		len = sg_dma_address(sg);
		WARN_ON(desc_read(chan->dma, &hwdesc->buff) != len);
		WARN_ON(desc_read(chan->dma, &hwdesc->orig_buff) != len);
#endif
		len = desc_read(chan->dma, &hwdesc->desc_info) & DESC_LEN_MASK;
		if (len != sg->length) {
			chan_vdbg(chan, "length adjusted %d -> %d\n",
				  sg->length, len);
			sg->length = len;
		}

		desc->status = status;
		desc_set_state(desc, DESC_STATE_COMPLETE);
	}

	while (!desc_list_empty(chan, DESC_STATE_COMPLETE)) {
		desc = first_complete_desc(chan);
		adesc = to_adesc(desc);

		if (chan->state != CHAN_STATE_ACTIVE && chan->state != CHAN_STATE_TEARDOWN)
			break;

		WARN_ON(desc->state != DESC_STATE_COMPLETE);
		desc_set_state(desc, DESC_STATE_USER_RETURN);

		spin_unlock_irqrestore(&chan->lock, flags);

		adesc->callback(adesc->callback_param);

		spin_lock_irqsave(&chan->lock, flags);

		WARN_ON(desc->state != DESC_STATE_USER_RETURN);
		desc_set_state(desc, DESC_STATE_FREE);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static void chan_complete_callback(void *arg)
{
	struct keystone_dma_chan *chan = arg;

	chan_complete(chan, chan->q_complete, DMA_SUCCESS);
}

static int __chan_submit(struct keystone_dma_chan *chan,
			 struct keystone_dma_desc *desc)
{
	struct keystone_hw_desc *hwdesc = to_hwdesc(desc);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&chan->lock, flags);

	if (WARN_ON(desc->state != DESC_STATE_USER_ALLOC)) {
		desc_set_state(desc, DESC_STATE_INVALID);
		goto done;
	}

	if (!chan_is_alive(chan))
		goto done;

	desc_set_state(desc, DESC_STATE_ACTIVE);

	hwdesc_dump(chan, hwdesc, "submit");
	chan_vdbg(chan, "pushing desc %p to queue %d\n",
		  hwdesc, hwqueue_get_id(chan->q_submit));

	ret = hwqueue_push(chan->q_submit, hwdesc, DESC_MAX_SIZE);
	WARN_ON(ret < 0);

done:
	spin_unlock_irqrestore(&chan->lock, flags);

	return ret;
}

dma_cookie_t chan_submit(struct dma_async_tx_descriptor *adesc)
{
	struct keystone_dma_desc *desc = from_adesc(adesc);
	struct keystone_dma_chan *chan = from_achan(adesc->chan);
	int ret;

	ret = __chan_submit(chan, desc);
	return (ret < 0) ? ret : adesc->cookie;
}

static struct hwqueue *chan_open_pool(struct keystone_dma_chan *chan)
{
	struct hwqueue *q;

	q = hwqueue_open(chan->qname_pool, HWQUEUE_BYNAME, O_RDWR | O_NONBLOCK);
	if (IS_ERR(q))
		dev_err(chan_dev(chan), "error %ld opening %s pool queue\n",
			PTR_ERR(q), chan->qname_pool);
	return q;
}

static struct hwqueue *chan_open_queue(struct keystone_dma_chan *chan,
				       unsigned id, unsigned flags,
				       const char *type)
{
	struct hwqueue *q;
	char name[32];

	/* always exclusive and non blocking */
	flags |= O_EXCL | O_NONBLOCK;

	scnprintf(name, sizeof(name), "%s:%s:%s", dev_name(chan_dev(chan)),
		  chan_name(chan), type);
	q = hwqueue_open(name, id, flags);
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "error %ld opening %s queue\n",
			PTR_ERR(q), type);
		return q;
	}
	return q;
}

static void chan_destroy_queues(struct keystone_dma_chan *chan)
{
	if (chan->q_complete) {
		hwqueue_set_notifier(chan->q_complete, NULL, NULL);
		hwqueue_close(chan->q_complete);
	}
	chan->q_complete = NULL;

	if (chan->q_submit)
		hwqueue_close(chan->q_submit);
	chan->q_submit = NULL;

	if (chan->q_pool)
		hwqueue_close(chan->q_pool);
	chan->q_pool = NULL;
}

static int chan_setup_queues(struct keystone_dma_chan *chan)
{
	unsigned flags = O_RDWR;
	struct hwqueue *q;
	int ret = 0;

	/* open pool queue */
	q = chan_open_pool(chan);
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "failed to open pool queue (%ld)\n",
			PTR_ERR(q));
		goto fail;
	}
	chan->q_pool = q;

	/* open submit queue */
	q = chan_open_queue(chan, chan->qcfg_submit, flags, "submit");
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "failed to open submit queue (%ld)\n",
			PTR_ERR(q));
		goto fail;
	}
	chan->qnum_submit = hwqueue_get_id(q);
	chan->q_submit = q;

	/* open completion queue */
	flags = O_RDONLY | O_HIGHTHROUGHPUT;
	q = chan_open_queue(chan, chan->qcfg_complete, flags, "complete");
	if (IS_ERR(q)) {
		dev_err(chan_dev(chan), "failed to open complete queue (%ld)\n",
			PTR_ERR(q));
		goto fail;
	}
	chan->qnum_complete = hwqueue_get_id(q);
	chan->q_complete = q;

	/* setup queue notifier */
	ret = hwqueue_set_notifier(q, chan_complete_callback, chan);
	if (ret < 0) {
		dev_err(chan_dev(chan), "failed to setup queue notify (%d)\n",
			ret);
		goto fail;
	}

	chan_dbg(chan, "opened queues: submit %d, complete %d\n",
		 chan->qnum_submit, chan->qnum_complete);

	return 0;

fail:
	chan_destroy_queues(chan);
	return IS_ERR(q) ? PTR_ERR(q) : ret;
}

static int chan_start(struct keystone_dma_chan *chan)
{
	unsigned v;

	if (chan->reg_chan && !chan->dma->enable_all) {
		__raw_writel(0, &chan->reg_chan->mode);
		__raw_writel(DMA_ENABLE, &chan->reg_chan->control);
	}

	if (chan->reg_tx_sched) {
		__raw_writel(chan->tx_priority, &chan->reg_tx_sched->prio);
	}

	if (chan->reg_rx_flow) {
		v  = CHAN_HAS_EPIB | CHAN_HAS_PSINFO | chan->qnum_complete;
		v |= DESC_TYPE_HOST << DESC_TYPE_SHIFT;
		__raw_writel(v, &chan->reg_rx_flow->control);

		__raw_writel(0, &chan->reg_rx_flow->tags);
		__raw_writel(0, &chan->reg_rx_flow->tag_sel);

		v = chan->qnum_submit << 16;
		__raw_writel(v, &chan->reg_rx_flow->fdq_sel[0]);

		__raw_writel(0, &chan->reg_rx_flow->fdq_sel[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[0]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[2]);
	}

	chan_vdbg(chan, "channel started\n");
	return 0;
}

static int chan_teardown(struct keystone_dma_chan *chan)
{
	unsigned long end, value;

	if (!chan->reg_chan)
		return 0;

	/* indicate teardown */
	__raw_writel(DMA_TEARDOWN, &chan->reg_chan->control);

	/* wait for the dma to shut itself down */
	end = jiffies + msecs_to_jiffies(DMA_TIMEOUT);
	do {
		value = __raw_readl(&chan->reg_chan->control);
		if ((value & DMA_ENABLE) == 0)
			break;
	} while (time_after(end, jiffies));

	if (__raw_readl(&chan->reg_chan->control) & DMA_ENABLE) {
		dev_err(chan_dev(chan), "timeout waiting for tx teardown\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void chan_stop(struct keystone_dma_chan *chan)
{
	unsigned long end;

	if (chan->reg_rx_flow) {
		/* first detach fdqs, starve out the flow */
		__raw_writel(0, &chan->reg_rx_flow->fdq_sel[0]);
		__raw_writel(0, &chan->reg_rx_flow->fdq_sel[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[0]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[1]);
		__raw_writel(0, &chan->reg_rx_flow->thresh[2]);
	}

	/* teardown the dma channel if we have such a thing */
	chan_teardown(chan);

	/* drain submitted buffers */
	chan_complete(chan, chan->q_submit, DMA_ERROR);

	/* wait for active transfers to complete */
	end = jiffies + msecs_to_jiffies(DMA_TIMEOUT);
	do {
		chan_complete(chan, chan->q_complete, DMA_SUCCESS);
		if (desc_list_empty(chan, DESC_STATE_ACTIVE) ||
		    signal_pending(current))
			break;
		schedule_timeout_interruptible(DMA_TIMEOUT / 10);
	} while (time_after(end, jiffies));

	WARN_ON(!desc_list_empty(chan, DESC_STATE_USER_ALLOC));
	WARN_ON(!desc_list_empty(chan, DESC_STATE_ACTIVE));
	WARN_ON(!desc_list_empty(chan, DESC_STATE_COMPLETE));
	WARN_ON(!desc_list_empty(chan, DESC_STATE_USER_RETURN));

	/* then disconnect the completion side and hope for the best */
	if (chan->reg_rx_flow) {
		__raw_writel(0, &chan->reg_rx_flow->control);
		__raw_writel(0, &chan->reg_rx_flow->tags);
		__raw_writel(0, &chan->reg_rx_flow->tag_sel);
	}
	chan_vdbg(chan, "channel stopped\n");
}

static int chan_setup_descs(struct keystone_dma_chan *chan)
{
	struct keystone_dma_desc *desc;
	struct dma_async_tx_descriptor *adesc;
	struct keystone_hw_desc *hwdesc;
	int ndesc = 0;

	chan->descs = kzalloc(sizeof(*desc) * chan->num_descs, GFP_KERNEL);
	if (!chan->descs)
		return -ENOMEM;

	for_all_descs(desc, chan) {
		desc->chan = chan;

		INIT_LIST_HEAD(&desc->list);
		desc_set_state(desc, DESC_STATE_INVALID);

		adesc = to_adesc(desc);
		adesc->cookie = to_cookie(chan, desc);
		adesc->chan = to_achan(chan);
		adesc->tx_submit = chan_submit;

		hwdesc = hwqueue_pop(chan->q_pool, &desc->orig_size, NULL);
		if (IS_ERR_OR_NULL(hwdesc))
			continue;

		if (desc->orig_size < sizeof(struct keystone_hw_desc)) {
			hwqueue_push(chan->q_pool, hwdesc, desc->orig_size);
			continue;
		}

		desc->hwdesc = hwdesc;

		memset(hwdesc, 0, desc->orig_size);
		hwdesc->priv.desc = desc;

		desc_set_state(desc, DESC_STATE_FREE);
		ndesc++;
	}

	if (!ndesc) {
		kfree(chan->descs);
		dev_err(chan_dev(chan), "no descriptors for channel\n");
	} else {
		chan_vdbg(chan, "initialized %d descriptors\n", ndesc);
	}

	return ndesc ? ndesc : -ENOMEM;
}

static void chan_destroy_descs(struct keystone_dma_chan *chan)
{
	struct keystone_dma_desc *desc;

	for_all_descs(desc, chan) {
		switch (desc->state) {
		case DESC_STATE_FREE:
			hwqueue_push(chan->q_pool, desc->hwdesc,
				     desc->orig_size);
			break;
		case DESC_STATE_INVALID:
			break;
		default:
			desc_dump(chan, desc, "leaked descriptor");
			break;
		}

		list_del(&desc->list);
	}

	kfree(chan->descs);
	chan->descs = NULL;

	BUG_ON(!desc_list_empty(chan, DESC_STATE_ACTIVE) ||
	       !desc_list_empty(chan, DESC_STATE_FREE));
}

static int chan_init(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	int ret, descs;

	chan_vdbg(chan, "initializing %s channel\n",
		  chan->direction == DMA_MEM_TO_DEV   ? "transmit" :
		  chan->direction == DMA_DEV_TO_MEM ? "receive"  :
		  "unknown");

	if (WARN_ON(chan->direction != DMA_MEM_TO_DEV &&
		    chan->direction != DMA_DEV_TO_MEM)) {
		dev_err(chan_dev(chan), "bad direction\n");
		return -EINVAL;
	}

	if (WARN_ON(chan->state != CHAN_STATE_INVALID)) {
		dev_err(chan_dev(chan), "bad state (%s) on alloc\n",
			chan_state_str(chan));
		return -EBUSY;
	}

	ret = chan_setup_queues(chan);
	if (ret < 0)
		return ret;

	ret = chan_setup_descs(chan);
	if (ret < 0) {
		chan_destroy_queues(chan);
		return ret;
	}
	descs = ret;

	ret = chan_start(chan);

	if (ret < 0) {
		chan_destroy_descs(chan);
		chan_destroy_queues(chan);
		return ret;
	}

	chan_set_state(chan, CHAN_STATE_ACTIVE);

	chan_vdbg(chan, "channel initialized\n");

	return descs;
}

static void chan_destroy(struct dma_chan *achan)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	if (!chan_is_alive(chan)) {
		dev_err(chan_dev(chan), "cannot destroy in %s state\n",
			chan_state_str(chan));
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	chan_vdbg(chan, "destroying channel\n");
	chan_set_state(chan, CHAN_STATE_TEARDOWN);

	spin_unlock_irqrestore(&chan->lock, flags);

	chan_stop(chan);
	chan_destroy_descs(chan);
	chan_destroy_queues(chan);

	chan_set_state(chan, CHAN_STATE_INVALID);

	chan_vdbg(chan, "channel destroyed\n");
}

static int chan_set_notify(struct keystone_dma_chan *chan, bool enable)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&chan->lock, flags);

	if (!chan_is_alive(chan)) {
		dev_err(chan_dev(chan), "invalid state (%s) for pause/resume\n",
			chan_state_str(chan));
		spin_unlock_irqrestore(&chan->lock, flags);
		return -EINVAL;
	}

	if ((chan->state == CHAN_STATE_PAUSED && !enable) ||
	    (chan->state == CHAN_STATE_ACTIVE && enable)) {
		chan_vdbg(chan, "already in state (%s) for %s\n",
			  chan_state_str(chan),
			  enable ? "resume" : "pause");
		/* nothing to do */
		spin_unlock_irqrestore(&chan->lock, flags);
		return 0;
	}

	ret = hwqueue_set_notifier(chan->q_complete,
				   enable ? chan_complete_callback : NULL,
				   enable ? chan : NULL);
	if (ret < 0) {
		dev_err(chan_dev(chan), "unable to setup queue notifier\n");
		spin_unlock_irqrestore(&chan->lock, flags);
		return ret;
	}

	chan_set_state(chan, enable ? CHAN_STATE_ACTIVE : CHAN_STATE_PAUSED);
	chan_vdbg(chan, "channel %s\n", enable ? "resumed" : "paused");

	spin_unlock_irqrestore(&chan->lock, flags);

	if (enable) {
		chan_vdbg(chan, "processing pending transfers\n");
		chan_complete(chan, chan->q_complete, DMA_SUCCESS);
	}

	return 0;
}

static void chan_issue_pending(struct dma_chan *chan)
{
	/* nothing to do, we enqueue right away on submit */
}

static int chan_control(struct dma_chan *achan, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_device *dma = chan->dma;
	struct dma_slave_config *config;
	int ret = 0;

	switch ((int)cmd) {
	case DMA_GET_RX_FLOW:
		ret = -EINVAL;
		if (chan->direction == DMA_DEV_TO_MEM && chan->reg_rx_flow)
			ret = chan->reg_rx_flow - dma->reg_rx_flow;
		break;

	case DMA_GET_RX_QUEUE:
		ret = -EINVAL;
		if (chan->direction == DMA_DEV_TO_MEM)
			ret = chan->qnum_complete;
		break;

	case DMA_PAUSE:
		ret = chan_set_notify(chan, false);
		break;

	case DMA_RESUME:
		ret = chan_set_notify(chan, true);
		break;

	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;
		ret = (config->direction != chan->direction) ? -EINVAL : 0;
		break;

	case DMA_TERMINATE_ALL:
	default:
		ret = -ENOTSUPP;
		break;
	}

	return ret;
}

static enum dma_status chan_xfer_status(struct dma_chan *achan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	struct keystone_dma_desc *desc = from_cookie(chan, cookie);
	enum dma_status status;

	switch (desc->state) {
	case DESC_STATE_USER_RETURN:
		status = desc->status;
		break;
	case DESC_STATE_ACTIVE:
	case DESC_STATE_COMPLETE:
		status = DMA_IN_PROGRESS;
		break;
	default:
		status = DMA_ERROR;
		break;
	}

	chan_vdbg(chan, "returning status %d for desc %p\n", status, desc);

	return status;
}

static struct dma_async_tx_descriptor *
chan_prep_slave_sg(struct dma_chan *achan, struct scatterlist *_sg,
		   unsigned int _num_sg, enum dma_transfer_direction direction,
		   unsigned long options)
{
	struct keystone_dma_chan *chan = from_achan(achan);
	void *caller = __builtin_return_address(0);
	struct dma_async_tx_descriptor *adesc;
	struct keystone_hw_desc *hwdesc;
	struct keystone_dma_desc *desc;
	struct scatterlist *sg = _sg;
	unsigned num_sg = _num_sg;
	u32 pslen = 0, *psdata = NULL;
	u32 swlen = 0, *swdata = NULL;
	u32 tstamplen = 0, *tsdata = NULL;
	unsigned long flags;
	u32 v, psflags;
	int i;

	chan_vdbg(chan, "prep, caller %pS, channel %p, direction %s\n",
		  achan, caller,
		  chan->direction == DMA_MEM_TO_DEV ? "transmit" :
		  chan->direction == DMA_DEV_TO_MEM ? "receive" : "unknown");

	psflags = options & DESC_PSFLAGS_MASK;

	if (unlikely(direction != chan->direction)) {
		dev_err(chan_dev(chan), "mismatched dma direction\n");
		return ERR_PTR(-EINVAL);
	}

	if (options & DMA_HAS_SWINFO) {
		swlen  = sg->length;
		swdata = sg_virt(sg);
		num_sg--;
		sg++;

		if (unlikely(swlen != sizeof(hwdesc->swdata))) {
			dev_err(chan_dev(chan), "invalid swdata length %d\n",
				swlen);
			return ERR_PTR(-EINVAL);
		}
		swlen /= sizeof(u32);
	}

	if (options & DMA_HAS_PSINFO) {
		pslen  = sg->length;
		psdata = sg_virt(sg);
		num_sg--;
		sg++;

		if (unlikely(pslen & (sizeof(u32) - 1) ||
			     pslen >= sizeof(hwdesc->psdata))) {
			dev_err(chan_dev(chan), "invalid psdata length %d\n",
				pslen);
			return ERR_PTR(-EINVAL);
		}
		pslen /= sizeof(u32);
	}

	if (options & DMA_HAS_TIMESTAMP) {
		tstamplen  = sg->length;
		tsdata = sg_virt(sg);
		num_sg--;
		sg++;

		if (tstamplen != 4) {
			dev_err(chan_dev(chan), "invalid time stamp length %d\n",
				tstamplen);
			return ERR_PTR(-EINVAL);
		}
	}

	spin_lock_irqsave(&chan->lock, flags);

	if (!chan_is_alive(chan)) {
		dev_err(chan_dev(chan), "cannot submit in state %s\n",
			chan_state_str(chan));
		spin_unlock_irqrestore(&chan->lock, flags);
		return ERR_PTR(-ENODEV);
	}

retry:
	if (desc_list_empty(chan, DESC_STATE_FREE)) {
		chan_vdbg(chan, "out of descriptors\n");
		spin_unlock_irqrestore(&chan->lock, flags);
		return ERR_PTR(-ENOMEM);
	}

	desc = first_free_desc(chan);

	if (WARN_ON(desc->state != DESC_STATE_FREE)) {
		desc_set_state(desc, DESC_STATE_INVALID);
		goto retry;
	}

	desc_set_state(desc, DESC_STATE_USER_ALLOC);

	spin_unlock_irqrestore(&chan->lock, flags);

	hwdesc = to_hwdesc(desc);
	adesc = to_adesc(desc);

	desc->options	= options;
	desc->sg_len	= _num_sg;
	desc->sg	= _sg;

	hwdesc_clear(hwdesc);

	desc_write(chan->dma, &hwdesc->tag_info,   chan->tag_info);

	WARN_ON(sg->length	    & ~DESC_LEN_MASK	||
		pslen		    & ~DESC_PSLEN_MASK	||
		chan->qnum_complete & ~DESC_RETQ_MASK);

	desc_write(chan->dma, &hwdesc->desc_info, sg->length);
	desc_write(chan->dma, &hwdesc->buff_len,  sg->length);
	desc_write(chan->dma, &hwdesc->orig_len,  sg->length);
	desc_write(chan->dma, &hwdesc->buff,	  sg_dma_address(sg));
	desc_write(chan->dma, &hwdesc->orig_buff, sg_dma_address(sg));

	v = (DESC_HAS_EPIB				|
	     pslen		 << DESC_PSLEN_SHIFT	|
	     psflags		 << DESC_PSFLAGS_SHIFT	|
	     chan->qnum_complete << DESC_RETQ_SHIFT);
	desc_write(chan->dma, &hwdesc->packet_info, v);

	if (options & DMA_HAS_TIMESTAMP)
		desc_write(chan->dma, &hwdesc->time_stamp, tsdata[0]);

	for (i = 0; i < ARRAY_SIZE(hwdesc->swdata); i++) {
		desc_write(chan->dma, &hwdesc->swdata[i],
			   (i < swlen) ? swdata[i] : 0);
	}

	for (i = 0; i < ARRAY_SIZE(hwdesc->psdata); i++) {
		desc_write(chan->dma, &hwdesc->psdata[i],
			   (i < pslen) ? psdata[i] : 0);
	}

	return adesc;
}

static __devinit void __iomem *
dma_get_regs(struct keystone_dma_device *dma, int index, const char *name,
	     resource_size_t *_size)
{
	struct device *dev = dma_dev(dma);
	struct device_node *node = dev->of_node;
	resource_size_t size;
	struct resource res;
	void __iomem *regs;

	if (of_address_to_resource(node, index, &res)) {
		dev_err(dev, "could not find %s resource (index %d)\n",
			name, index);
		return NULL;
	}
	size = resource_size(&res);

	if (!devm_request_mem_region(dev, res.start, size, dev_name(dev))) {
		dev_err(dev, "could not reserve %s resource (index %d)\n",
			name, index);
		return NULL;
	}

	regs = devm_ioremap_nocache(dev, res.start, size);
	if (!regs) {
		dev_err(dev, "could not map %s resource (index %d)\n",
			name, index);
		return NULL;
	}

	dev_vdbg(dev, "index: %d, res:%s, size:%x, phys:%x, virt:%p\n",
		 index, name, size, res.start, regs);

	if (_size)
		*_size = size;

	return regs;
}

static __devinit int dma_init_rx_chan(struct keystone_dma_chan *chan,
				      struct device_node *node)
{
	struct keystone_dma_device *dma = chan->dma;
	struct device *dev = dma_dev(chan->dma);
	u32 flow = 0, channel = 0;
	int ret;

	ret = of_property_read_u32(node, "flow", &flow);
	if (ret < 0) {
		dev_dbg(dev, "no flow for %s channel\n", chan_name(chan));
	} else if (flow >= dma->max_rx_flow) {
		dev_err(dev, "invalid flow %d for %s channel\n",
			flow, chan_name(chan));
		return -EINVAL;
	} else {
		chan->flow = flow;
		chan->reg_rx_flow = dma->reg_rx_flow + flow;
	}

	ret = of_property_read_u32(node, "channel", &channel);
	if (ret < 0) {
		dev_dbg(dev, "no hw channel for %s channel\n",
			chan_name(chan));
	} else if (channel >= dma->max_rx_chan) {
		dev_err(dev, "invalid hw channel %d for %s channel\n",
			channel, chan_name(chan));
		return -EINVAL;
	} else {
		chan->channel = channel;
		chan->reg_chan = dma->reg_rx_chan + channel;
	}

	dev_dbg(dev, "%s rx channel: pool %s, descs %d, "
		"channel %d (%p), flow %d (%p), submit %d, complete %d\n",
		chan_name(chan), chan->qname_pool, chan->num_descs,
		chan->channel, chan->reg_chan,
		chan->flow, chan->reg_rx_flow,
		chan->qcfg_submit, chan->qcfg_complete);

	return 0;
}

static __devinit int dma_init_tx_chan(struct keystone_dma_chan *chan,
				      struct device_node *node)
{
	struct keystone_dma_device *dma = chan->dma;
	struct device *dev = dma_dev(chan->dma);
	u32 channel, priority, flowtag;
	int ret;

	ret = of_property_read_u32(node, "channel", &channel);
	if (ret < 0) {
		dev_dbg(dev, "no hw channel for %s channel\n",
			chan_name(chan));
	} else if (channel >= dma->max_tx_chan) {
		dev_err(dev, "invalid hw channel %d for %s channel\n",
			channel, chan_name(chan));
		return -EINVAL;
	} else {
		chan->channel = channel;
		chan->reg_chan = dma->reg_tx_chan + channel;
		chan->reg_tx_sched = dma->reg_tx_sched + channel;
	}

	ret = of_property_read_u32(node, "priority", &priority);
	if (ret < 0) {
		dev_dbg(dev, "no priority for %s channel\n", chan_name(chan));
		priority = DMA_DEFAULT_PRIORITY;
	}
	chan->tx_priority = priority;

	ret = of_property_read_u32(node, "flowtag", &flowtag);
	if (ret < 0) {
		dev_dbg(dev, "no flowtag for %s channel\n", chan_name(chan));
		flowtag = DMA_DEFAULT_FLOWTAG;
	} else if (flowtag & ~DESC_FLOWTAG_MASK) {
		dev_err(dev, "invalid flow tag %x\n", flowtag);
		return -EINVAL;
	} else {
		chan->tag_info = flowtag << DESC_FLOWTAG_SHIFT;
	}

	dev_dbg(dev, "%s tx channel: pool %s, descs %d, "
		"channel %d (%p), prio %d, tag %x, submit %d, complete %d\n",
		chan_name(chan), chan->qname_pool, chan->num_descs,
		chan->channel, chan->reg_chan,
		chan->tx_priority, chan->tag_info,
		chan->qcfg_submit, chan->qcfg_complete);

	return 0;
}

static __devinit int dma_init_chan(struct keystone_dma_device *dma,
				   struct device_node *node)
{
	struct device *dev = dma_dev(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	int ret, i;
	u32 queue;

	chan = devm_kzalloc(dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	spin_lock_init(&chan->lock);

	for (i = 0; i < __DESC_STATE_MAX; i++)
	     INIT_LIST_HEAD(&chan->desc_list[i]);

	chan->dma	= dma;
	chan->direction	= DMA_TRANS_NONE;
	chan->state	= CHAN_STATE_INVALID;

	achan = to_achan(chan);
	achan->device = to_dma(dma);

	ret = of_property_read_string(node, "label", &achan->name);
	if (ret < 0)
		achan->name = node->name;
	if (!achan->name)
		achan->name = "unknown";

	ret = of_property_read_string(node, "pool", &chan->qname_pool);
	if (ret < 0) {
		dev_err(dev, "no descriptor pool for %s channel\n",
			chan_name(chan));
		goto fail;
	}

	ret = of_property_read_u32(node, "descriptors", &chan->num_descs);
	if (ret < 0) {
		chan->num_descs = DMA_DEFAULT_NUM_DESCS;
		dev_dbg(dev, "descriptors default to %d for %s channel\n",
			chan->num_descs, chan_name(chan));
	}

	chan->debug = (of_get_property(node, "debug",  NULL) != NULL);
	chan->debug = chan->debug || dma->debug;

	ret = of_property_read_u32(node, "submit-queue", &queue);
	if (ret < 0) {
		dev_dbg(dev, "unspecified submit queue for %s channel\n",
			chan_name(chan));
		queue = HWQUEUE_ANY;
	}
	chan->qcfg_submit = queue;

	ret = of_property_read_u32(node, "complete-queue", &queue);
	if (ret < 0) {
		dev_dbg(dev, "unspecified completion queue for %s channel\n",
			chan_name(chan));
		queue = HWQUEUE_ANY;
	}
	chan->qcfg_complete = queue;

	ret = -EINVAL;
	if (of_find_property(node, "transmit", NULL)) {
		chan->direction = DMA_MEM_TO_DEV;
		ret = dma_init_tx_chan(chan, node);
	} else if (of_find_property(node, "receive", NULL)) {
		chan->direction = DMA_DEV_TO_MEM;
		ret = dma_init_rx_chan(chan, node);
	} else {
		dev_err(dev, "%s channel direction unknown\n", chan_name(chan));
	}

	if (ret < 0)
		goto fail;

	list_add_tail(&to_achan(chan)->device_node, &to_dma(dma)->channels);

	return 0;

fail:
	devm_kfree(dev, chan);
	return ret;
}

static void keystone_dma_hw_init(struct keystone_dma_device *dma)
{
	int i;

	__raw_writel(dma->loopback ? DMA_LOOPBACK : 0,
		     &dma->reg_global->emulation_control);

	if (dma->enable_all) {
		for (i = 0; i < dma->max_rx_chan; i++)
			__raw_writel(DMA_ENABLE, &dma->reg_rx_chan[i].control);

		for (i = 0; i < dma->max_tx_chan; i++) {
			__raw_writel(0, &dma->reg_tx_chan[i].mode);
			__raw_writel(DMA_ENABLE, &dma->reg_tx_chan[i].control);
		}
	}
}

static ssize_t keystone_dma_show_name(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", achan->name);
}

static ssize_t keystone_dma_show_flow_tag(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
				(chan->tag_info >> DESC_FLOWTAG_SHIFT));
}

static ssize_t keystone_dma_store_flow_tag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);
	unsigned long flowtag;

	if (kstrtoul(buf, 0, &flowtag))
		return -EINVAL;

	/* validate the flowtag */
	if (flowtag & ~DESC_FLOWTAG_MASK)
		return -EINVAL;

	chan->tag_info = flowtag << DESC_FLOWTAG_SHIFT;
	return count;
}

static ssize_t keystone_dma_show_chan_num(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->channel);
}

static ssize_t keystone_dma_show_flow(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct dma_chan *achan = dev_to_dma_chan(dev);
	struct keystone_dma_chan *chan = from_achan(achan);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chan->flow);
}

static DEVICE_ATTR(name, S_IRUSR, keystone_dma_show_name, NULL);
static DEVICE_ATTR(chan_num, S_IRUSR, keystone_dma_show_chan_num, NULL);
static DEVICE_ATTR(tx_flow_tag, S_IRUSR | S_IWUSR, \
	 keystone_dma_show_flow_tag, keystone_dma_store_flow_tag);
static DEVICE_ATTR(rx_flow, S_IRUSR, keystone_dma_show_flow, NULL);

static void keystone_dma_destroy_attr(struct keystone_dma_device *dma)
{
	struct dma_device *engine = to_dma(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	struct device *dev;

	list_for_each_entry(achan, &engine->channels, device_node) {
		chan = from_achan(achan);
		dev = chan_dev(chan);

		/* remove sysfs entries */
		device_remove_file(dev, &dev_attr_name);
		device_remove_file(dev, &dev_attr_chan_num);
		if (chan->direction == DMA_TO_DEVICE)
			device_remove_file(dev, &dev_attr_tx_flow_tag);
		else
			device_remove_file(dev, &dev_attr_rx_flow);
	}
}

static int  keystone_dma_setup_attr(struct keystone_dma_device *dma)
{
	struct dma_device *engine = to_dma(dma);
	struct keystone_dma_chan *chan;
	struct dma_chan *achan;
	struct device *dev;
	int status = 0;

	list_for_each_entry(achan, &engine->channels, device_node) {
		chan = from_achan(achan);
		dev = chan_dev(chan);

		/* add sysfs entries */
		status = device_create_file(dev, &dev_attr_name);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file name\n");
		status = device_create_file(dev, &dev_attr_chan_num);
		if (status)
			dev_warn(dev, "Couldn't create sysfs file chan_num\n");
		if (chan->direction == DMA_TO_DEVICE) {
			status = device_create_file(dev, &dev_attr_tx_flow_tag);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file tx_flow_tag\n");
		} else {
			status = device_create_file(dev, &dev_attr_rx_flow);
			if (status)
				dev_warn(dev, "Couldn't create sysfs file tx_flow\n");
		}
	}
	return status;
}

static int __devinit keystone_dma_probe(struct platform_device *pdev)
{
	unsigned max_tx_chan, max_rx_chan, max_rx_flow, max_tx_sched;
	struct device_node *node = pdev->dev.of_node;
	struct keystone_dma_device *dma;
	struct device_node *chans, *chan;
	struct dma_device *engine;
	resource_size_t size;
	int ret, num_chan = 0;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	dma = devm_kzalloc(&pdev->dev, sizeof(*dma), GFP_KERNEL);
	if (!dma) {
		dev_err(&pdev->dev, "could not allocate driver mem\n");
		return -ENOMEM;
	}
	engine = to_dma(dma);
	engine->dev = &pdev->dev;
	platform_set_drvdata(pdev, dma);

	dma->reg_global	 = dma_get_regs(dma, 0, "global", &size);
	if (!dma->reg_global)
		return -ENODEV;
	if (size < sizeof(struct reg_global)) {
		dev_err(dma_dev(dma), "bad size (%d) for global regs\n",
			size);
		return -ENODEV;
	}

	dma->reg_tx_chan = dma_get_regs(dma, 1, "txchan", &size);
	if (!dma->reg_tx_chan)
		return -ENODEV;
	max_tx_chan = size / sizeof(struct reg_chan);

	dma->reg_rx_chan = dma_get_regs(dma, 2, "rxchan", &size);
	if (!dma->reg_rx_chan)
		return -ENODEV;
	max_rx_chan = size / sizeof(struct reg_chan);

	dma->reg_tx_sched = dma_get_regs(dma, 3, "txsched", &size);
	if (!dma->reg_tx_sched)
		return -ENODEV;
	max_tx_sched = size / sizeof(struct reg_tx_sched);

	dma->reg_rx_flow = dma_get_regs(dma, 4, "rxflow", &size);
	if (!dma->reg_rx_flow)
		return -ENODEV;
	max_rx_flow = size / sizeof(struct reg_rx_flow);

	dma->enable_all	= (of_get_property(node, "enable-all", NULL) != NULL);
	dma->big_endian	= (of_get_property(node, "big-endian", NULL) != NULL);
	dma->loopback	= (of_get_property(node, "loop-back",  NULL) != NULL);
	dma->debug	= (of_get_property(node, "debug",  NULL) != NULL);

	dma->max_rx_chan = max_rx_chan;
	dma->max_rx_flow = max_rx_flow;
	dma->max_tx_chan = min(max_tx_chan, max_tx_sched);

	INIT_LIST_HEAD(&engine->channels);

	chans = of_find_child_by_name(node, "channels");
	if (!chans) {
		dev_err(dma_dev(dma), "could not find channels\n");
		return -ENODEV;
	}

	for_each_child_of_node(chans, chan) {
		if (dma_init_chan(dma, chan) >= 0)
			num_chan++;
	}

	of_node_put(chans);

	if (list_empty(&engine->channels)) {
		dev_err(dma_dev(dma), "no valid channels\n");
		return -ENODEV;
	}

	dma_cap_set(DMA_SLAVE, engine->cap_mask);

	engine->device_alloc_chan_resources = chan_init;
	engine->device_free_chan_resources  = chan_destroy;
	engine->device_issue_pending	    = chan_issue_pending;
	engine->device_tx_status	    = chan_xfer_status;
	engine->device_control		    = chan_control;
	engine->device_prep_slave_sg	    = chan_prep_slave_sg;

	ret = dma_async_device_register(engine);
	if (ret) {
		dev_err(&pdev->dev, "unable to register dma engine\n");
		return ret;
	}

	ret = keystone_dma_setup_attr(dma);
	if (ret) {
		dev_err(&pdev->dev, "unable to setup device attr\n");
		return ret;
	}
	keystone_dma_hw_init(dma);

	dev_info(dma_dev(dma), "registered %d logical channels, flows %d, "
		 "tx chans: %d, rx chans: %d%s%s\n", num_chan,
		 dma->max_rx_flow, dma->max_tx_chan, dma->max_rx_chan,
		 dma->big_endian ? ", big-endian" : "",
		 dma->loopback ? ", loopback" : "");

	return 0;
}

static int __devexit keystone_dma_remove(struct platform_device *pdev)
{
	struct keystone_dma_device *dma = platform_get_drvdata(pdev);
	struct dma_device *engine = to_dma(dma);

	keystone_dma_destroy_attr(dma);
	dma_async_device_unregister(engine);

	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-pktdma", },
	{},
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver keystone_dma_driver = {
	.probe	= keystone_dma_probe,
	.remove	= __devexit_p(keystone_dma_remove),
	.driver = {
		.name		= "keystone-pktdma",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
};

static int __init keystone_dma_init(void)
{
	BUILD_CHECK_REGS();
	return platform_driver_register(&keystone_dma_driver);
}
subsys_initcall_sync(keystone_dma_init);

static void __exit keystone_dma_exit(void)
{
	platform_driver_unregister(&keystone_dma_driver);
}
module_exit(keystone_dma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cyril Chemparathy <cyril@ti.com>");
MODULE_DESCRIPTION("TI Keystone Packet DMA driver");
