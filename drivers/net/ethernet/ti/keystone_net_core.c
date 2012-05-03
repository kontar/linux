/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com>
 *	    Sandeep Paulraj <s-paulraj@ti.com>
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
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/net_tstamp.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include <mach/keystone-dma.h>

#include "keystone_net.h"

static LIST_HEAD(netcp_modules);
static DEFINE_MUTEX(netcp_modules_lock);

#define for_each_netcp_module(module)			\
	list_for_each_entry(module, &netcp_modules, list)

int netcp_register_module(struct netcp_module *module)
{
	struct netcp_module *tmp;
	int ret;

	BUG_ON(!module->name);
	BUG_ON(!module->probe);

	mutex_lock(&netcp_modules_lock);

	ret = -EEXIST;
	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, module->name))
			goto found;
	}

	list_add_tail(&module->list, &netcp_modules);

found:
	mutex_unlock(&netcp_modules_lock);
	return ret;
}
EXPORT_SYMBOL(netcp_register_module);

static struct netcp_module *netcp_find_module(const char *name)
{
	struct netcp_module *tmp;
	mutex_lock(&netcp_modules_lock);
	for_each_netcp_module(tmp) {
		if (!strcasecmp(tmp->name, name))
			goto found;
	}
	mutex_unlock(&netcp_modules_lock);
	return NULL;
found:
	mutex_unlock(&netcp_modules_lock);
	return tmp;
}

void netcp_unregister_module(struct netcp_module *module)
{
	if (module == netcp_find_module(module->name)) {
		mutex_lock(&netcp_modules_lock);
		list_del(&module->list);
		mutex_unlock(&netcp_modules_lock);
	}
}
EXPORT_SYMBOL(netcp_unregister_module);

#define NETCP_DEBUG (NETIF_MSG_HW	| NETIF_MSG_WOL		|	\
		    NETIF_MSG_DRV	| NETIF_MSG_LINK	|	\
		    NETIF_MSG_IFUP	| NETIF_MSG_INTR	|	\
		    NETIF_MSG_PROBE	| NETIF_MSG_TIMER	|	\
		    NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	|	\
		    NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	|	\
		    NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	|	\
		    NETIF_MSG_RX_STATUS)

#define NETCP_NAPI_WEIGHT	256
#define NETCP_TX_TIMEOUT	40
#define NETCP_MIN_PACKET_SIZE	64
#define NETCP_MAX_PACKET_SIZE	9504

static int netcp_rx_packet_max = NETCP_MAX_PACKET_SIZE;
static int netcp_debug_level;

#define for_each_module(netcp, module)			\
	list_for_each_entry(module, &netcp->modules, list)

struct netcp_packet {
	struct scatterlist		 sg[4];
	int				 sg_ents;
	struct sk_buff			*skb;
	u32				 swdata[3];
	u32				 psdata[6];
	unsigned int			 timestamp[1];
	struct netcp_priv		*netcp;
	struct list_head		 stash;
	enum dma_status			 status;
	dma_cookie_t			 cookie;
};

#define first_stashed_packet(netcp)				\
	list_first_entry(&(netcp)->stash, struct netcp_packet, stash)

static const char *netcp_rx_state_str(struct netcp_priv *netcp)
{
	static const char * const state_str[] = {
		[RX_STATE_POLL]		= "poll",
		[RX_STATE_SCHEDULED]	= "scheduled",
		[RX_STATE_TEARDOWN]	= "teardown",
		[RX_STATE_INTERRUPT]	= "interrupt",
		[RX_STATE_INVALID]	= "invalid",
	};

	if (netcp->rx_state < 0 || netcp->rx_state >= ARRAY_SIZE(state_str))
		return state_str[RX_STATE_INVALID];
	else
		return state_str[netcp->rx_state];
}

static inline void netcp_set_rx_state(struct netcp_priv *netcp,
				     enum netcp_rx_state state)
{
	netcp->rx_state = state;
	cpu_relax();
}

static inline bool netcp_is_alive(struct netcp_priv *netcp)
{
	return (netcp->rx_state == RX_STATE_POLL ||
		netcp->rx_state == RX_STATE_INTERRUPT);
}

static void netcp_dump_packet(struct netcp_packet *p_info, const char *cause)
{
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	unsigned char *head, *tail;

	head = skb->data;
	tail = skb->data + (skb->len - 16);

	dev_dbg(netcp->dev, "packet %p %s, size %d (%d): "
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x%02x%02x%02x%02x%02x%02x%02x\n",
		p_info, cause, skb->len, p_info->sg[0].length,
		head[0x00], head[0x01], head[0x02], head[0x03],
		head[0x04], head[0x05], head[0x06], head[0x07],
		head[0x08], head[0x09], head[0x0a], head[0x0b],
		head[0x0c], head[0x0d], head[0x0e], head[0x0f],
		tail[0x00], tail[0x01], tail[0x02], tail[0x03],
		tail[0x04], tail[0x05], tail[0x06], tail[0x07],
		tail[0x08], tail[0x09], tail[0x0a], tail[0x0b],
		tail[0x0c], tail[0x0d], tail[0x0e], tail[0x0f]);
}

static struct netcp_packet *netcp_stash_get(struct netcp_priv *netcp)
{
	struct netcp_packet *p_info = NULL;

	BUG_ON(!irqs_disabled());
	if (!list_empty(&netcp->stash)) {
		p_info = first_stashed_packet(netcp);
		list_del(&p_info->stash);
	}
	cpu_relax();

	return p_info;
}

static void netcp_stash_put(struct netcp_priv *netcp,
			    struct netcp_packet *p_info)
{
	BUG_ON(!irqs_disabled());
	list_add_tail(&p_info->stash, &netcp->stash);
	cpu_relax();
}

static void netcp_reclaim_packet(struct netcp_priv *netcp,
				 struct netcp_packet *p_info)
{
	if (netcp->rx_state == RX_STATE_TEARDOWN) {
		dev_dbg(netcp->dev,
			"receive: reclaimed packet %p, status %d, state %s\n",
			p_info, p_info->status, netcp_rx_state_str(netcp));
	} else {
		dev_warn(netcp->dev,
			 "receive: reclaimed packet %p, status %d, state %s\n",
			 p_info, p_info->status, netcp_rx_state_str(netcp));
	}
	dev_kfree_skb_any(p_info->skb);
	kfree(p_info);
}

static u64 pa_to_sys_time(u64 offset, u64 pa_ticks)
{
	return (offset + pa_ticks * 6);
}

static void netcp_tx_timestamp(struct netcp_priv *netcp,
			       struct sk_buff *skb)
{
	struct skb_shared_hwtstamps sh_hw_tstamps;
	u64 raw_timestamp = 0;
	u64 tx_timestamp = 0;
	u64 ts[2] = {0, 0};
	u64 sys_time = 0;

	if (!netcp->hwts_tx_en)
		return;

	get_timestamp(ts);

	tx_timestamp = netcp->tx_timestamp;
	tx_timestamp |= (u64)((ts[1] & 0xffff0000) << 16);

	raw_timestamp = ts[0];
	raw_timestamp |= (u64)((ts[1] & 0xffffffff) << 16);

	if (raw_timestamp < tx_timestamp) {
		tx_timestamp = netcp->tx_timestamp;
		tx_timestamp |= (u64)(((ts[1] - 1) & 0xffff0000) << 16);
	}

        /* convert to system time */
        sys_time = pa_to_sys_time(netcp->pa_offset, tx_timestamp);

	memset(&sh_hw_tstamps, 0, sizeof(sh_hw_tstamps));
	sh_hw_tstamps.hwtstamp = ns_to_ktime(tx_timestamp * 6);
	sh_hw_tstamps.syststamp = ns_to_ktime(sys_time);
	skb_tstamp_tx(skb, &sh_hw_tstamps);

}

static void netcp_rx_timestamp(struct netcp_priv *netcp,
			       struct netcp_packet *p_info,
			       struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	u64 raw_timestamp = 0;
	u64 rx_timestamp = 0;
	u64 ts[2] = {0, 0};
	u64 sys_time = 0;

	if (!netcp->hwts_rx_en)
		return;

	get_timestamp(ts);

	rx_timestamp = p_info->timestamp[0];
	rx_timestamp |= (u64)((ts[1] & 0xffff0000) << 16);

	raw_timestamp = ts[0];
	raw_timestamp |= (u64)((ts[1] & 0xffffffff) << 16);
	
	if (raw_timestamp < rx_timestamp) {
		rx_timestamp = p_info->timestamp[0];
		rx_timestamp |= (u64)(((ts[1] -1) & 0xffff0000) << 16);
	}

	if (++netcp->pa_calib_cnt > 10)
        {
		netcp->pa_calib_cnt = 0;
		netcp->pa_offset = (ktime_to_ns(ktime_get_real()) -
				    pa_to_sys_time(0, rx_timestamp));
        }

	/* convert to system time */
	sys_time = pa_to_sys_time(netcp->pa_offset, rx_timestamp);

	sh_hw_tstamps = skb_hwtstamps(skb);
	memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
	sh_hw_tstamps->hwtstamp = ns_to_ktime(rx_timestamp * 6);
	sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);
}

static unsigned netcp_deliver_stash(struct netcp_priv *netcp,
				    unsigned max_packets)
{
	struct netcp_packet *p_info;
	unsigned packets = 0;
	struct sk_buff *skb;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&netcp->lock, flags);

		p_info = netcp_stash_get(netcp);
		if (!p_info) {
			spin_unlock_irqrestore(&netcp->lock, flags);
			break;
		}

		if (unlikely(p_info->status != DMA_SUCCESS ||
			     netcp->rx_state != RX_STATE_POLL)) {
			spin_unlock_irqrestore(&netcp->lock, flags);
			netcp_reclaim_packet(netcp, p_info);
			continue;
		}

		skb = p_info->skb;
		skb_put(skb, p_info->sg[3].length);

		if (p_info->swdata[0] == 0x12340002)
			netcp->tx_timestamp = p_info->timestamp[0];
		else
			netcp_rx_timestamp(netcp, p_info, skb);

		/* fill statistics */
		netcp->ndev->last_rx = jiffies;
		netcp->ndev->stats.rx_packets++;
		netcp->ndev->stats.rx_bytes += skb->len;

		spin_unlock_irqrestore(&netcp->lock, flags);

		/* cleanup our mappings and memory */
		dma_unmap_sg(netcp->dev, p_info->sg, 4, DMA_FROM_DEVICE);
		netcp_dump_packet(p_info, "rx");
		kfree(p_info);

		/* push skb up the stack */
		skb->protocol = eth_type_trans(skb, netcp->ndev);
		netif_receive_skb(skb);

		if (++packets >= max_packets)
			break;
	}

	return packets;
}

static void netcp_rx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	unsigned long flags;

	spin_lock_irqsave(&netcp->lock, flags);
	
	p_info->status = dma_async_is_tx_complete(netcp->rx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status  != DMA_SUCCESS		&&
		p_info->status  != DMA_ERROR);

	WARN_ON(netcp->rx_state != RX_STATE_INTERRUPT	&&
		netcp->rx_state != RX_STATE_POLL	&&
		netcp->rx_state != RX_STATE_TEARDOWN);

	if (netcp->rx_state == RX_STATE_INTERRUPT) {
		dev_dbg(netcp->dev,
			"receive: packet %p, status %d, scheduling poll\n",
			p_info, p_info->status);
		dmaengine_pause(netcp->rx_channel);
		netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
		napi_schedule(&netcp->napi);
	} else if (netcp->rx_state == RX_STATE_POLL) {
		dev_dbg(netcp->dev,
			"receive: packet %p, status %d, in poll\n",
			p_info, p_info->status);
	} else if (netcp->rx_state == RX_STATE_TEARDOWN) {
		dev_dbg(netcp->dev,
			"receive: packet %p, status %d, in teardowm\n",
			p_info, p_info->status);
	}

	netcp_stash_put(netcp, p_info);

	spin_unlock_irqrestore(&netcp->lock, flags);
}

static void netcp_tx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;
	unsigned long flags;

	spin_lock_irqsave(&netcp->lock, flags);

	netcp_dump_packet(p_info, "tx");

	/* Fill statistics */
	netcp->ndev->stats.tx_packets++;
	netcp->ndev->stats.tx_bytes += skb->len;
	netcp->ndev->trans_start = jiffies;

	if (netif_queue_stopped(netcp->ndev) &&
	    netcp_is_alive(netcp))
		netif_wake_queue(netcp->ndev);

	spin_unlock_irqrestore(&netcp->lock, flags);

	skb_tx_timestamp(skb);
	dev_kfree_skb_any(skb);
	kfree(p_info);
}

/* Initialize the queues */
static int netcp_refill_rx(struct netcp_priv *netcp)
{
	struct dma_async_tx_descriptor *desc;
	struct netcp_packet *p_info;
	struct dma_device *device;
	struct sk_buff *skb;
	int packets = 0;
	u32 err = 0;

	for (;;) {
		p_info = kzalloc(sizeof(*p_info), GFP_KERNEL);
		if (!p_info) {
			dev_err(netcp->dev, "packet alloc failed\n");
			break;
		}
		p_info->netcp = netcp;

		skb = netdev_alloc_skb_ip_align(netcp->ndev,
						netcp->rx_packet_max);
		if (!skb) {
			dev_err(netcp->dev, "skb alloc failed\n");
			kfree(p_info);
			break;
		}
		skb->dev = netcp->ndev;
		p_info->skb = skb;

		sg_init_table(p_info->sg, 4);
		sg_set_buf(&p_info->sg[0], p_info->swdata,
			   sizeof(p_info->swdata));
		sg_set_buf(&p_info->sg[1], p_info->psdata,
			   sizeof(p_info->psdata));
		sg_set_buf(&p_info->sg[2], p_info->timestamp, 4);
		sg_set_buf(&p_info->sg[3], skb->data, skb_tailroom(skb));

		p_info->sg_ents = dma_map_sg(netcp->dev, p_info->sg, 4,
					     DMA_FROM_DEVICE);
		if (p_info->sg_ents != 4) {
			dev_err(netcp->dev, "dma map failed\n");
			dev_kfree_skb_any(skb);
			kfree(p_info);
			break;
		}

		device = netcp->rx_channel->device;

		desc = device->device_prep_slave_sg(netcp->rx_channel,
						    p_info->sg, 4,
						    DMA_DEV_TO_MEM,
						    DMA_HAS_SWINFO |
						    DMA_HAS_PSINFO |
						    DMA_HAS_TIMESTAMP);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(netcp->dev, p_info->sg, 4,
				     DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
				return err;
			}
			break;
		}

		desc->callback_param = p_info;
		desc->callback = netcp_rx_complete;
		p_info->cookie = dmaengine_submit(desc);

		packets++;
	}

	dev_dbg(netcp->dev, "refilled %d packets\n", packets);
	return packets;
}

static int netcp_hwtstamp_ioctl(struct net_device *ndev,
				struct ifreq *ifr, int cmd)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct hwtstamp_config cfg;
	
	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		netcp->hwts_tx_en = 0;
		break;
	case HWTSTAMP_TX_ON:
		netcp->hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		netcp->hwts_rx_en = 0;
		break;
	default:
		netcp->hwts_rx_en = 1;
		cfg.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int match_first_device(struct device *dev, void *data)
{
	return 1;
}

/* NAPI poll */
static int netcp_poll(struct napi_struct *napi, int budget)
{
	struct netcp_priv *netcp = container_of(napi, struct netcp_priv, napi);
	unsigned long flags;
	unsigned packets;

	spin_lock_irqsave(&netcp->lock, flags);

	dev_dbg(netcp->dev, "beginning napi poll processing\n");
	BUG_ON(netcp->rx_state != RX_STATE_SCHEDULED);
	netcp_set_rx_state(netcp, RX_STATE_POLL);

	spin_unlock_irqrestore(&netcp->lock, flags);

	dev_dbg(netcp->dev, "resuming dma engine\n");
	dmaengine_resume(netcp->rx_channel); /* callbacks galore in here */
	packets = netcp_deliver_stash(netcp, budget);

	spin_lock_irqsave(&netcp->lock, flags);

	if (packets < budget) {
		dev_dbg(netcp->dev, "consumed all packets\n");
		napi_complete(napi);
		netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);
	} else {
		dev_dbg(netcp->dev, "out of budget\n");
		dmaengine_pause(netcp->rx_channel);
		netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	}

	spin_unlock_irqrestore(&netcp->lock, flags);

	netcp_refill_rx(netcp);

	return packets;
}

/* Push an outcoming packet */
static int netcp_ndo_start_xmit(struct sk_buff *skb,
			       struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct dma_async_tx_descriptor *desc;
	struct netcp_module_data *module;
	unsigned pkt_len = skb->len;
	struct netcp_packet *p_info;
	struct dma_device *device;	
	int ret = 0;
	int pa_ret = 0;

	p_info = kzalloc(sizeof(*p_info), GFP_KERNEL);
	if (!p_info) {
		ret = -ENOMEM;
		goto out;
	}
	memset(p_info, 0, sizeof(*p_info));
	p_info->netcp = netcp;
	p_info->skb = skb;

	if (unlikely(pkt_len < NETCP_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, NETCP_MIN_PACKET_SIZE);
		if (ret < 0)
			dev_warn(netcp->dev, "padding failed, ignoring\n");
		pkt_len = NETCP_MIN_PACKET_SIZE;
	}
#if 0
	if (netcp->format_tx_cmd == 0) {
		for_each_module(netcp, module) {
			pa_ret = module->send_packet(module, netcp->tx_psdata);
			if (pa_ret) {
				dev_err(netcp->dev, "PA send packet failed\n");
				goto out;
			}
		}
		netcp->format_tx_cmd = 1;
	}
		
	memcpy(p_info->psdata, netcp->tx_psdata,
		       sizeof(p_info->psdata));

	printk("PSDATA 0 = 0x%x\n", p_info->psdata[0]);
	printk("PSDATA 1 = 0x%x\n", p_info->psdata[1]);
	printk("PSDATA 2 = 0x%x\n", p_info->psdata[2]);
	printk("PSDATA 3 = 0x%x\n", p_info->psdata[3]);
	printk("PSDATA 4 = 0x%x\n", p_info->psdata[4]);
#endif
	
	p_info->psdata[0] = 0xc01e0291;
	p_info->psdata[1] = 0x12340002;
	p_info->psdata[2] = 0x67000000;
	p_info->psdata[3] = 0x00000000;
	p_info->psdata[4] = 0x00000000;

	sg_init_table(p_info->sg, 4);
	sg_set_buf(&p_info->sg[0], p_info->swdata, sizeof(p_info->swdata));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->timestamp, 4);
	sg_set_buf(&p_info->sg[3], skb->data, pkt_len);

	p_info->sg_ents = dma_map_sg(netcp->dev, p_info->sg, 4, DMA_TO_DEVICE);
	if (p_info->sg_ents != 4) {
		dev_kfree_skb_any(skb);
		kfree(p_info);
		ret = -ENOMEM;
		goto out;
	}

	device = netcp->tx_channel->device;

	dev_dbg(netcp->dev, "tx prep, channel %p\n", netcp->tx_channel);

	desc = device->device_prep_slave_sg(netcp->tx_channel,
					    p_info->sg, 4, DMA_MEM_TO_DEV,
					    DMA_HAS_SWINFO |
					    DMA_HAS_PSINFO |
					    DMA_HAS_TIMESTAMP);

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(netcp->dev, p_info->sg, 4, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		kfree(p_info);

		dev_err(netcp->dev, "packet dropped\n");
		ndev->stats.tx_dropped++;
		ndev->trans_start = jiffies; /* TODO: check if this should be
						incremented only on success */
		netif_stop_queue(ndev);
		ret = NETDEV_TX_BUSY;
		goto out;
	}
	
	netcp_tx_timestamp(netcp, skb);

	desc->callback_param = p_info;
	desc->callback = netcp_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	ret = NETDEV_TX_OK;
out:
	return ret;
}

/* Change receive flags */
static void netcp_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

static void keystone_update_phystatus(struct netcp_priv *netcp)
{
	struct net_device *ndev = netcp->ndev;

	if (netcp->link) {
		/* link ON */
		if (!netif_carrier_ok(ndev))
			netif_carrier_on(ndev);
		/*
		 * reactivate the transmit queue if
		 * it is stopped
		 */
		if (netif_running(ndev) && netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
	} else {
		/* link OFF */
		if (netif_carrier_ok(ndev))
			netif_carrier_off(ndev);
		if (!netif_queue_stopped(ndev))
			netif_stop_queue(ndev);
	}
}

static void keystone_adjust_link(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct phy_device *phydev = netcp->phydev;
	unsigned long flags;
	int new_state = 0;

	spin_lock_irqsave(&netcp->lock, flags);

	if (phydev->link) {
		/* check the mode of operation - full/half duplex */
		if (phydev->duplex != netcp->duplex) {
			new_state = 1;
			netcp->duplex = phydev->duplex;
		}
		if (phydev->speed != netcp->speed) {
			new_state = 1;
			netcp->speed = phydev->speed;
		}
		if (!netcp->link) {
			new_state = 1;
			netcp->link = 1;
		}

	} else if (netcp->link) {
		new_state = 1;
		netcp->link = 0;
		netcp->speed = 0;
		netcp->duplex = ~0;
	}
	if (new_state) {
		keystone_update_phystatus(netcp);
#ifndef KEYSTONE_NET_SIMULATION
		phy_print_status(netcp->phydev);
#endif
	}

	spin_unlock_irqrestore(&netcp->lock, flags);
}

/* Open the device */
static int netcp_ndo_open(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module;
	struct dma_slave_config config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;

	netif_carrier_off(ndev);

	streaming_switch_setup();

	for_each_module(netcp, module) {
		err = module->open(module);
		if (err != 0) {
			dev_err(netcp->dev, "Open failed\n");
			goto fail;
		}
	}

	evm_pa_ss_init();

	ethss_start();

	BUG_ON(netcp->rx_state != RX_STATE_INVALID);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	name = netcp->tx_chan_name;
	netcp->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(netcp->tx_channel))
		goto fail;
	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	err = dmaengine_slave_config(netcp->tx_channel, &config);
	if (err)
		goto fail;

	name = netcp->rx_chan_name;
	netcp->rx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(netcp->rx_channel))
		goto fail;
	memset(&config, 0, sizeof(config));
	config.direction = DMA_DEV_TO_MEM;
	err = dmaengine_slave_config(netcp->rx_channel, &config);
	if (err)
		goto fail;

	dev_dbg(netcp->dev, "opened channels: tx %p, rx %p\n",
		 netcp->tx_channel, netcp->rx_channel);

	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	netcp->format_tx_cmd = 0;
	netcp->pa_calib_cnt = 0;
	netcp->pa_offset = 0;

	netcp->phydev = NULL;
#ifndef KEYSTONE_NET_SIMULATION
	/* use the first phy on the bus if pdata did not give us a phy id */
	if (!netcp->phy_id) {
		struct device *phy;

		phy = bus_find_device(&mdio_bus_type, NULL, NULL,
				      match_first_device);
		if (phy)
			netcp->phy_id = dev_name(phy);
	}

	if (netcp->phy_id && *netcp->phy_id) {
		netcp->phydev = phy_connect(ndev, netcp->phy_id,
					   &keystone_adjust_link, 0,
					   PHY_INTERFACE_MODE_SGMII);

		if (IS_ERR(netcp->phydev)) {
			dev_err(netcp->dev, "could not connect to phy %s\n",
				netcp->phy_id);
			netcp->phydev = NULL;
			return PTR_ERR(netcp->phydev);
		}

		netcp->link = 0;
		netcp->speed = 0;
		netcp->duplex = ~0;

		dev_info(netcp->dev, "attached PHY driver [%s] "
			"(sgmii_bus:phy_addr=%s, id=%x)\n",
			netcp->phydev->drv->name, dev_name(&netcp->phydev->dev),
			netcp->phydev->phy_id);
	} else
#endif
	{
		/* No PHY , fix the link, speed and duplex settings */
		dev_notice(netcp->dev, "no phy, defaulting to 100/full\n");
		netcp->link = 1;
		netcp->speed = SPEED_100;
		netcp->duplex = DUPLEX_FULL;
		keystone_update_phystatus(netcp);
	}

#ifndef KEYSTONE_NET_SIMULATION
	if (netcp->phydev)
		phy_start(netcp->phydev);
#endif

	napi_enable(&netcp->napi);

	netcp_refill_rx(netcp);

	netif_carrier_on(ndev);
	if (netif_queue_stopped(ndev))
		netif_start_queue(ndev);

	for_each_module(netcp, module) {
		err = module->add_mac(module, ndev->dev_addr,
			dma_get_rx_flow(netcp->rx_channel),
			dma_get_rx_queue(netcp->rx_channel));
		if (err != 0) {
			dev_err(netcp->dev, "Add Mac failed\n");
			goto fail;
		}
	}

	dev_info(netcp->dev, "netcp device %s opened\n", ndev->name);

	return 0;
fail:
	if (netcp->tx_channel) {
		dma_release_channel(netcp->tx_channel);
		netcp->tx_channel = NULL;
	}

	if (netcp->rx_channel) {
		dma_release_channel(netcp->rx_channel);
		netcp->rx_channel = NULL;
	}
	return err;
}

/* Close the device */
static int netcp_ndo_stop(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&netcp->lock, flags);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	BUG_ON(!netcp_is_alive(netcp));

	netcp_set_rx_state(netcp, RX_STATE_TEARDOWN);

	dmaengine_pause(netcp->tx_channel);
	dmaengine_pause(netcp->rx_channel);

	spin_unlock_irqrestore(&netcp->lock, flags);

	napi_disable(&netcp->napi);

	if (netcp->tx_channel) {
		dma_release_channel(netcp->tx_channel);
		netcp->tx_channel = NULL;
	}

	if (netcp->rx_channel) {
		dma_release_channel(netcp->rx_channel);
		netcp->rx_channel = NULL;
	}

	netcp_deliver_stash(netcp, -1);

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

#ifndef KEYSTONE_NET_SIMULATION
	if (netcp->phydev)
		phy_disconnect(netcp->phydev);
#endif
	ethss_stop();

	dev_dbg(netcp->dev, "netcp device %s stopped\n", ndev->name);

	return 0;
}

static int netcp_ndo_ioctl(struct net_device *ndev,
			   struct ifreq *req, int cmd)
{
	if (!netif_running(ndev))
		return -EINVAL;

	if (cmd == SIOCSHWTSTAMP)
		return netcp_hwtstamp_ioctl(ndev, req, cmd);
	
	return 0;
}

static int netcp_ndo_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	int old_max_frame = ndev->mtu + ETH_HLEN + ETH_FCS_LEN;
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&netcp->lock, flags);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	BUG_ON(!netcp_is_alive(netcp));

	netcp_set_rx_state(netcp, RX_STATE_TEARDOWN);

	dmaengine_pause(netcp->tx_channel);
	dmaengine_pause(netcp->rx_channel);

	napi_disable(&netcp->napi);

	netcp_deliver_stash(netcp, -1);

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	/* MTU < 68 is an error for IPv4 traffic, just don't allow it */
	if ((new_mtu < 68) ||
	    (max_frame > 9504)) {
		dev_err(netcp->dev, "Invalid mtu size = %d\n", new_mtu);
		ret = -EINVAL;
		goto out_change_mtu;
	}

	if (old_max_frame == max_frame) {
		ret = 0;
		goto out_change_mtu;
	}

	netcp->rx_buffer_len = max_frame;

	ndev->mtu = new_mtu;

out_change_mtu:
	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	dmaengine_resume(netcp->rx_channel);
	dmaengine_resume(netcp->tx_channel);

	napi_enable(&netcp->napi);

	netcp_refill_rx(netcp);

	netif_carrier_on(ndev);
	netif_start_queue(ndev);

	spin_unlock_irqrestore(&netcp->lock, flags);
	return ret;
}

static void netcp_ndo_tx_timeout(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);

	dev_err(netcp->dev, "transmit timed out\n");
	ndev->stats.tx_errors++;
	ndev->trans_start = jiffies;
	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static const struct net_device_ops netcp_netdev_ops = {
	.ndo_open		= netcp_ndo_open,
	.ndo_stop		= netcp_ndo_stop,
	.ndo_start_xmit		= netcp_ndo_start_xmit,
	.ndo_change_rx_flags	= netcp_ndo_change_rx_flags,
	.ndo_do_ioctl           = netcp_ndo_ioctl,
	.ndo_change_mtu		= netcp_ndo_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= netcp_ndo_tx_timeout,
};

static const char *netcp_node_name(struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = node->name;
	if (!name)
		name = "unknown";
	return name;
}

static int __devinit netcp_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child;
	struct netcp_module *module;
	struct netcp_module_data *module_data;
	struct netcp_priv *netcp;
	struct net_device *ndev;
	const char *name;
	u8 mac_addr[6];
	int ret = 0;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	ndev = alloc_etherdev(sizeof(struct netcp_priv));
	if (!ndev) {
		dev_err(&pdev->dev, "Error allocating net_device\n");
		ret = -ENOMEM;
		goto probe_quit;
	}

	platform_set_drvdata(pdev, ndev);
	netcp = netdev_priv(ndev);
	spin_lock_init(&netcp->lock);
	INIT_LIST_HEAD(&netcp->stash);
	INIT_LIST_HEAD(&netcp->modules);
	netcp->pdev = pdev;
	netcp->ndev = ndev;
	netcp->dev  = &ndev->dev;
	netcp->msg_enable = netif_msg_init(netcp_debug_level, NETCP_DEBUG);
	netcp->rx_packet_max = netcp_rx_packet_max;
	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	ret = of_property_read_string(node, "tx_channel", &netcp->tx_chan_name);
	if (ret < 0)
		netcp->tx_chan_name = "nettx";

	ret = of_property_read_string(node, "rx_channel", &netcp->rx_chan_name);
	if (ret < 0)
		netcp->rx_chan_name = "netrx";

	for_each_child_of_node(node, child) {
		name = netcp_node_name(child);
		module = netcp_find_module(name);
		if (!module) {
			dev_err(&pdev->dev, "Could not find module %s\n", name);
			goto clean_ndev_ret;
		}
		module_data = module->probe(&pdev->dev, child);
		if (IS_ERR_OR_NULL(module_data)) {
			dev_err(&pdev->dev, "Probe of module %s failed\n", name);
			goto clean_ndev_ret;
		}
		list_add_tail(&module_data->list, &netcp->modules);
	}

	ethss_init(&pdev->dev);
	ethss_stop();

	emac_arch_get_mac_addr(mac_addr);

	if (is_valid_ether_addr(mac_addr))
		memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
	else
		random_ether_addr(ndev->dev_addr);

	ether_setup(ndev);

	/* NAPI register */
	netif_napi_add(ndev, &netcp->napi, netcp_poll, NETCP_NAPI_WEIGHT);

	/* Register the network device */
	ndev->dev_id		= 0;
	ndev->watchdog_timeo	= NETCP_TX_TIMEOUT;
	ndev->netdev_ops	= &netcp_netdev_ops;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	keystone_set_ethtool_ops(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(netcp->dev, "Error registering net device\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
probe_quit:
	return ret;
}

static int __devexit netcp_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	free_netdev(ndev);
	ethss_destroy();
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-netcp", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver netcp_driver = {
	.driver = {
		.name		= "keystone-netcp",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe = netcp_probe,
	.remove = __devexit_p(netcp_remove),
};

static int __init netcp_init(void)
{
	return platform_driver_register(&netcp_driver);
}
module_init(netcp_init);

static void __exit netcp_exit(void)
{
	platform_driver_unregister(&netcp_driver);
}
module_exit(netcp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Ethernet driver");
