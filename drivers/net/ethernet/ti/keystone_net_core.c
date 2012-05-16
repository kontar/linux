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
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
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

#define NETCP_NAPI_WEIGHT	128
#define NETCP_TX_TIMEOUT	40
#define NETCP_TX_THRESHOLD	32
#define NETCP_MIN_PACKET_SIZE	64
#define NETCP_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static int netcp_rx_packet_max = NETCP_MAX_PACKET_SIZE;
static int netcp_debug_level;

#define for_each_module(netcp, module)			\
	list_for_each_entry(module, &netcp->modules, list)
#define for_each_module_safe(netcp, module, tmp)	\
	list_for_each_entry_safe(module, tmp, &netcp->modules, list)

struct netcp_packet {
	struct scatterlist		 sg[4];
	int				 sg_ents;
	struct sk_buff			*skb;
	u32				 epib[4];
	u32				 psdata[6];
	struct netcp_priv		*netcp;
	enum dma_status			 status;
	dma_cookie_t			 cookie;
};

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
		p_info, cause, skb->len, p_info->sg[2].length,
		head[0x00], head[0x01], head[0x02], head[0x03],
		head[0x04], head[0x05], head[0x06], head[0x07],
		head[0x08], head[0x09], head[0x0a], head[0x0b],
		head[0x0c], head[0x0d], head[0x0e], head[0x0f],
		tail[0x00], tail[0x01], tail[0x02], tail[0x03],
		tail[0x04], tail[0x05], tail[0x06], tail[0x07],
		tail[0x08], tail[0x09], tail[0x0a], tail[0x0b],
		tail[0x0c], tail[0x0d], tail[0x0e], tail[0x0f]);
}

static void netcp_rx_timestamp(struct netcp_priv *netcp,
			       struct netcp_packet *p_info,
			       struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	struct netcp_module_data *module;
	u64 raw_timestamp = 0;
	u64 rx_timestamp = 0;
	u64 ts[2] = {0, 0};
	u64 sys_time = 0;
	int ret = 0;

	if (!netcp->hwts_rx_en)
		return;

	for_each_module(netcp, module) {
		ret = module->get_timestamp(module, ts);
		if (ret) {
			dev_err(netcp->dev, "PA get timestamp failed\n");
		}
	}

	rx_timestamp = p_info->epib[0];
	rx_timestamp |= (u64)((ts[1] & 0xffff0000) << 16);

	raw_timestamp = ts[0];
	raw_timestamp |= (u64)((ts[1] & 0xffffffff) << 16);

	if (raw_timestamp < rx_timestamp) {
		rx_timestamp = p_info->epib[0];
		rx_timestamp |= (u64)(((ts[1] -1) & 0xffff0000) << 16);
	}

	if (!netcp->pa_calib_cnt)
	{
		netcp->pa_calib_cnt = 1;

		for_each_module(netcp, module)
			netcp->pa_offset = module->get_offset(module);
        }

	/* convert to system time */
	for_each_module(netcp, module)
		sys_time = module->to_sys_time(module,
					      netcp->pa_offset,
					      rx_timestamp);

	sh_hw_tstamps = skb_hwtstamps(skb);
	memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
	sh_hw_tstamps->hwtstamp = ns_to_ktime((rx_timestamp * 100000LL) >> 14);
	sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);
}

static void netcp_rx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	int len = p_info->sg[2].length;
	struct sk_buff *skb;

	netcp->ndev->stats.rx_packets++;
	netcp->ndev->stats.rx_bytes += len;

	p_info->status = dma_async_is_tx_complete(netcp->rx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);
	WARN_ON(netcp->rx_state != RX_STATE_INTERRUPT	&&
		netcp->rx_state != RX_STATE_POLL	&&
		netcp->rx_state != RX_STATE_TEARDOWN);

	dma_unmap_sg(netcp->dev, p_info->sg + 2, 1, DMA_FROM_DEVICE);

	if (unlikely(netcp->rx_state == RX_STATE_TEARDOWN)) {
		dev_dbg(netcp->dev,
			"receive: reclaimed packet %p, status %d, state %s\n",
			p_info, p_info->status, netcp_rx_state_str(netcp));
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		netcp->ndev->stats.rx_dropped++;
		return;
	}
	
	if (unlikely(p_info->status != DMA_SUCCESS)) {
		dev_warn(netcp->dev,
			 "receive: reclaimed packet %p, status %d, state %s\n",
			 p_info, p_info->status, netcp_rx_state_str(netcp));
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		netcp->ndev->stats.rx_errors++;
		return;
	}

	if (unlikely(!len)) {
		dev_warn(netcp->dev, "receive: zero length packet\n");
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		netcp->ndev->stats.rx_errors++;
		return;
	}

	BUG_ON(netcp->rx_state != RX_STATE_POLL);

	skb = p_info->skb;
	p_info->skb = NULL;

	netcp->ndev->last_rx = jiffies;

	netcp_dump_packet(p_info, "rx");

	skb_put(skb, len);

	netcp_rx_timestamp(netcp, p_info, skb);

	/* push skb up the stack */
	skb->protocol = eth_type_trans(skb, netcp->ndev);
	netif_receive_skb(skb);

	kfree(p_info);
}

static void netcp_tx_complete(void *data)
{
	struct netcp_packet *p_info = data;
	struct netcp_priv *netcp = p_info->netcp;
	struct sk_buff *skb = p_info->skb;

	p_info->status = dma_async_is_tx_complete(netcp->tx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

	dma_unmap_sg(netcp->dev, p_info->sg + 2, 1, DMA_TO_DEVICE);

	netcp_dump_packet(p_info, "txc");

	if (p_info->status != DMA_SUCCESS)
		netcp->ndev->stats.tx_errors++;

	dev_kfree_skb_any(skb);
	kfree(p_info);

	if (netif_queue_stopped(netcp->ndev) && netcp_is_alive(netcp))
		netif_wake_queue(netcp->ndev);
}

/* Initialize the queues */
static void netcp_refill_rx(struct netcp_priv *netcp, int packets)
{
	struct dma_async_tx_descriptor *desc;
	struct netcp_packet *p_info;
	struct dma_device *device;
	struct sk_buff *skb;
	u32 err = 0;

	for (;;) {
		if (packets > 0)
			packets --;

		p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
		if (!p_info) {
			dev_err(netcp->dev, "packet alloc failed\n");
			break;
		}

		BUG_ON((((unsigned long)(p_info)) >> PAGE_SHIFT) !=
		       (((unsigned long)(p_info + 1)) >> PAGE_SHIFT));

		p_info->netcp = netcp;

		skb = netdev_alloc_skb(netcp->ndev, netcp->rx_packet_max);

		if (!skb) {
			dev_err(netcp->dev, "skb alloc failed\n");
			kfree(p_info);
			break;
		}

		if (WARN_ON(skb->len)) {
			dev_err(netcp->dev, "refill skb %p len is %d (leaking)\n", skb, skb->len);
			kfree(p_info);
			continue;
		}

		skb->dev = netcp->ndev;
		p_info->skb = skb;

		sg_init_table(p_info->sg, 3);
		sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
		sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
		sg_set_buf(&p_info->sg[2], skb_tail_pointer(skb), skb_tailroom(skb));

		p_info->sg_ents = 2 + dma_map_sg(netcp->dev, p_info->sg + 2,
						 1, DMA_FROM_DEVICE);

		if (p_info->sg_ents != 3) {
			dev_err(netcp->dev, "dma map failed\n");
			dev_kfree_skb_any(skb);
			kfree(p_info);
			break;
		}

		device = netcp->rx_channel->device;

		desc = dmaengine_prep_slave_sg(netcp->rx_channel, p_info->sg,
					       3, DMA_DEV_TO_MEM,
					       DMA_HAS_EPIB | DMA_HAS_PSINFO);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(netcp->dev, p_info->sg + 2, 1,
				     DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(netcp->dev,
					"dma prep failed, error %d\n", err);
			}
			break;
		}

		desc->callback_param = p_info;
		desc->callback = netcp_rx_complete;
		p_info->cookie = dmaengine_submit(desc);

		if (!packets)
			break;
	}
}

/* NAPI poll */
static int netcp_poll(struct napi_struct *napi, int budget)
{
	struct netcp_priv *netcp = container_of(napi, struct netcp_priv, napi);
	unsigned long flags;
	unsigned packets;

	spin_lock_irqsave(&netcp->lock, flags);

	BUG_ON(netcp->rx_state != RX_STATE_SCHEDULED);
	netcp_set_rx_state(netcp, RX_STATE_POLL);

	spin_unlock_irqrestore(&netcp->lock, flags);

	packets = dma_poll(netcp->rx_channel, budget);
	netcp_refill_rx(netcp, packets);

	if (packets < budget) {
		netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);
		napi_complete(&netcp->napi);
		dmaengine_resume(netcp->rx_channel);
	} else {
		netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	}

	return packets;
}

/* Push an outcoming packet */
static int netcp_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct dma_async_tx_descriptor *desc;
	struct netcp_module_data *module;
	unsigned pkt_len = skb->len;
	struct netcp_packet *p_info;
	struct dma_device *device;
	unsigned long flags;
	bool need_poll = 0;
	int ret = 0;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		dev_warn(netcp->dev, "failed to alloc packet info\n");
		ret = -ENOMEM;
		goto out;
	}

	BUG_ON((((unsigned long)(p_info)) >> PAGE_SHIFT) !=
	       (((unsigned long)(p_info + 1)) >> PAGE_SHIFT));

	p_info->netcp = netcp;
	p_info->skb = skb;

	if (unlikely(pkt_len < NETCP_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, NETCP_MIN_PACKET_SIZE);
		if (ret < 0)
			dev_warn(netcp->dev, "padding failed, ignoring\n");
		pkt_len = NETCP_MIN_PACKET_SIZE;
	}

	netcp_dump_packet(p_info, "txs");
#if 0
	if (!netcp->format_tx_cmd) {
		for_each_module(netcp, module) {
			ret = module->send_packet(module, netcp->tx_psdata);
			if (ret) {
				dev_err(netcp->dev, "PA send packet failed\n");
				goto out;
			}

			netcp->format_tx_cmd = 1;
		}
	}

	memcpy(p_info->psdata, netcp->tx_psdata, sizeof(p_info->psdata));
#endif
	sg_init_table(p_info->sg, 3);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], skb->data, pkt_len);

	p_info->sg_ents = 2 + dma_map_sg(netcp->dev, p_info->sg + 2,
					 1, DMA_TO_DEVICE);
	if (p_info->sg_ents != 3) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_warn(netcp->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto out;
	}

	device = netcp->tx_channel->device;

	desc = dmaengine_prep_slave_sg(netcp->tx_channel, p_info->sg, 3,
				       DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		ndev->stats.tx_dropped++;
		dma_unmap_sg(netcp->dev, p_info->sg + 2, 1, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_dbg(netcp->dev, "failed to prep slave dma\n");
		netif_stop_queue(ndev);
		ret = -ENOBUFS;
		goto out;
	}

	desc->callback_param = p_info;
	desc->callback = netcp_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	ndev->trans_start = jiffies;

	ret = NETDEV_TX_OK;

out:
	spin_lock_irqsave(&netcp->lock, flags);

	if (++netcp->tx_packets >= NETCP_TX_THRESHOLD) {
		dev_dbg(netcp->dev, "transmit poll threshold reached\n");
		need_poll = true;
		netcp->tx_packets = 0;
	}

	spin_unlock_irqrestore(&netcp->lock, flags);

	if (need_poll || ret < 0) {
		dev_dbg(netcp->dev, "polling transmit channel\n");
		dma_poll(netcp->tx_channel, -1);
	}

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

struct dma_chan *netcp_get_rx_chan(struct netcp_priv *netcp)
{
	return netcp->rx_channel;
}

struct dma_chan *netcp_get_tx_chan(struct netcp_priv *netcp)
{
	return netcp->tx_channel;
}

static void netcp_rx_notify(struct dma_chan *chan, void *arg)
{
	struct netcp_priv *netcp = arg;

	BUG_ON(netcp->rx_state != RX_STATE_INTERRUPT);
	dmaengine_pause(netcp->rx_channel);
	netcp_set_rx_state(netcp, RX_STATE_SCHEDULED);
	napi_schedule(&netcp->napi);
}

static int match_first_device(struct device *dev, void *data)
{
	return 1;
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
		phy_print_status(netcp->phydev);
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

	dmaengine_pause(netcp->tx_channel);

	name = netcp->rx_chan_name;
	netcp->rx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(netcp->rx_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction = DMA_DEV_TO_MEM;
	err = dmaengine_slave_config(netcp->rx_channel, &config);
	if (err)
		goto fail;

	dma_set_notify(netcp->rx_channel, netcp_rx_notify, netcp);

	dev_dbg(netcp->dev, "opened channels: tx %p, rx %p\n",
		 netcp->tx_channel, netcp->rx_channel);

	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	for_each_module(netcp, module) {
		err = module->open(module, ndev->dev_addr);
		if (err != 0) {
			dev_err(netcp->dev, "Open failed\n");
			goto fail;
		}
	}

	netcp->format_tx_cmd = 0;
	netcp->pa_calib_cnt = 0;
	netcp->pa_offset = 0;

	netcp->phydev = NULL;
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
	} else {
		/* No PHY , fix the link, speed and duplex settings */
		dev_notice(netcp->dev, "no phy, defaulting to 100/full\n");
		netcp->link = 1;
		netcp->speed = SPEED_100;
		netcp->duplex = DUPLEX_FULL;
		keystone_update_phystatus(netcp);
	}

	if (netcp->phydev)
		phy_start(netcp->phydev);

	napi_enable(&netcp->napi);

	netcp_refill_rx(netcp, -1);

	netif_start_queue(ndev);
	netif_carrier_on(ndev);

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
	struct netcp_module_data *module;
	unsigned long flags;
	int err = 0;

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

	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	if (netcp->phydev) {
		phy_stop(netcp->phydev);
		phy_disconnect(netcp->phydev);
		netcp->phydev = NULL;
	}

	for_each_module(netcp, module) {
		err = module->close(module);
		if (err != 0) {
			dev_err(netcp->dev, "Close failed\n");
			goto out;
		}
	}
out:
	dev_dbg(netcp->dev, "netcp device %s stopped\n", ndev->name);

	return 0;
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

	spin_unlock_irqrestore(&netcp->lock, flags);

	napi_disable(&netcp->napi);

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

	netcp->rx_packet_max = max_frame;

	ndev->mtu = new_mtu;

out_change_mtu:
	netcp_set_rx_state(netcp, RX_STATE_INTERRUPT);

	dmaengine_resume(netcp->rx_channel);
	dmaengine_resume(netcp->tx_channel);

	napi_enable(&netcp->napi);

	netcp_refill_rx(netcp, -1);

	netif_start_queue(ndev);
	netif_carrier_on(ndev);	

	return ret;
}

static void netcp_ndo_tx_timeout(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);

	dev_err(netcp->dev, "transmit timed out\n");
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
	resource_size_t size;
	struct resource res;
	void __iomem *efuse;
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
	INIT_LIST_HEAD(&netcp->modules);
	netcp->pdev = pdev;
	netcp->ndev = ndev;
	netcp->dev  = &ndev->dev;
	netcp->msg_enable = netif_msg_init(netcp_debug_level, NETCP_DEBUG);
	netcp->rx_packet_max = netcp_rx_packet_max;
	netcp_set_rx_state(netcp, RX_STATE_INVALID);

	if (of_address_to_resource(node, 1, &res)) {
		dev_err(&pdev->dev, "could not find resource\n");
		ret = -ENODEV;
		goto probe_quit;
	}
	size = resource_size(&res);

	if (!devm_request_mem_region(&pdev->dev, res.start, size,
				     dev_name(netcp->dev))) {
		dev_err(&pdev->dev, "could not reserve resource\n");
		ret = -ENOMEM;
		goto probe_quit;
	}

	efuse = devm_ioremap_nocache(&pdev->dev, res.start, size);
	if (!efuse) {
		dev_err(&pdev->dev, "could not map resource\n");
		ret = -ENOMEM;
		goto probe_quit;
	}

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
		module_data->priv = netcp;
	}

	emac_arch_get_mac_addr(mac_addr, efuse);

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
	struct netcp_priv *netcp = netdev_priv(ndev);
	struct netcp_module_data *module, *tmp;

	for_each_module_safe(netcp, module, tmp)
		module->remove(module);
	free_netdev(ndev);
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
