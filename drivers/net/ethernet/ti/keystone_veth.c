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
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include <mach/keystone-dma.h>

#define VETH_DRIVER_NAME	"TI KeyStone Virtual Ethernet Driver"
#define VETH_DRIVER_VERSION	"v1.0"

#define VETH_DEBUG (NETIF_MSG_HW	| NETIF_MSG_WOL		|	\
		    NETIF_MSG_DRV	| NETIF_MSG_LINK	|	\
		    NETIF_MSG_IFUP	| NETIF_MSG_INTR	|	\
		    NETIF_MSG_PROBE	| NETIF_MSG_TIMER	|	\
		    NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	|	\
		    NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	|	\
		    NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	|	\
		    NETIF_MSG_RX_STATUS)

#define VETH_NAPI_WEIGHT	128
#define VETH_TX_TIMEOUT		40
#define VETH_TX_THRESHOLD	32
#define VETH_MIN_PACKET_SIZE	64
#define VETH_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

static int veth_rx_packet_max = VETH_MAX_PACKET_SIZE;
static int veth_debug_level;

enum veth_rx_state {
	RX_STATE_INTERRUPT,
	RX_STATE_SCHEDULED,
	RX_STATE_POLL,
	RX_STATE_TEARDOWN,
	RX_STATE_INVALID,
};

struct veth_priv {
	spinlock_t			 lock;
	struct platform_device		*pdev;
	struct net_device		*ndev;
	struct napi_struct		 napi;
	struct device			*dev;
	u32				 msg_enable;
	struct net_device_stats		 stats;
	int				 rx_packet_max;

	struct dma_chan			*tx_channel;
	struct dma_chan			*rx_channel;
	const char			*tx_chan_name;
	const char			*rx_chan_name;

	int				 rx_buffer_len;
	int				 tx_packets;

	enum veth_rx_state		 rx_state;
};

struct veth_packet {
	struct scatterlist		 sg[4];
	int				 sg_ents;
	struct sk_buff			*skb;
	struct veth_priv		*veth;
	enum dma_status			 status;
	dma_cookie_t			 cookie;
};

static const char *veth_rx_state_str(struct veth_priv *veth)
{
	static const char * const state_str[] = {
		[RX_STATE_POLL]		= "poll",
		[RX_STATE_SCHEDULED]	= "scheduled",
		[RX_STATE_TEARDOWN]	= "teardown",
		[RX_STATE_INTERRUPT]	= "interrupt",
		[RX_STATE_INVALID]	= "invalid",
	};

	if (veth->rx_state < 0 || veth->rx_state >= ARRAY_SIZE(state_str))
		return state_str[RX_STATE_INVALID];
	else
		return state_str[veth->rx_state];
}

static inline void veth_set_rx_state(struct veth_priv *veth,
				     enum veth_rx_state state)
{
	veth->rx_state = state;
	cpu_relax();
}

static inline bool veth_is_alive(struct veth_priv *veth)
{
	return (veth->rx_state == RX_STATE_POLL ||
		veth->rx_state == RX_STATE_INTERRUPT);
}

static void veth_dump_packet(struct veth_packet *p_info, const char *cause)
{
	struct veth_priv *veth = p_info->veth;
	struct sk_buff *skb = p_info->skb;
	unsigned char *head, *tail;

	head = skb->data;
	tail = skb->data + (skb->len - 16);

	dev_dbg(veth->dev, "packet %p %s, size %d (%d): "
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

static void veth_rx_complete(void *data)
{
	struct veth_packet *p_info = data;
	struct veth_priv *veth = p_info->veth;
	int len = p_info->sg[0].length;
	struct sk_buff *skb;

	veth->ndev->stats.rx_packets++;
	veth->ndev->stats.rx_bytes += len;

	p_info->status = dma_async_is_tx_complete(veth->rx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);
	WARN_ON(veth->rx_state != RX_STATE_INTERRUPT	&&
		veth->rx_state != RX_STATE_POLL	&&
		veth->rx_state != RX_STATE_TEARDOWN);

	dma_unmap_sg(veth->dev, p_info->sg, 1, DMA_FROM_DEVICE);

	if (unlikely(veth->rx_state == RX_STATE_TEARDOWN)) {
		dev_dbg(veth->dev,
			"receive: reclaimed packet %p, status %d, state %s\n",
			p_info, p_info->status, veth_rx_state_str(veth));
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		veth->ndev->stats.rx_dropped++;
		return;
	}
	
	if (unlikely(p_info->status != DMA_SUCCESS)) {
		dev_warn(veth->dev,
			 "receive: reclaimed packet %p, status %d, state %s\n",
			 p_info, p_info->status, veth_rx_state_str(veth));
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		veth->ndev->stats.rx_errors++;
		return;
	}

	if (unlikely(!len)) {
		dev_warn(veth->dev, "receive: zero length packet\n");
		dev_kfree_skb_any(p_info->skb);
		kfree(p_info);
		veth->ndev->stats.rx_errors++;
		return;
	}

	BUG_ON(veth->rx_state != RX_STATE_POLL);

	skb = p_info->skb;
	p_info->skb = NULL;

	veth->ndev->last_rx = jiffies;

	veth_dump_packet(p_info, "rx");

	skb_put(skb, len);

	/* push skb up the stack */
	skb->protocol = eth_type_trans(skb, veth->ndev);
	netif_receive_skb(skb);

	kfree(p_info);
}

static void veth_tx_complete(void *data)
{
	struct veth_packet *p_info = data;
	struct veth_priv *veth = p_info->veth;
	struct sk_buff *skb = p_info->skb;

	p_info->status = dma_async_is_tx_complete(veth->tx_channel,
						  p_info->cookie, NULL, NULL);
	WARN_ON(p_info->status != DMA_SUCCESS && p_info->status != DMA_ERROR);

	dma_unmap_sg(veth->dev, p_info->sg, 1, DMA_TO_DEVICE);

	veth_dump_packet(p_info, "txc");

	if (p_info->status != DMA_SUCCESS)
		veth->ndev->stats.tx_errors++;

	dev_kfree_skb_any(skb);
	kfree(p_info);

	if (netif_queue_stopped(veth->ndev) && veth_is_alive(veth))
		netif_wake_queue(veth->ndev);
}

/* Initialize the queues */
static void veth_refill_rx(struct veth_priv *veth, int packets)
{
	struct dma_async_tx_descriptor *desc;
	struct veth_packet *p_info;
	struct dma_device *device;
	struct sk_buff *skb;
	u32 err = 0;

	for (;;) {
		if (packets > 0)
			packets --;

		p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
		if (!p_info) {
			dev_err(veth->dev, "packet alloc failed\n");
			break;
		}

		BUG_ON((((unsigned long)(p_info)) >> PAGE_SHIFT) !=
		       (((unsigned long)(p_info + 1)) >> PAGE_SHIFT));

		p_info->veth = veth;

		skb = netdev_alloc_skb(veth->ndev, veth->rx_packet_max);

		if (!skb) {
			dev_err(veth->dev, "skb alloc failed\n");
			kfree(p_info);
			break;
		}

		if (WARN_ON(skb->len)) {
			dev_err(veth->dev, "refill skb %p len is %d (leaking)\n", skb, skb->len);
			kfree(p_info);
			continue;
		}

		skb->dev = veth->ndev;
		p_info->skb = skb;

		sg_init_table(p_info->sg, 1);
		sg_set_buf(&p_info->sg[0], skb_tail_pointer(skb), skb_tailroom(skb));

		p_info->sg_ents = dma_map_sg(veth->dev, p_info->sg, 1, DMA_FROM_DEVICE);

		if (p_info->sg_ents != 1) {
			dev_err(veth->dev, "dma map failed\n");
			dev_kfree_skb_any(skb);
			kfree(p_info);
			break;
		}

		device = veth->rx_channel->device;

		desc = dmaengine_prep_slave_sg(veth->rx_channel, p_info->sg, 1,
					       DMA_DEV_TO_MEM, 0);
		if (IS_ERR_OR_NULL(desc)) {
			dma_unmap_sg(veth->dev, p_info->sg, 1, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			kfree(p_info);
			err = PTR_ERR(desc);
			if (err != -ENOMEM) {
				dev_err(veth->dev,
					"dma prep failed, error %d\n", err);
			}
			break;
		}

		desc->callback_param = p_info;
		desc->callback = veth_rx_complete;
		p_info->cookie = dmaengine_submit(desc);

		if (!packets)
			break;
	}
}

/* NAPI poll */
static int veth_poll(struct napi_struct *napi, int budget)
{
	struct veth_priv *veth = container_of(napi, struct veth_priv, napi);
	unsigned long flags;
	unsigned packets;

	spin_lock_irqsave(&veth->lock, flags);

	BUG_ON(veth->rx_state != RX_STATE_SCHEDULED);
	veth_set_rx_state(veth, RX_STATE_POLL);

	spin_unlock_irqrestore(&veth->lock, flags);

	packets = dma_poll(veth->rx_channel, budget);
	veth_refill_rx(veth, packets);

	if (packets < budget) {
		veth_set_rx_state(veth, RX_STATE_INTERRUPT);
		napi_complete(&veth->napi);
		dmaengine_resume(veth->rx_channel);
	} else {
		veth_set_rx_state(veth, RX_STATE_SCHEDULED);
	}

	return packets;
}

/* Push an outcoming packet */
static int veth_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct veth_priv *veth = netdev_priv(ndev);
	struct dma_async_tx_descriptor *desc;
	unsigned pkt_len = skb->len;
	struct veth_packet *p_info;
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
		dev_warn(veth->dev, "failed to alloc packet info\n");
		ret = -ENOMEM;
		goto out;
	}

	BUG_ON((((unsigned long)(p_info)) >> PAGE_SHIFT) !=
	       (((unsigned long)(p_info + 1)) >> PAGE_SHIFT));

	p_info->veth = veth;
	p_info->skb = skb;

	if (unlikely(pkt_len < VETH_MIN_PACKET_SIZE)) {
		ret = skb_padto(skb, VETH_MIN_PACKET_SIZE);
		if (ret < 0)
			dev_warn(veth->dev, "padding failed, ignoring\n");
		pkt_len = VETH_MIN_PACKET_SIZE;
	}

	veth_dump_packet(p_info, "txs");

	sg_init_table(p_info->sg, 1);
	sg_set_buf(&p_info->sg[0], skb->data, pkt_len);

	p_info->sg_ents = dma_map_sg(veth->dev, p_info->sg, 1, DMA_TO_DEVICE);

	if (p_info->sg_ents != 1) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_warn(veth->dev, "failed to map transmit packet\n");
		ret = -ENXIO;
		goto out;
	}

	device = veth->tx_channel->device;

	desc = dmaengine_prep_slave_sg(veth->tx_channel, p_info->sg, 1,
				       DMA_MEM_TO_DEV, 0);

	if (IS_ERR_OR_NULL(desc)) {
		ndev->stats.tx_dropped++;
		dma_unmap_sg(veth->dev, p_info->sg, 1, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		kfree(p_info);
		dev_dbg(veth->dev, "failed to prep slave dma\n");
		netif_stop_queue(ndev);
		ret = -ENOBUFS;
		goto out;
	}

	desc->callback_param = p_info;
	desc->callback = veth_tx_complete;
	p_info->cookie = dmaengine_submit(desc);

	ndev->trans_start = jiffies;

	ret = NETDEV_TX_OK;

out:
	spin_lock_irqsave(&veth->lock, flags);

	if (++veth->tx_packets >= VETH_TX_THRESHOLD) {
		dev_dbg(veth->dev, "transmit poll threshold reached\n");
		need_poll = true;
		veth->tx_packets = 0;
	}

	spin_unlock_irqrestore(&veth->lock, flags);

	if (need_poll || ret < 0) {
		dev_dbg(veth->dev, "polling transmit channel\n");
		dma_poll(veth->tx_channel, -1);
	}

	return ret;
}

/* Change receive flags */
static void veth_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}

struct dma_chan *veth_get_rx_chan(struct veth_priv *veth)
{
	return veth->rx_channel;
}

struct dma_chan *veth_get_tx_chan(struct veth_priv *veth)
{
	return veth->tx_channel;
}

static void veth_rx_notify(struct dma_chan *chan, void *arg)
{
	struct veth_priv *veth = arg;

	BUG_ON(veth->rx_state != RX_STATE_INTERRUPT);
	dmaengine_pause(veth->rx_channel);
	veth_set_rx_state(veth, RX_STATE_SCHEDULED);
	napi_schedule(&veth->napi);
}

/* Open the device */
static int veth_ndo_open(struct net_device *ndev)
{
	struct veth_priv *veth = netdev_priv(ndev);
	struct dma_slave_config config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;

	netif_carrier_off(ndev);

	BUG_ON(veth->rx_state != RX_STATE_INVALID);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	name = veth->tx_chan_name;
	veth->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(veth->tx_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	err = dmaengine_slave_config(veth->tx_channel, &config);
	if (err)
		goto fail;

	dmaengine_pause(veth->tx_channel);

	name = veth->rx_chan_name;
	veth->rx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(veth->rx_channel))
		goto fail;

	memset(&config, 0, sizeof(config));
	config.direction = DMA_DEV_TO_MEM;
	err = dmaengine_slave_config(veth->rx_channel, &config);
	if (err)
		goto fail;

	dma_set_notify(veth->rx_channel, veth_rx_notify, veth);

	dev_dbg(veth->dev, "opened channels: tx %p, rx %p\n",
		 veth->tx_channel, veth->rx_channel);

	veth_set_rx_state(veth, RX_STATE_INTERRUPT);

	napi_enable(&veth->napi);

	veth_refill_rx(veth, -1);

	netif_start_queue(ndev);
	netif_carrier_on(ndev);

	dev_info(veth->dev, "veth device %s opened\n", ndev->name);

	return 0;
fail:
	if (veth->tx_channel) {
		dma_release_channel(veth->tx_channel);
		veth->tx_channel = NULL;
	}

	if (veth->rx_channel) {
		dma_release_channel(veth->rx_channel);
		veth->rx_channel = NULL;
	}
	return err;
}

/* Close the device */
static int veth_ndo_stop(struct net_device *ndev)
{
	struct veth_priv *veth = netdev_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&veth->lock, flags);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	BUG_ON(!veth_is_alive(veth));

	veth_set_rx_state(veth, RX_STATE_TEARDOWN);

	dmaengine_pause(veth->tx_channel);
	dmaengine_pause(veth->rx_channel);

	spin_unlock_irqrestore(&veth->lock, flags);

	napi_disable(&veth->napi);

	if (veth->tx_channel) {
		dma_release_channel(veth->tx_channel);
		veth->tx_channel = NULL;
	}

	if (veth->rx_channel) {
		dma_release_channel(veth->rx_channel);
		veth->rx_channel = NULL;
	}

	veth_set_rx_state(veth, RX_STATE_INVALID);

	dev_dbg(veth->dev, "veth device %s stopped\n", ndev->name);

	return 0;
}

static void veth_ndo_tx_timeout(struct net_device *ndev)
{
	struct veth_priv *veth = netdev_priv(ndev);

	dev_err(veth->dev, "transmit timed out\n");
	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static void veth_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strcpy(info->driver, VETH_DRIVER_NAME);
	strcpy(info->version, VETH_DRIVER_VERSION);
}

static u32 veth_get_msglevel(struct net_device *ndev)
{
	struct veth_priv *veth = netdev_priv(ndev);
	return veth->msg_enable;
}

static void veth_set_msglevel(struct net_device *ndev, u32 value)
{
	struct veth_priv *veth = netdev_priv(ndev);
	veth->msg_enable = value;
}

static int veth_get_sset_count(struct net_device *netdev, int stringset)
{
	switch (stringset) {
	case ETH_SS_TEST:
	case ETH_SS_STATS:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct ethtool_ops veth_ethtool_ops = {
	.get_drvinfo		= veth_get_drvinfo,
	.get_msglevel		= veth_get_msglevel,
	.set_msglevel		= veth_set_msglevel,
	.get_sset_count		= veth_get_sset_count,
};

static const struct net_device_ops veth_netdev_ops = {
	.ndo_open		= veth_ndo_open,
	.ndo_stop		= veth_ndo_stop,
	.ndo_start_xmit		= veth_ndo_start_xmit,
	.ndo_change_rx_flags	= veth_ndo_change_rx_flags,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= veth_ndo_tx_timeout,
};

static int __devinit veth_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct net_device *ndev;
	struct veth_priv *veth;
	u8 mac_addr[6];
	int ret = 0;

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	ndev = alloc_netdev_mqs(sizeof(struct veth_priv), "veth%d",
				ether_setup, 1, 1);
	if (!ndev) {
		dev_err(&pdev->dev, "Error allocating net_device\n");
		ret = -ENOMEM;
		goto probe_quit;
	}

	platform_set_drvdata(pdev, ndev);
	veth = netdev_priv(ndev);
	spin_lock_init(&veth->lock);
	veth->pdev = pdev;
	veth->ndev = ndev;
	veth->dev  = &ndev->dev;
	veth->msg_enable = netif_msg_init(veth_debug_level, VETH_DEBUG);
	veth->rx_packet_max = veth_rx_packet_max;
	veth_set_rx_state(veth, RX_STATE_INVALID);

	ret = of_property_read_string(node, "tx_channel", &veth->tx_chan_name);
	if (ret < 0)
		veth->tx_chan_name = "vethtx";

	ret = of_property_read_string(node, "rx_channel", &veth->rx_chan_name);
	if (ret < 0)
		veth->rx_chan_name = "vethrx";

	mac_addr[0] = 0x0e;
	mac_addr[1] = 0x99;
	mac_addr[2] = 0xde;
	mac_addr[3] = 0x01;
	mac_addr[4] = 0x02;
	mac_addr[5] = 0x03;
	memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
//	random_ether_addr(ndev->dev_addr);
	ether_setup(ndev);

	/* NAPI register */
	netif_napi_add(ndev, &veth->napi, veth_poll, VETH_NAPI_WEIGHT);

	/* Register the network device */
	ndev->dev_id		= 0;
	ndev->watchdog_timeo	= VETH_TX_TIMEOUT;
	ndev->netdev_ops	= &veth_netdev_ops;

	SET_ETHTOOL_OPS(ndev, &veth_ethtool_ops);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(veth->dev, "Error registering net device\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}

	return 0;

clean_ndev_ret:
	free_netdev(ndev);
probe_quit:
	return ret;
}

static int __devexit veth_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id __devinitdata of_match[] = {
	{ .compatible = "ti,keystone-veth", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver veth_driver = {
	.driver = {
		.name		= "keystone-veth",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe = veth_probe,
	.remove = __devexit_p(veth_remove),
};

static int __init veth_init(void)
{
	return platform_driver_register(&veth_driver);
}
late_initcall_sync(veth_init);

static void __exit veth_exit(void)
{
	platform_driver_unregister(&veth_driver);
}
module_exit(veth_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI Keystone Virtual Ethernet driver");
