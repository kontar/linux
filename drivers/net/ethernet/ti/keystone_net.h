/*
 * Copyright (C) 2012 Texas Instruments
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
#ifndef __KEYSTONE_NECTP_H__
#define __KEYSTONE_NECTP_H__

#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>

/* Read the e-fuse value as 32 bit values to be endian independent */
static int inline emac_arch_get_mac_addr(char *x,
					 void __iomem *efuse_mac)
{
	unsigned int addr0, addr1;

	addr1 = __raw_readl(efuse_mac + 4);
	addr0 = __raw_readl(efuse_mac);

	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;

	return 0;
}

int evm_pa_ss_init(void);
void keystone_set_ethtool_ops(struct net_device *ndev);

enum netcp_rx_state {
	RX_STATE_INTERRUPT,
	RX_STATE_SCHEDULED,
	RX_STATE_POLL,
	RX_STATE_TEARDOWN,
	RX_STATE_INVALID,
};

struct netcp_priv {
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

	int				 format_tx_cmd;
	u32				 tx_psdata[6];

	int				 hwts_tx_en;
	int				 hwts_rx_en;
	int				 tx_timestamp;
	int				 pa_calib_cnt;
	u64				 pa_offset;

	int				 rx_buffer_len;
	int				 tx_packets;

	/* 1 = link on, 0 = link off */
	u32				 link;

	/* 0 = Auto Neg, 1 = No PHY, 10, 100, 1000 - mbps */
	u32				 speed;

	/* Link duplex: 0 = Half, 1 = Full */
	u32				 duplex;

	const char			*phy_id;
	struct phy_device		*phydev;

	enum netcp_rx_state		 rx_state;
	struct list_head		 modules;
};

struct netcp_module_data {
	struct netcp_priv	*priv;
	struct netcp_module	*module;
	struct list_head	 list;
	int			(*open)(struct netcp_module_data *data,
					const u8 *mac_addr);
	int			(*close)(struct netcp_module_data *data);
	int			(*remove)(struct netcp_module_data *data);
	int			(*send_packet)(struct netcp_module_data *data,
					       void *buffer);
	int			(*get_timestamp)(struct netcp_module_data *data,
						 u64 *ts);
	u64			(*get_offset)(struct netcp_module_data *data);
	u64			(*to_sys_time)(struct netcp_module_data *data,
					       u64 offset, u64 pa_ticks);
};

struct netcp_module {
	const char		*name;
	struct module		*owner;
	struct list_head	 list;
	struct netcp_module_data *(*probe)(struct device *device,
					   struct device_node *node);
};

int netcp_register_module(struct netcp_module *module);
void netcp_unregister_module(struct netcp_module *module);

struct dma_chan *netcp_get_rx_chan(struct netcp_priv *priv);
struct dma_chan *netcp_get_tx_chan(struct netcp_priv *priv);

#endif
