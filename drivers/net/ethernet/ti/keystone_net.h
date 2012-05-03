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

#include "keystone_pa.h"

#define KEYSTONE_NET_SIMULATION
#define DEVICE_PSTREAM_CFG_REG_ADDR             0x02000604
#define DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_PDSP0	0

/*
 * Configure the streaming switch
 */
static void inline streaming_switch_setup(void)
{
	void __iomem *stream;

	stream = ioremap(DEVICE_PSTREAM_CFG_REG_ADDR, 4);

	__raw_writel(DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_PDSP0, stream);

	iounmap(stream);
}

struct emac_config {
	u8  enetaddr[6];
};

#define EFUSE_REG_MAC_ADDR	        0x2620110
#define SRAM_SIZE			0x2000		
#define SYS_TIMESTAMP_ADDR		0x6460
#define SYS_TIMESTAMP_SRAM_INDEX	(SYS_TIMESTAMP_ADDR / SRAM_SIZE)
#define SYS_TIMESTAMP_OFFSET		((SYS_TIMESTAMP_ADDR % \
					  SRAM_SIZE)/sizeof(u32))
#define PA_PDSP_SRAM(num)		((num) * 0x2000)

/* Read the e-fuse value as 32 bit values to be endian independent */
static int inline emac_arch_get_mac_addr(char *x)
{
	void __iomem *efuse_mac;
	unsigned int addr0, addr1;

	efuse_mac = ioremap(EFUSE_REG_MAC_ADDR, 8);
	addr1 = __raw_readl(efuse_mac + 4);
	addr0 = __raw_readl(efuse_mac);

	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;

	iounmap(efuse_mac);

	return 0;
}

#define DEVICE_PA_PDSP02_FIRMWARE "keystone/pa_pdsp02_1_2_1_2.fw"
#define DEVICE_PA_PDSP3_FIRMWARE "keystone/pa_pdsp3_1_2_1_2.fw"
#define DEVICE_PA_PDSP45_FIRMWARE "keystone/pa_pdsp45_1_2_1_2.fw"

static int inline get_timestamp(u64 *x)
{
	void __iomem *timer_val0;
	void __iomem *pdsp_sram;

	timer_val0 = ioremap(0x2003008, 4);
	pdsp_sram  = ioremap(0x2046460, 4);

	x[0] = __raw_readl(timer_val0);
	x[1] = __raw_readl(pdsp_sram);

	iounmap(timer_val0);
	iounmap(pdsp_sram);

	return 0;
}

int ethss_start(void);
int ethss_stop(void);
int ethss_init(struct device *dev);
void ethss_destroy(void);
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

	/* 1 = link on, 0 = link off */
	u32				 link;

	/* 0 = Auto Neg, 1 = No PHY, 10, 100, 1000 - mbps */
	u32				 speed;

	/* Link duplex: 0 = Half, 1 = Full */
	u32				 duplex;

	const char			*phy_id;
	struct phy_device		*phydev;

	enum netcp_rx_state		 rx_state;
	struct list_head		 stash;
	struct list_head		 modules;
};

struct netcp_module_data {
	struct netcp_module	 *module;
	struct list_head	 list;
	int			(*open)(struct netcp_module_data *data);
	int			(*close)(struct netcp_module_data *data);
	int			(*remove)(struct netcp_module_data *data);
	int			(*add_mac)(struct netcp_module_data *data,
					   u8* mac_addr, int flow, int queue);
	int			(*send_packet)(struct netcp_module_data *data,
					       void *buffer);
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


int pa_close(struct netcp_module_data *data);
int pa_open(struct netcp_module_data *data);
int pa_remove(struct netcp_module_data *data);
int pa_add_mac(struct netcp_module_data *data, u8* mac_addr,
		       int flow, int queue);
int pa_send_packet(struct netcp_module_data *data,
		   void *buffer);

#endif
