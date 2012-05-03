/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
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

#include "keystone_net.h"

#define NETCP_DRIVER_NAME	"TI KeyStone Ethernet Driver"
#define NETCP_DRIVER_VERSION	"v1.2.1"

#define TCI6614_SS_BASE				0x02090000
#define DEVICE_N_GMACSL_PORTS			2
#define DEVICE_EMACSL_RESET_POLL_COUNT		100

#define DEVICE_RX_CDMA_TIMEOUT_COUNT		1000

#define DEVICE_RX_INT_THRESHOLD                 3
#define DEVICE_TX_INT_THRESHOLD                 3

/* Soft reset register values */
#define CPGMAC_REG_RESET_VAL_RESET_MASK		(1 << 0)
#define CPGMAC_REG_RESET_VAL_RESET		(1 << 0)

/* Maxlen register values */
#define CPGMAC_REG_MAXLEN_LEN			0x3fff

#define GMACSL_RX_ENABLE_EXT_CTL		(1 << 18)
#define GMACSL_ENABLE				(1 <<  5)
#define GMACSL_RET_OK				0
#define GMACSL_RET_INVALID_PORT			-1
#define GMACSL_RET_WARN_RESET_INCOMPLETE	-2
#define GMACSL_RET_WARN_MAXLEN_TOO_BIG		-3
#define GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE	-4

#define CPSW_NUM_PORTS		                3
#define CPSW_CTL_P0_ENABLE			(1 << 2)
#define CPSW_REG_VAL_STAT_ENABLE_ALL		0xf
#define CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE	((u32)0xc0000010)
#define CPSW_REG_VAL_PORTCTL_FORWARD_MODE	0x3

#define CPSW_STATSA_MODULE			0
#define CPSW_STATSB_MODULE			1

#define MAX_SIZE_STREAM_BUFFER		        9500

struct cpsw_ss_regs {
	u32	id_ver;
	u32	soft_reset;
	u32	control;
	u32	int_control;
	u32	rx_thresh_en;
	u32	rx_en;
	u32	tx_en;
	u32	misc_en;
	u32	mem_allign1[8];
	u32	rx_thresh_stat;
	u32	rx_stat;
	u32	tx_stat;
	u32	misc_stat;
	u32	mem_allign2[8];
	u32	rx_imax;
	u32	tx_imax;
};

struct cpsw_regs {
	u32	id_ver;
	u32	control;
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
};

struct cpsw_slave_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	sa_lo;
	u32	sa_hi;
	u32	ts_ctl;
	u32	ts_seq_ltype;
	u32	ts_vlan;
};

struct cpsw_host_regs {
	u32	src_id;
	u32	port_vlan;
	u32	rx_pri_map;
	u32	rx_maxlen;
};

struct cpsw_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
	u32	rsvd[6];
};

struct cpsw_hw_stats {
	u32	rx_good_frames;
	u32	rx_broadcast_frames;
	u32	rx_multicast_frames;
	u32	rx_pause_frames;
	u32	rx_crc_errors;
	u32	rx_align_code_errors;
	u32	rx_oversized_frames;
	u32	rx_jabber_frames;
	u32	rx_undersized_frames;
	u32	rx_fragments;
	u32	__pad_0[2];
	u32	rx_bytes;
	u32	tx_good_frames;
	u32	tx_broadcast_frames;
	u32	tx_multicast_frames;
	u32	tx_pause_frames;
	u32	tx_deferred_frames;
	u32	tx_collision_frames;
	u32	tx_single_coll_frames;
	u32	tx_mult_coll_frames;
	u32	tx_excessive_collisions;
	u32	tx_late_collisions;
	u32	tx_underrun;
	u32	tx_carrier_sense_errors;
	u32	tx_bytes;
	u32	tx_64byte_frames;
	u32	tx_65_to_127byte_frames;
	u32	tx_128_to_255byte_frames;
	u32	tx_256_to_511byte_frames;
	u32	tx_512_to_1023byte_frames;
	u32	tx_1024byte_frames;
	u32	net_bytes;
	u32	rx_sof_overruns;
	u32	rx_mof_overruns;
	u32	rx_dma_overruns;
};

struct cpsw_ale_regs {
	u32	ale_idver;
	u32	rsvd0;
	u32	ale_control;
	u32	rsvd1;
	u32	ale_prescale;
	u32	rsvd2;
	u32	ale_unknown_vlan;
	u32	rsvd3;
	u32	ale_tblctl;
	u32	rsvd4[4];
	u32	ale_tblw2;
	u32	ale_tblw1;
	u32	ale_tblw0;
	u32	ale_portctl[6];
};

struct cpsw_priv {
	struct cpsw_regs __iomem	*regs;
	struct cpsw_ss_regs __iomem	*ss_regs;
	struct cpsw_hw_stats __iomem	*hw_stats[2];
	struct cpsw_host_regs __iomem	*host_port_regs;
	struct cpsw_slave_regs __iomem	*slave_port_regs[2];
	struct cpsw_sliver_regs __iomem	*sliver_port_regs[2];	
	struct cpsw_ale_regs __iomem	*ale_reg;
};

static struct cpsw_priv *priv;

/*
 * Statistic management
 */
struct netcp_ethtool_stat {
	char desc[ETH_GSTRING_LEN];
	int type;
	int size;
	int offset;
};

#define FIELDINFO(_struct, field)       FIELD_SIZEOF(_struct, field),	\
		                                offsetof(_struct, field)
#define CPSW_STATSA_INFO(field) 	"CPSW_A:"#field, CPSW_STATSA_MODULE,\
					FIELDINFO(struct cpsw_hw_stats,\
						field)
#define CPSW_STATSB_INFO(field) 	"CPSW_B:"#field, CPSW_STATSB_MODULE,\
					FIELDINFO(struct cpsw_hw_stats,\
						field)

static const struct netcp_ethtool_stat et_stats[] = {
	/* CPSW module A */
	{CPSW_STATSA_INFO(rx_good_frames)},
	{CPSW_STATSA_INFO(rx_broadcast_frames)},
	{CPSW_STATSA_INFO(rx_multicast_frames)},
	{CPSW_STATSA_INFO(rx_pause_frames)},
	{CPSW_STATSA_INFO(rx_crc_errors)},
	{CPSW_STATSA_INFO(rx_align_code_errors)},
	{CPSW_STATSA_INFO(rx_oversized_frames)},
	{CPSW_STATSA_INFO(rx_jabber_frames)},
	{CPSW_STATSA_INFO(rx_undersized_frames)},
	{CPSW_STATSA_INFO(rx_fragments)},
	{CPSW_STATSA_INFO(rx_bytes)},
	{CPSW_STATSA_INFO(tx_good_frames)},
	{CPSW_STATSA_INFO(tx_broadcast_frames)},
	{CPSW_STATSA_INFO(tx_multicast_frames)},
	{CPSW_STATSA_INFO(tx_pause_frames)},
	{CPSW_STATSA_INFO(tx_deferred_frames)},
	{CPSW_STATSA_INFO(tx_collision_frames)},
	{CPSW_STATSA_INFO(tx_single_coll_frames)},
	{CPSW_STATSA_INFO(tx_mult_coll_frames)},
	{CPSW_STATSA_INFO(tx_excessive_collisions)},
	{CPSW_STATSA_INFO(tx_late_collisions)},
	{CPSW_STATSA_INFO(tx_underrun)},
	{CPSW_STATSA_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSA_INFO(tx_bytes)},
	{CPSW_STATSA_INFO(tx_64byte_frames)},
	{CPSW_STATSA_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSA_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSA_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSA_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSA_INFO(tx_1024byte_frames)},
	{CPSW_STATSA_INFO(net_bytes)},
	{CPSW_STATSA_INFO(rx_sof_overruns)},
	{CPSW_STATSA_INFO(rx_mof_overruns)},
	{CPSW_STATSA_INFO(rx_dma_overruns)},
	/* CPSW module B */
	{CPSW_STATSB_INFO(rx_good_frames)},
	{CPSW_STATSB_INFO(rx_broadcast_frames)},
	{CPSW_STATSB_INFO(rx_multicast_frames)},
	{CPSW_STATSB_INFO(rx_pause_frames)},
	{CPSW_STATSB_INFO(rx_crc_errors)},
	{CPSW_STATSB_INFO(rx_align_code_errors)},
	{CPSW_STATSB_INFO(rx_oversized_frames)},
	{CPSW_STATSB_INFO(rx_jabber_frames)},
	{CPSW_STATSB_INFO(rx_undersized_frames)},
	{CPSW_STATSB_INFO(rx_fragments)},
	{CPSW_STATSB_INFO(rx_bytes)},
	{CPSW_STATSB_INFO(tx_good_frames)},
	{CPSW_STATSB_INFO(tx_broadcast_frames)},
	{CPSW_STATSB_INFO(tx_multicast_frames)},
	{CPSW_STATSB_INFO(tx_pause_frames)},
	{CPSW_STATSB_INFO(tx_deferred_frames)},
	{CPSW_STATSB_INFO(tx_collision_frames)},
	{CPSW_STATSB_INFO(tx_single_coll_frames)},
	{CPSW_STATSB_INFO(tx_mult_coll_frames)},
	{CPSW_STATSB_INFO(tx_excessive_collisions)},
	{CPSW_STATSB_INFO(tx_late_collisions)},
	{CPSW_STATSB_INFO(tx_underrun)},
	{CPSW_STATSB_INFO(tx_carrier_sense_errors)},
	{CPSW_STATSB_INFO(tx_bytes)},
	{CPSW_STATSB_INFO(tx_64byte_frames)},
	{CPSW_STATSB_INFO(tx_65_to_127byte_frames)},
	{CPSW_STATSB_INFO(tx_128_to_255byte_frames)},
	{CPSW_STATSB_INFO(tx_256_to_511byte_frames)},
	{CPSW_STATSB_INFO(tx_512_to_1023byte_frames)},
	{CPSW_STATSB_INFO(tx_1024byte_frames)},
	{CPSW_STATSB_INFO(net_bytes)},
	{CPSW_STATSB_INFO(rx_sof_overruns)},
	{CPSW_STATSB_INFO(rx_mof_overruns)},
	{CPSW_STATSB_INFO(rx_dma_overruns)},
};

#define ETHTOOL_STATS_NUM ARRAY_SIZE(et_stats)

/*
 * Reset the the gmac sliver
 * Soft reset is set and polled until clear, or until a timeout occurs
 */
static int ethss_port_reset(struct cpsw_priv *priv, u16 port)
{
	u32 i, v;

	/* Set the soft reset bit */

	__raw_writel(CPGMAC_REG_RESET_VAL_RESET,
			&priv->sliver_port_regs[port]->soft_reset);

	/* Wait for the bit to clear */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(&priv->sliver_port_regs[port]->soft_reset);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			return 0;
	}

	/* Timeout on the reset */
	return GMACSL_RET_WARN_RESET_INCOMPLETE;
}

/*
 * Configure the mac sliver
 */
static int ethss_port_config(struct cpsw_priv *priv, u16 port, int max_rx_len)
{
	int ret = GMACSL_RET_OK;
	u32 v, i;

	if (max_rx_len > CPGMAC_REG_MAXLEN_LEN) {
		max_rx_len = CPGMAC_REG_MAXLEN_LEN;
		ret = GMACSL_RET_WARN_MAXLEN_TOO_BIG;
	}

	/* Must wait if the device is undergoing reset */
	for (i = 0; i < DEVICE_EMACSL_RESET_POLL_COUNT; i++) {
		v = __raw_readl(&priv->sliver_port_regs[port]->soft_reset);
		if ((v & CPGMAC_REG_RESET_VAL_RESET_MASK) != CPGMAC_REG_RESET_VAL_RESET)
			break;
	}

	if (i == DEVICE_EMACSL_RESET_POLL_COUNT)
		return GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE;

	__raw_writel(max_rx_len,
			&priv->sliver_port_regs[port]->rx_maxlen);
	
	__raw_writel(GMACSL_ENABLE | GMACSL_RX_ENABLE_EXT_CTL,
			&priv->sliver_port_regs[port]->mac_control);

	return ret;
}

int ethss_start()
{
	int i;

	/* Max length register */
	__raw_writel(9504, &priv->host_port_regs->rx_maxlen);
	
	/* Control register */
	__raw_writel(CPSW_CTL_P0_ENABLE, &priv->regs->control);
	
	/* All statistics enabled by default */
	__raw_writel(CPSW_REG_VAL_STAT_ENABLE_ALL, &priv->regs->stat_port_en);
	
	/* Reset and enable the ALE */
	__raw_writel(CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE,
			&priv->ale_reg->ale_control);

	/* All ports put into forward mode */
	for (i = 0; i < CPSW_NUM_PORTS; i++)
		__raw_writel(CPSW_REG_VAL_PORTCTL_FORWARD_MODE,
				&priv->ale_reg->ale_portctl[i]);

	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++)  {
		ethss_port_reset(priv, i);
		ethss_port_config(priv, i, MAX_SIZE_STREAM_BUFFER);
        }

	return 0;
}

int ethss_stop()
{
	int i;

	for (i = 0; i < DEVICE_N_GMACSL_PORTS; i++)
		ethss_port_reset(priv, i);

	return 0;
}

static void keystone_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strcpy(info->driver, NETCP_DRIVER_NAME);
	strcpy(info->version, NETCP_DRIVER_VERSION);
}

static u32 keystone_get_msglevel(struct net_device *ndev)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	return netcp->msg_enable;
}

static void keystone_set_msglevel(struct net_device *ndev, u32 value)
{
	struct netcp_priv *netcp = netdev_priv(ndev);
	netcp->msg_enable = value;
}

static void keystone_get_stat_strings(struct net_device *ndev,
				   uint32_t stringset, uint8_t *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
			memcpy(data, et_stats[i].desc, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		break;
	}
}

static int keystone_get_sset_count(struct net_device *ndev, int stringset)
{
	switch (stringset) {
	case ETH_SS_TEST:
		return 0;
	case ETH_SS_STATS:
		return ETHTOOL_STATS_NUM;
	default:
		return -EINVAL;
	}
}

static void keystone_get_ethtool_stats(struct net_device *ndev,
				       struct ethtool_stats *stats,
				       uint64_t *data)
{
	struct cpsw_hw_stats __iomem *cpsw_statsa = priv->hw_stats[0];
	struct cpsw_hw_stats __iomem *cpsw_statsb = priv->hw_stats[1];
	void *p = NULL;
	int i;

	for (i = 0; i < ETHTOOL_STATS_NUM; i++) {
		switch (et_stats[i].type) {
		case CPSW_STATSA_MODULE:
			p = cpsw_statsa;
			break;
		case CPSW_STATSB_MODULE:
			p  = cpsw_statsb;
			break;
		}

		p = (u8 *)p + et_stats[i].offset;
		data[i] = (et_stats[i].size == sizeof(u64)) ?
			*(u64 *)p: *(u32 *)p;
	}

	return;
}

static int keystone_get_settings(struct net_device *ndev,
				 struct ethtool_cmd *ecmd)
{
#ifndef KEYSTONE_NET_SIMULATION
	struct netcp_priv *netcp = netdev_priv(ndev);

	if (netcp->phydev)
		return phy_ethtool_gset(netcp->phydev, ecmd);
	else
		return -EOPNOTSUPP;
#else
	return -EOPNOTSUPP;
#endif
}

static int keystone_set_settings(struct net_device *ndev,
				 struct ethtool_cmd *ecmd)
{
#ifndef KEYSTONE_NET_SIMULATION
	struct netcp_priv *netcp = netdev_priv(ndev);

	if (netcp->phydev)
		return phy_ethtool_sset(netcp->phydev, ecmd);
	else
		return -EOPNOTSUPP;
#else
	return -EOPNOTSUPP;
#endif
}

static const struct ethtool_ops keystone_ethtool_ops = {
	.get_drvinfo		= keystone_get_drvinfo,
	.get_settings		= keystone_get_settings,
	.set_settings		= keystone_set_settings,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= keystone_get_msglevel,
	.set_msglevel		= keystone_set_msglevel,
	.get_strings		= keystone_get_stat_strings,
	.get_sset_count		= keystone_get_sset_count,
	.get_ethtool_stats	= keystone_get_ethtool_stats,
};

void keystone_set_ethtool_ops(struct net_device *ndev)
{
	SET_ETHTOOL_OPS(ndev, &keystone_ethtool_ops);
}

int ethss_init(struct device *dev)
{
	struct clk *cpgmac;
	void __iomem *regs;

#ifndef KEYSTONE_NET_SIMULATION
	cpgmac = clk_get(dev, "clk_cpgmac");
	if (IS_ERR(cpgmac)) {
		dev_err(dev, "unable to get CPGMAC clock\n");
		return -EBUSY;
	}
	else
		clk_enable(cpgmac);
#endif

	priv = kzalloc(sizeof(struct cpsw_priv), GFP_KERNEL);
	BUG_ON(!priv);

	regs = ioremap(TCI6614_SS_BASE, 0xf00);
	BUG_ON(!regs);

	priv->ss_regs = regs;
	priv->regs = regs + 0x800;
	priv->host_port_regs = regs + 0x800 + 0x34;
	priv->slave_port_regs[0] = regs + 0x800 + 0x60;
	priv->slave_port_regs[1] = regs + 0x800 + 0x90;
	priv->sliver_port_regs[0] = regs + 0x800 + 0x100;
	priv->sliver_port_regs[1] = regs + 0x800 + 0x100 + 0x40;
	priv->hw_stats[0] = regs + 0x800 + 0x300;
	priv->hw_stats[1] = regs + 0x800 + 0x400;
	priv->ale_reg	  = regs + 0x800 + 0x600;

	return 0;
}

void ethss_destroy(void)
{
	iounmap(priv->ss_regs);
	kfree(priv);
	priv = NULL;
}
