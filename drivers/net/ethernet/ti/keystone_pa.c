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
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>

#include <mach/keystone-dma.h>

#include "keystone_net.h"
#include "keystone_pa.h"
#include "keystone_pasahost.h"

#define PA_PDSP_ALREADY_ACTIVE	0
#define PA_PDSP_RESET_RELEASED	1
#define PA_PDSP_NO_RESTART	2
#define PA_MAX_PDSP_ENABLE_LOOP_COUNT	100000

#define PA_STATE_RESET			0  /* Sub-system state reset */
#define PA_STATE_ENABLE			1  /* Sub-system state enable  */
#define PA_STATE_QUERY			2  /* Query the Sub-system state */
#define PA_STATE_INCONSISTENT		3  /* Sub-system is partially enabled */
#define PA_STATE_INVALID_REQUEST	4  /* Invalid state command to the Sub-system */
#define PA_STATE_ENABLE_FAILED		5  /*  The Sub-system did not respond after restart */

/* System Timestamp */
#define PAFRM_SRAM_SIZE			0x2000		
#define PAFRM_SYS_TIMESTAMP_ADDR	0x6460
#define PAFRM_SYS_TIMESTAMP_SRAM_INDEX	(PAFRM_SYS_TIMESTAMP_ADDR / PAFRM_SRAM_SIZE)
#define PAFRM_SYS_TIMESTAMP_OFFSET	((PAFRM_SYS_TIMESTAMP_ADDR % \
					  PAFRM_SRAM_SIZE)/sizeof(u32))

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_REGION_SIZE			0x48000
#define DEVICE_PA_NUM_PDSPS			6

#define PA_MEM_PDSP_IRAM(pdsp)			((pdsp) * 0x8000)
#define PA_MEM_PDSP_SRAM(num)			((num) * 0x2000)
#define PA_REG_PKTID_SOFT_RESET	                0x00404
#define PA_REG_LUT2_SOFT_RESET	                0x00504
#define PA_REG_STATS_SOFT_RESET	                0x06004

#define PA_PDSP_CONST_REG_INDEX_C25_C24     0
#define PA_PDSP_CONST_REG_INDEX_C27_C26     1
#define PA_PDSP_CONST_REG_INDEX_C29_C28     2
#define PA_PDSP_CONST_REG_INDEX_C31_C30     3

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE               (1 << 15)
#define PA_REG_VAL_PDSP_CTL_ENABLE              (1 << 1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET          (1 << 0)
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16)	\
						 | PA_REG_VAL_PDSP_CTL_ENABLE \
						 | PA_REG_VAL_PDSP_CTL_SOFT_RESET)

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS	4
#define TEST_SWINFO0_TIMESTAMP	0x12340002

const u32 pap_pdsp_const_reg_map[6][4] =
{
	/* PDSP0: C24-C31 */
	{
		0x0000007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP1: C24-C31 */
	{
		0x0001007F,		/* C25-C24 */
		0x00480040,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP2: C24-C31 */
	{
		0x0002007F,		/* C25-C24 */
		0x00490044,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP3: C24-C31 */
	{
		0x0003007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP4: C24-C31 */
	{
		0x0070007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP5: C24-C31 */
	{
		0x0071007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	}
};

struct pa_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

struct pa_packet_id_alloc_regs {
	u32 revision;
	u32 soft_reset;
	u32 range_limit;
	u32 idvalue;
};

struct pa_lut2_control_regs {
	u32 revision;
	u32 soft_reset;
	u32 rsvd[6];
	u32 add_data0;
	u32 add_data1;
	u32 add_data2;
	u32 add_data3;
	u32 add_del_key;
	u32 add_del_control;
};

struct pa_pdsp_control_regs {
	u32 control;
	u32 status;
	u32 wakeup_enable;
	u32 cycle_count;
	u32 stall_count;
	u32 rsvd[3];
	u32 const_tbl_blk_index0;
	u32 const_tbl_blk_index1;
	u32 const_tbl_prog_pointer0;
	u32 const_tbl_prog_pointer1;
	u32 rsvd1[52];
};

struct pa_pdsp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
	u32 rsvd[60];
};

struct pa_statistics_regs {
	u32 revision;
	u32 soft_reset;
	u32 incr_flags;
	u32 stats_capture;
	u32 rsvd[4];
	u32 stats_red[32];
};

struct pa_device {
	struct device			*dev;
	struct netcp_module_data	 module;
	struct clk			*pktproc;
	struct dma_chan			*tx_channel;
	struct dma_chan			*rx_channel;
	const char			*tx_chan_name;
	const char			*rx_chan_name;
	unsigned			 cmd_flow_num;
	unsigned			 cmd_queue_num;
	unsigned			 data_flow_num;
	unsigned			 data_queue_num;

	struct pa_mailbox_regs __iomem	*reg_mailbox;
	struct pa_packet_id_alloc_regs __iomem	*reg_packet_id;
	struct pa_lut2_control_regs __iomem	*reg_lut2;
	struct pa_pdsp_control_regs __iomem	*reg_control;
	struct pa_pdsp_timer_regs   __iomem	*reg_timer;
	struct pa_statistics_regs   __iomem	*reg_stats;
	void __iomem			*pa_sram;
	void __iomem			*pa_iram;
};

struct pa_device *p_dev;

#define pa_from_module(data)	container_of(data, struct pa_device, module)
#define pa_to_module(pa)	(&(pa)->module)

struct pa_packet {
	struct scatterlist		 sg[3];
	enum dma_status			 status;
	enum dma_data_direction		 direction;
	struct pa_device		*priv;
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	dma_cookie_t			 cookie;
	u32				 swdata[3];
	u32				 psdata[1];
	struct completion		 complete;
	void				*data;
};

static void pdsp_fw_put(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = le32_to_cpu(*src++);
}

static void pdsp_fw_get(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = cpu_to_le32(*src++);
}

static inline void swizFwd (struct pa_frm_forward *fwd)
{
	fwd->flow_id = fwd->flow_id;
	fwd->queue   = cpu_to_be16(fwd->queue);

	if (fwd->forward_type == PAFRM_FORWARD_TYPE_HOST) {
		fwd->u.host.context      = cpu_to_be32(fwd->u.host.context);
		fwd->u.host.multi_route  = fwd->u.host.multi_route;
		fwd->u.host.multi_idx    = fwd->u.host.multi_idx;
		fwd->u.host.pa_pdsp_router = fwd->u.host.pa_pdsp_router;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SA) {
		fwd->u.sa.sw_info_0 = cpu_to_be32(fwd->u.sa.sw_info_0);
		fwd->u.sa.sw_info_1 = cpu_to_be32(fwd->u.sa.sw_info_1);
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SRIO) {
		fwd->u.srio.ps_info0 = cpu_to_be32(fwd->u.srio.ps_info0);
		fwd->u.srio.ps_info1 = cpu_to_be32(fwd->u.srio.ps_info1);
		fwd->u.srio.pkt_type = fwd->u.srio.pkt_type;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_ETH) {
		fwd->u.eth.ps_flags	= fwd->u.eth.ps_flags;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_PA) {
		fwd->u.pa.pa_dest	= fwd->u.pa.pa_dest;
		fwd->u.pa.custom_type	= fwd->u.pa.custom_type;
		fwd->u.pa.custom_idx	= fwd->u.pa.custom_idx;
	}

	fwd->forward_type = fwd->forward_type;
}

static inline void swizFcmd (struct pa_frm_command *fcmd)
{
	fcmd->command_result =  cpu_to_be32(fcmd->command_result);
	fcmd->command	     =  fcmd->command;
	fcmd->magic          =  fcmd->magic;
	fcmd->com_id         =  cpu_to_be16(fcmd->com_id);
	fcmd->ret_context    =  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue    =  cpu_to_be16(fcmd->reply_queue);
	fcmd->reply_dest     =  fcmd->reply_dest;
	fcmd->flow_id        =  fcmd->flow_id;
}

static inline void swizAl1 (struct pa_frm_cmd_add_lut1 *al1)
{
	al1->index         =  al1->index;
	al1->type          =  al1->type;
	al1->cust_index    =  al1->cust_index;

	if (al1->type == PAFRM_COM_ADD_LUT1_STANDARD) {
		al1->u.eth_ip.etype = cpu_to_be16(al1->u.eth_ip.etype);
		al1->u.eth_ip.vlan  = cpu_to_be16(al1->u.eth_ip.vlan);
		al1->u.eth_ip.spi   = cpu_to_be32(al1->u.eth_ip.spi);
		al1->u.eth_ip.flow  = cpu_to_be32(al1->u.eth_ip.flow);

		if (al1->u.eth_ip.key & PAFRM_LUT1_KEY_MPLS)
			al1->u.eth_ip.pm.mpls     =  cpu_to_be32(al1->u.eth_ip.pm.mpls);
		else {
			al1->u.eth_ip.pm.ports[0] =  cpu_to_be16(al1->u.eth_ip.pm.ports[0]);
			al1->u.eth_ip.pm.ports[1] =  cpu_to_be16(al1->u.eth_ip.pm.ports[1]);
		}

		al1->u.eth_ip.proto_next  =  al1->u.eth_ip.proto_next;
		al1->u.eth_ip.tos_tclass  =  al1->u.eth_ip.tos_tclass;
		al1->u.eth_ip.inport      =  al1->u.eth_ip.inport;
		al1->u.eth_ip.key         =  al1->u.eth_ip.key;
		al1->u.eth_ip.match_flags =  cpu_to_be16(al1->u.eth_ip.match_flags);
	} else if (al1->type == PAFRM_COM_ADD_LUT1_SRIO) {
		al1->u.srio.src_id	= cpu_to_be16(al1->u.srio.src_id);
		al1->u.srio.dest_id	= cpu_to_be16(al1->u.srio.dest_id);
		al1->u.srio.etype	= cpu_to_be16(al1->u.srio.etype);
		al1->u.srio.vlan	= cpu_to_be16(al1->u.srio.vlan);
		al1->u.srio.pri         = al1->u.srio.pri;
		al1->u.srio.type_param1 = cpu_to_be16(al1->u.srio.type_param1);
		al1->u.srio.type_param2 = al1->u.srio.type_param2;
		al1->u.srio.key         = al1->u.srio.key;
		al1->u.srio.match_flags = cpu_to_be16(al1->u.srio.match_flags);
		al1->u.srio.next_hdr    = al1->u.srio.next_hdr;
		al1->u.srio.next_hdr_offset = cpu_to_be16(al1->u.srio.next_hdr_offset);
	} else {
		al1->u.custom.etype		=  cpu_to_be16(al1->u.custom.etype);
		al1->u.custom.vlan		=  cpu_to_be16(al1->u.custom.vlan);
		al1->u.custom.key		=  al1->u.custom.key;
		al1->u.custom.match_flags	=  cpu_to_be16(al1->u.custom.match_flags);
	}

	swizFwd(&(al1->match));
	swizFwd(&(al1->next_fail));
}

int pa_conv_routing_info(struct	pa_frm_forward *fwd_info,
			 struct	pa_route_info *route_info,
			 int cmd_dest, u16 fail_route)
{
	u8 *pcmd = NULL;
	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;

	if (route_info->dest == PA_DEST_HOST) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;

		if (route_info->m_route_index >= 0) {
			if (route_info->m_route_index >= PA_MAX_MULTI_ROUTE_SETS) {
				return (PA_ERR_CONFIG);
			}

			fwd_info->u.host.multi_route	= 1;
			fwd_info->u.host.multi_idx	= route_info->m_route_index;
			fwd_info->u.host.pa_pdsp_router	= PAFRM_DEST_PA_M_0;
		}
		pcmd = fwd_info->u.host.cmd;
	} else if (route_info->dest == PA_DEST_DISCARD)	{
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA_DEST_EMAC) {
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = (route_info->pkt_type_emac_ctrl &
					    PA_EMAC_CTRL_CRC_DISABLE)?
			PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
		fwd_info->u.eth.ps_flags |= ((route_info->pkt_type_emac_ctrl &
					      PA_EMAC_CTRL_PORT_MASK) <<
					     PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
	} else if (fail_route) {
		return (PA_ERR_CONFIG);

	} else if (((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT1))) {

		/* Custom Error check */
		if (((route_info->custom_type == PA_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT2)))
			return(PA_ERR_CONFIG);

		fwd_info->forward_type = PAFRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;

		if (route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PAFRM_DEST_PA_C2;
		} else {
			/*
			 * cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			fwd_info->u.pa.pa_dest = (cmd_dest == PA_CMD_TX_DEST_0)?
				PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2;
		}
	} else if (route_info->dest == PA_DEST_SASS) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_SA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
		pcmd = fwd_info->u.sa.cmd;
	} else if (route_info->dest == PA_DEST_SRIO) {
		fwd_info->forward_type		= PAFRM_FORWARD_TYPE_SRIO;
		fwd_info->u.srio.ps_info0	= route_info->sw_info_0;
		fwd_info->u.srio.ps_info1	= route_info->sw_info_1;
		fwd_info->u.srio.pkt_type	= route_info->pkt_type_emac_ctrl;
	} else {
		return (PA_ERR_CONFIG);
	}

	if (pcmd && route_info->pcmd) {
		struct pa_cmd_info *pacmd = route_info->pcmd;
		struct pa_patch_info *patch_info;
		struct pa_cmd_set *cmd_set;

		switch (pacmd->cmd) {
		case PA_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) ||
			    (patch_info->overwrite) ||
			    (patch_info->patch_data == NULL))
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if(cmd_set->index >= PA_MAX_CMD_SETS)
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;
		default:
			return(PA_ERR_CONFIG);
		}
	}
	return (PA_OK);
}

int keystone_pa_reset(struct pa_device *pa_dev)
{
	struct pa_packet_id_alloc_regs __iomem	*packet_id_regs = pa_dev->reg_packet_id;
	struct pa_lut2_control_regs __iomem	*lut2_regs = pa_dev->reg_lut2;
	struct pa_statistics_regs   __iomem	*stats_regs = pa_dev->reg_stats;
	u32 i;

	/* Reset and disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[i];
		__raw_writel(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
			     &ctrl_reg->control);

		while((__raw_readl(&ctrl_reg->control)
		       & PA_REG_VAL_PDSP_CTL_STATE));
	}

	/* Reset packet Id */
	__raw_writel(1, &packet_id_regs->soft_reset);

	/* Reset LUT2 */
	__raw_writel(1, &lut2_regs->soft_reset);

	/* Reset statistic */
	__raw_writel(1, &stats_regs->soft_reset);

	/* Reset timers */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[i];
		__raw_writel(0, &timer_reg->timer_control);
	}

	return 0;
}

int pa_config_timestamp(struct pa_device *pa_dev, int factor)
{	
	struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[0];

	if (factor < PA_TIMESTAMP_SCALER_FACTOR_2 ||
	    factor > PA_TIMESTAMP_SCALER_FACTOR_8192)
		return -1;
	else {
		__raw_writel(0xffff, &timer_reg->timer_load);
		__raw_writel((PA_SS_TIMER_CNTRL_REG_GO |
			      PA_SS_TIMER_CNTRL_REG_MODE |
			      PA_SS_TIMER_CNTRL_REG_PSE |
			      (factor << PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)),
			      &timer_reg->timer_control);
	}

	return 0;
}

int pa_pdsp_run(struct pa_device *pa_dev, int pdsp)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[pdsp];
	u32 i, v;
	
	/* Check for enabled PDSP */
	v = __raw_readl(&ctrl_reg->control);
	if ((v & PA_REG_VAL_PDSP_CTL_ENABLE) ==
	    PA_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return (PA_PDSP_ALREADY_ACTIVE);
	}

	/* Clear the mailbox */
	__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	__raw_writel(PA_REG_VAL_PDSP_CTL_ENABLE |
		     PA_REG_VAL_PDSP_CTL_SOFT_RESET,
		     &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)
		v = __raw_readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return (PA_PDSP_RESET_RELEASED);

	return (PA_PDSP_NO_RESTART);
}

int keystone_pa_reset_control(struct pa_device *pa_dev, int new_state)
{
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[0];
	int do_global_reset = 1;
	int i, res;
	int ret;

	if (new_state == PA_STATE_ENABLE) {
		ret = PA_STATE_ENABLE;

		/*
		 * Do nothing if a pdsp is already out of reset.
		 * If any PDSPs are out of reset
		 * a global init is not performed
		 */
		for (i = 0; i < 6; i++) {
			res = pa_pdsp_run(pa_dev, i);

			if (res == PA_PDSP_ALREADY_ACTIVE)
				do_global_reset = 0;

			if (res == PA_PDSP_NO_RESTART) {
				ret = PA_STATE_ENABLE_FAILED;
				do_global_reset = 0;
			}
		}

		/* If global reset is required any PDSP can do it */
		if (do_global_reset) {
			__raw_writel(1, &mailbox_reg->pdsp_mailbox_slot1);
			__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

			while (__raw_readl(&mailbox_reg->pdsp_mailbox_slot1) != 0);

			for (i = 1; i < 6; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}
		} else {
			for (i = 0; i < 6; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}

		}
		
		return (ret);
	}

	return (PA_STATE_INVALID_REQUEST);
}

/*
 * download/upload firmware
 */
int keystone_pa_get_firmware(struct pa_device *pa_dev,
			     int pdsp, unsigned int *buffer, int len)
{

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_get(buffer, (u32 *)(pa_dev->pa_iram + PA_MEM_PDSP_IRAM(pdsp)),
		    len >> 2);

	return 0;
}

int keystone_pa_set_firmware(struct pa_device *pa_dev,
			     int pdsp, const unsigned int *buffer, int len)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_put((u32 *)(pa_dev->pa_iram + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);


	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C25_C24],
		     &ctrl_reg->const_tbl_blk_index0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C27_C26],
		     &ctrl_reg->const_tbl_blk_index1);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C29_C28],
		     &ctrl_reg->const_tbl_prog_pointer0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C31_C30],
		     &ctrl_reg->const_tbl_prog_pointer1);

	return 0;
}

static struct pa_packet *pa_alloc_packet(struct pa_device *pa_dev,
					 unsigned cmd_size,
					 enum dma_data_direction direction)
{
	struct pa_packet *p_info;

	p_info = kzalloc(sizeof(*p_info) + cmd_size, GFP_KERNEL);
	if (!p_info)
		return NULL;

	p_info->priv = pa_dev;
	p_info->data = p_info + 1;
	p_info->direction = direction;
	p_info->chan = (direction == DMA_TO_DEVICE) ? pa_dev->tx_channel :
		pa_dev->rx_channel;

	sg_init_table(p_info->sg, 3);
	sg_set_buf(&p_info->sg[0], p_info->swdata, sizeof(p_info->swdata));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->data, cmd_size);

	init_completion(&p_info->complete);

	return p_info;
}

static void pa_free_packet(struct pa_packet *p_info)
{
	kfree(p_info);
}

static void pa_dma_callback(void *data)
{
	struct pa_packet *p_info = data;
	struct pa_device *pa_dev = p_info->priv;

	dma_unmap_sg(pa_dev->dev, p_info->sg, 3, p_info->direction);
	p_info->desc = NULL;
	complete(&p_info->complete);
}

static int pa_submit_packet(struct pa_packet *p_info)
{
	unsigned flags = DMA_HAS_SWINFO | DMA_HAS_PSINFO;
	struct dma_device *dma = p_info->chan->device;
	struct pa_device *pa_dev = p_info->priv;
	int ret;

	ret = dma_map_sg(pa_dev->dev, p_info->sg, 3, p_info->direction);
	if (ret < 0)
		return ret;

	p_info->desc = dma->device_prep_slave_sg(p_info->chan, p_info->sg, 3,
						 p_info->direction, flags);
	if (IS_ERR_OR_NULL(p_info->desc)) {
		dma_unmap_sg(pa_dev->dev, p_info->sg, 3, p_info->direction);
		return PTR_ERR(p_info->desc);
	}

	p_info->desc->callback = pa_dma_callback;
	p_info->desc->callback_param = p_info;
	p_info->cookie = dmaengine_submit(p_info->desc);

	return 0;
}

static void pa_complete_packet(struct pa_packet *p_info)
{
	wait_for_completion(&p_info->complete);
}

int keystone_pa_add_mac(struct pa_device *priv, const u8 *mac, bool to_host,
			unsigned etype, int index)
{
	struct pa_frm_command *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	struct pa_packet *tx, *rx;
	u32 context = 0xdeadbeef;
	struct pa_route_info route_info, fail_info;
	int size, ret;

	memset(&fail_info, 0, sizeof(fail_info));

	fail_info.dest		= PA_DEST_HOST;
	fail_info.flow_id	= priv->data_flow_num;
	fail_info.queue		= priv->data_queue_num;
	fail_info.m_route_index	= -1;

	memset(&route_info, 0, sizeof(route_info));

	if (to_host) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= priv->data_flow_num;
		route_info.queue		= priv->data_queue_num;
		route_info.m_route_index	= -1;
	} else {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);
	tx = pa_alloc_packet(priv, size, DMA_TO_DEVICE);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	rx = pa_alloc_packet(priv, size, DMA_FROM_DEVICE);
	if (!rx) {
		dev_err(priv->dev, "could not allocate cmd rx packet\n");
		pa_free_packet(tx);
		return -ENOMEM;
	}
	pa_submit_packet(rx);

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);

	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= PA_COMID_L2;
	fcmd->ret_context	= context;	
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;

	al1->index		= index;
	al1->type		= PAFRM_COM_ADD_LUT1_STANDARD;

	if (etype) {
		al1->u.eth_ip.etype	= etype;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_ETYPE;
	}
	
	al1->u.eth_ip.vlan	= 0;
	al1->u.eth_ip.pm.mpls	= 0;

	if (mac) {
		al1->u.eth_ip.dmac[0] = mac[0];
		al1->u.eth_ip.dmac[1] = mac[1];
		al1->u.eth_ip.dmac[2] = mac[2];
		al1->u.eth_ip.dmac[3] = mac[3];
		al1->u.eth_ip.dmac[4] = mac[4];
		al1->u.eth_ip.dmac[5] = mac[5];
		al1->u.eth_ip.key |= PAFRM_LUT1_KEY_MAC;
	}

	al1->u.eth_ip.smac[0] = 0;
	al1->u.eth_ip.smac[1] = 0;
	al1->u.eth_ip.smac[2] = 0;
	al1->u.eth_ip.smac[3] = 0;
	al1->u.eth_ip.smac[4] = 0;
	al1->u.eth_ip.smac[5] = 0;
	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0)
		dev_err(priv->dev, "route info config failed\n");

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret != 0)
		dev_err(priv->dev, "fail info config failed\n");

	swizFcmd(fcmd);
	swizAl1((struct pa_frm_cmd_add_lut1 *)&(fcmd->cmd));

	tx->psdata[0] = ((u32)(4 << 5) << 24);

	tx->swdata[0] = 0x11112222;
	tx->swdata[1] = 0x33334444;
	tx->swdata[2] = 0;
	
	pa_submit_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");
	
	pa_complete_packet(tx);
	dev_dbg(priv->dev, "command transmit complete\n");

	dev_dbg(priv->dev, "waiting for command response complete\n");
	pa_complete_packet(rx);
	if (rx->swdata[0] != context) {
		dev_warn(priv->dev, "bad response context, have %x, want %x\n",
			 rx->swdata[0], context);
	} else {
		dev_info(priv->dev, "command response complete\n");
	}

	pa_free_packet(tx);
	pa_free_packet(rx);

	return 0;
}

int pa_add_mac(struct netcp_module_data *data, u8* mac_addr,
	       int flow, int queue)
{
	struct pa_device *pa_dev = pa_from_module(data);
	int i, factor;
	const struct firmware *fw;
	dma_cap_mask_t mask;
	int ret = 0;
	
	for (i = 0; i <= 5; i++) {
		if (i <= 2)
			ret = request_firmware(&fw, "keystone/pa_pdsp02_1_2_1_2.fw", pa_dev->dev);
		else if (i == 3)
			ret = request_firmware(&fw, "keystone/pa_pdsp3_1_2_1_2.fw", pa_dev->dev);
		else if (i > 3)
			ret = request_firmware(&fw, "keystone/pa_pdsp45_1_2_1_2.fw", pa_dev->dev);
		if (ret != 0) {
			dev_err(pa_dev->dev, "cannot find firmware for pdsp %d\n", i);
			return ret;
		}

		/* Download the firmware to the PDSP */
		keystone_pa_set_firmware(pa_dev, i,
					 (const unsigned int*) fw->data,
					 fw->size);

		release_firmware(fw);
	}

	ret = keystone_pa_reset_control(pa_dev, PA_STATE_ENABLE);
	if (ret != 1) {
		dev_err(pa_dev->dev, "enabling failed, ret = %d\n", ret);
		return ret;
	}

	factor = PA_TIMESTAMP_SCALER_FACTOR_2;

	ret = pa_config_timestamp(pa_dev, factor);
	if (ret != 0) {
		dev_err(pa_dev->dev, "timestamp configuration failed, ret = %d\n", ret);
		return ret;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	pa_dev->tx_channel = dma_request_channel_by_name(mask, "patx");
	if (IS_ERR_OR_NULL(pa_dev->tx_channel)) {
		dev_err(pa_dev->dev, "Could not get TX channel\n");
		ret = -ENODEV;
		goto fail;
	}

	pa_dev->rx_channel = dma_request_channel_by_name(mask, "parx");
	if (IS_ERR_OR_NULL(pa_dev->rx_channel)) {
		dev_err(pa_dev->dev, "Could not get RX channel\n");
		ret = -ENODEV;
		goto fail;
	}
	
	pa_dev->data_flow_num = flow;
	pa_dev->data_queue_num = queue;
	pa_dev->cmd_flow_num = dma_get_rx_flow(pa_dev->rx_channel);	
	pa_dev->cmd_queue_num = dma_get_rx_queue(pa_dev->rx_channel);

	dev_dbg(pa_dev->dev, "configuring command receive flow %d, queue %d\n",
		pa_dev->cmd_flow_num, pa_dev->cmd_queue_num);

	ret = keystone_pa_add_mac(pa_dev, NULL,     true,  0x0000, 63);
	ret = keystone_pa_add_mac(pa_dev, mac_addr, false, 0x0800, 62);
	ret = keystone_pa_add_mac(pa_dev, mac_addr, false, 0x86dd, 61);
	
	dma_release_channel(pa_dev->tx_channel);
	dma_release_channel(pa_dev->rx_channel);

	kfree(pa_dev);
	return 0;
fail:
	kfree(pa_dev);
	return ret;
}

static int pa_format_tx_cmd(int n_cmd, struct pa_cmd_info *cmd_info,
			u16 offset, void *cmd_buffer, u16 *cmd_size)
{
	u16	cmd_offset = offset;
	u16	nr_cmd_size = 0;
	int	index;
	u16 pdest;
	char  *buf;
	struct pasaho_next_route *nr;

	if (((u32)cmd_buffer & 0x3) || (offset & 0x3) ||
			(offset > PAFRM_MAX_CMD_SET_SIZE))
		return (PA_ERR_CONFIG);

	if ((cmd_info == NULL) || (cmd_size == NULL))
		return (PA_ERR_CONFIG);

	if (n_cmd <= 0)
		return (PA_ERR_CONFIG);


	buf = (char *)kzalloc(*cmd_size, GFP_KERNEL);
	if (!buf) {
		printk("Could not allocate buffer\n");
		return (PA_ERR_CONFIG);
	}

	memset(buf, 0, *cmd_size);

	for (index = 0; index < n_cmd; index++) {
		switch (cmd_info[index].cmd) {

		case PA_CMD_NEXT_ROUTE:
		{
			struct pa_cmd_next_route *route = &cmd_info[index].params.route;
			nr = (struct pasaho_next_route *)&buf[cmd_offset];
			nr_cmd_size = sizeof(struct pasaho_next_route) - sizeof(nr->word1);

			/* Make sure the destination is valid */
			switch (route->dest)  {

			case PA_DEST_HOST:
			case PA_DEST_EMAC:
				pdest = (route->dest == PA_DEST_EMAC)?PAFRM_DEST_ETH:PAFRM_DEST_PKTDMA;

				if (route->pkt_type_emac_ctrl) {
					u8 ps_flags;
					PASAHO_SET_E(nr, 1);
					
					ps_flags = (route->pkt_type_emac_ctrl &
						   PA_EMAC_CTRL_CRC_DISABLE)?
						   PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
					
					ps_flags |= ((route->pkt_type_emac_ctrl &
						     PA_EMAC_CTRL_PORT_MASK) <<
						    PAFRM_ETH_PS_FLAGS_PORT_SHIFT);  

					PASAHO_SET_PKTTYPE(nr, ps_flags);
					
					nr_cmd_size += sizeof(nr->word1);
				}

				break;

			default:
				return (PA_ERR_CONFIG);
			}

			PASAHO_SET_CMDID(nr, PASAHO_PAMOD_NROUTE);
			PASAHO_SET_DEST(nr, pdest);
			PASAHO_SET_FLOW(nr, route->flow_id);
			PASAHO_SET_QUEUE (nr, route->queue);
			
			if (route->ctrl_bit_field & PA_NEXT_ROUTE_PROC_NEXT_CMD)
				PASAHO_SET_N  (nr, 1);

			nr->sw_info0 = route->sw_info_0;
			nr->sw_info1 = route->sw_info_1;
			cmd_offset += nr_cmd_size;

		}
		break;

		case PA_CMD_REPORT_TX_TIMESTAMP:
		{	
			struct pa_cmd_tx_timestamp *tx_ts =
				&cmd_info[index].params.tx_ts;
			struct pasaho_report_timestamp *rt_info =
				(struct pasaho_report_timestamp *)&buf[cmd_offset];
			PASAHO_SET_CMDID(rt_info, PASAHO_PAMOD_REPORT_TIMESTAMP);
			PASAHO_SET_REPORT_FLOW(rt_info, (u8)tx_ts->flow_id);
			PASAHO_SET_REPORT_QUEUE(rt_info, tx_ts->dest_queue);
			rt_info->sw_info0 = tx_ts->sw_info0;
			cmd_offset += sizeof(struct pasaho_report_timestamp);
		}
		break;
		
		default:
			return (PA_ERR_CONFIG);
		}
		if((cmd_offset > 128) || (cmd_offset > *cmd_size))
			return (PA_INSUFFICIENT_CMD_BUFFER_SIZE);
	}


	memcpy(cmd_buffer + offset, buf + offset, cmd_offset - offset);
	*cmd_size = cmd_offset;
	
	kfree(buf);

	return (PA_OK);
}

int pa_send_packet(struct netcp_module_data *data, void *buffer)
{
	struct pa_device *pa_dev = pa_from_module(data);
	struct pa_cmd_info *cmd_info;
	u16 cmd_stack_size = 0;
	int pa_ret = 0;
	int ret = PA_OK;
	struct pa_cmd_next_route route_cmd = {
					0,		/*  ctrlBitfield */
					PA_DEST_EMAC,	/* Route - host */
					0,              /* pktType don't care */
					0,              /* flow Id */
					0,  		/* Queue */
					0,              /* SWInfo 0 */
					0,              /* SWInfo 1 */
					0,
					};

	cmd_stack_size = sizeof(struct pasaho_report_timestamp) +
			sizeof(struct pasaho_next_route);

	cmd_info = kzalloc((2 * sizeof(struct pa_cmd_info)), GFP_KERNEL);
	if (!cmd_info)
		return -ENOMEM;

	memset(cmd_info, 0 , (2 * sizeof(struct pa_cmd_info)));

	cmd_info[0].cmd				= PA_CMD_REPORT_TX_TIMESTAMP;
	cmd_info[0].params.tx_ts.dest_queue	= 657;
	cmd_info[0].params.tx_ts.flow_id	= 30;
	cmd_info[0].params.tx_ts.sw_info0	= TEST_SWINFO0_TIMESTAMP;

	cmd_info[1].cmd				= PA_CMD_NEXT_ROUTE;
	cmd_info[1].params.route		= route_cmd;

	pa_ret = pa_format_tx_cmd(2, cmd_info, 0,
				 buffer, &cmd_stack_size);
	if (pa_ret != PA_OK) {
		dev_err(pa_dev->dev, "PA format TX cmd returned an error\n");
		ret = -1;
		goto out;
	}
out:
	kfree(cmd_info);
	return ret;
}

static struct netcp_module_data *pa_probe(struct device *dev,
					  struct device_node *node)
{
	struct pa_device *pa_dev;
	int ret = 0;
	
	pa_dev = devm_kzalloc(dev, sizeof(struct pa_device), GFP_KERNEL);
	if (!pa_dev) {
		dev_err(dev, "memory allocation failed\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		ret = -ENODEV;
		goto exit;
	}

	pa_dev->dev = dev;

	pa_dev->reg_mailbox	= devm_ioremap(dev, 0x2000000, 0x60);
	pa_dev->reg_packet_id	= devm_ioremap(dev, 0x2000400, 0x10);
	pa_dev->reg_lut2	= devm_ioremap(dev, 0x2000500, 0x40);
	pa_dev->reg_control	= devm_ioremap(dev, 0x2001000, 0x600);
	pa_dev->reg_timer	= devm_ioremap(dev, 0x2003000, 0x600);
	pa_dev->reg_stats	= devm_ioremap(dev, 0x2006000, 0x100);
	pa_dev->pa_iram		= devm_ioremap(dev, 0x2010000, 0x30000);
	pa_dev->pa_sram		= devm_ioremap(dev, 0x2040000, 0x8000);

	if (!pa_dev->reg_mailbox || !pa_dev->reg_packet_id ||
	    !pa_dev->reg_lut2 || !pa_dev->reg_control ||
	    !pa_dev->reg_timer || !pa_dev->reg_stats ||
	    !pa_dev->pa_sram || !pa_dev->pa_iram) {
		dev_err(dev, "failed to set up register areas\n");
		ret = -ENOMEM;
		goto exit;
	}
	
	pa_dev->module.open		= pa_open;
	pa_dev->module.close		= pa_close;
	pa_dev->module.remove		= pa_remove;
	pa_dev->module.add_mac		= pa_add_mac;	
	pa_dev->module.send_packet	= pa_send_packet;

	p_dev = pa_dev;

	return pa_to_module(pa_dev);
exit:
	return NULL;
}

int pa_open(struct netcp_module_data *data)
{
	struct pa_device *pa_dev = pa_from_module(data);

	pa_dev->pktproc = clk_get(pa_dev->dev, "clk_pktproc");
	if (IS_ERR(pa_dev->pktproc)) {
		dev_err(pa_dev->dev, "unable to get Packet Processor clock\n");
		return -EBUSY;
	}
	else
		clk_enable(pa_dev->pktproc);

	keystone_pa_reset(pa_dev);

	return 0;
}

int pa_close(struct netcp_module_data *data)
{
	struct pa_device *pa_dev = pa_from_module(data);

	clk_disable(pa_dev->pktproc);
	clk_put(pa_dev->pktproc);

	return 0;
}

int pa_remove(struct netcp_module_data *data)
{
	struct pa_device *pa_dev = pa_from_module(data);

	return 0;
}

static struct netcp_module pa_module = {
	.name	= "keystone-pa",
	.owner	= THIS_MODULE,
	.probe	= pa_probe,
};

static int __init keystone_pa_init(void)
{
	return netcp_register_module(&pa_module);
}
subsys_initcall(keystone_pa_init);

static void __exit keystone_pa_exit(void)
{
	netcp_register_module(&pa_module);
}
module_exit(keystone_pa_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Packet Accelerator driver for Keystone devices");
