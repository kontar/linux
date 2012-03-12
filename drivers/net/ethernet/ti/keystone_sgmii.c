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

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#define SGMII_REG_BASE                  0x02090100
#define DSCR_SGMII_SERDES_BASE		0x02620340
#define DSCR_SGMII_SERDES_CFGPLL	0x0
#define DSCR_SGMII_SERDES_CFGRX0	0x4
#define DSCR_SGMII_SERDES_CFGTX0	0x8
#define DSCR_SGMII_SERDES_CFGRX1	0xC
#define DSCR_SGMII_SERDES_CFGTX1	0x10
#define DSCR_SGMII_SERDES_SIZE		0x14

#define SGMII_SRESET_RESET		0x1
#define SGMII_SRESET_RTRESET		0x2
#define SGMII_CTL_AUTONEG		0x01
#define SGMII_CTL_LOOPBACK		0x10
#define SGMII_CTL_MASTER		0x20
#define SGMII_REG_STATUS_FIELD_LOCK	(1<<4)

/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    ((x * 0x100) + 0x000)
#define SGMII_SRESET_REG(x)   ((x * 0x100) + 0x004)
#define SGMII_CTL_REG(x)      ((x * 0x100) + 0x010)
#define SGMII_STATUS_REG(x)   ((x * 0x100) + 0x014)
#define SGMII_MRADV_REG(x)    ((x * 0x100) + 0x018)
#define SGMII_LPADV_REG(x)    ((x * 0x100) + 0x020)
#define SGMII_TXCFG_REG(x)    ((x * 0x100) + 0x030)
#define SGMII_RXCFG_REG(x)    ((x * 0x100) + 0x034)
#define SGMII_AUXCFG_REG(x)   ((x * 0x100) + 0x038)

struct sgmii_config {
	unsigned int loopback;
	unsigned int master;
	unsigned int autoneg;
	unsigned int txconfig;
	unsigned int rxconfig;
	unsigned int auxconfig;
};

static void __iomem *dssr_sgmii_serdes;

static inline void dscr_write_reg(int reg, u32 val)
{
	__raw_writel(val, dssr_sgmii_serdes + reg);
}

int serdes_init(void)
{
	dssr_sgmii_serdes = ioremap(DSCR_SGMII_SERDES_BASE,
					DSCR_SGMII_SERDES_SIZE);

	dscr_write_reg(DSCR_SGMII_SERDES_CFGPLL, 0x00000041);
	
	udelay(2000);

	dscr_write_reg(DSCR_SGMII_SERDES_CFGRX0, 0x00700621);
	dscr_write_reg(DSCR_SGMII_SERDES_CFGRX1, 0x00700621);

	dscr_write_reg(DSCR_SGMII_SERDES_CFGTX0, 0x000108a1);
	dscr_write_reg(DSCR_SGMII_SERDES_CFGTX1, 0x000108a1);

	udelay(2000);

	iounmap(dssr_sgmii_serdes);

	return 0;
}	

static void __iomem	*base;

static inline void sgmii_write_reg(int reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 sgmii_read_reg(int reg)
{
	return __raw_readl(base + reg);
}

static inline void sgmii_write_reg_bit(int reg, u32 val)
{
	__raw_writel((__raw_readl(base + reg) | val),
			base + reg);
}

extern int keystone_sgmii_reset(int port)
{
	sgmii_write_reg(SGMII_CTL_REG(port), 0);

	/* Soft reset */
	sgmii_write_reg_bit(SGMII_SRESET_REG(port), 0x1);
	while(sgmii_read_reg(SGMII_SRESET_REG(port)) != 0x0);

	return 0;
}

int keystone_sgmii_config(int port, struct sgmii_config *config)
{
	unsigned int i, status;

	sgmii_write_reg(SGMII_CTL_REG(port), 0);

	sgmii_write_reg(SGMII_TXCFG_REG(port), config->txconfig);
	sgmii_write_reg(SGMII_RXCFG_REG(port), config->rxconfig);
	sgmii_write_reg(SGMII_AUXCFG_REG(port), config->auxconfig);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
		udelay(2000);
		status = sgmii_read_reg(SGMII_STATUS_REG(port));
		if ( (status & SGMII_REG_STATUS_FIELD_LOCK) != 0 )
			break;
	}

	sgmii_write_reg(SGMII_MRADV_REG(port), 1);
	sgmii_write_reg(SGMII_CTL_REG(port), 1);

	return 0;
}

static int evm_sgmii_configure(void)
{
	struct sgmii_config sgmiic0, sgmiic1;

	sgmiic0.master    = 1;
	sgmiic0.loopback  = 0;
	sgmiic0.autoneg   = 0;
	sgmiic0.txconfig  = 0x000108a1;
	sgmiic0.rxconfig  = 0x00700621;
	sgmiic0.auxconfig = 0x00000041; /* PLL multiplier */

	keystone_sgmii_config(0, &sgmiic0);

	sgmiic1.master    = 1;
	sgmiic1.loopback  = 0;
	sgmiic1.autoneg   = 0;
	sgmiic1.txconfig  = 0x000108a1;
	sgmiic1.rxconfig  = 0x00700621;
	sgmiic1.auxconfig = 0x00000041; /* PLL multiplier */

	keystone_sgmii_config(1, &sgmiic1);

	printk("SGMII init complete\n");

	return 0;
}

int keystone_sgmii_init(void)
{

	base = ioremap(SGMII_REG_BASE, 0x150);

	return 0;
}

int evm_pa_ss_init(void)
{
	serdes_init();

	keystone_sgmii_init();
	/* Reset SGMII */
	keystone_sgmii_reset(0);
	keystone_sgmii_reset(1);
	
	/* Configure the SGMII */
	evm_sgmii_configure();

	return 0;
}


