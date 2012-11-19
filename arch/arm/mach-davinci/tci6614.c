/*
 * Texas Instruments TCI6614 SoC Support
 *
 * Copyright (C) 2011 Texas Instruments
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
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/platform_data/clk-keystone-pll.h>
#include <linux/platform_data/clk-davinci-psc.h>
#include <linux/platform_data/davinci-clock.h>

#include <asm/mach/map.h>

#include <mach/common.h>
#include <mach/time.h>
#include <mach/cputype.h>
#include <mach/psc.h>
#include <mach/cp_intc.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/cp_intd.h>
#include <mach/tci6614.h>
#include <mach/pll.h>

#include "mux.h"

/* Base addresses for on-chip devices */
#define TCI6614_TIMER7_BASE			0x02270000
#define TCI6614_TIMER8_BASE			0x02280000
#define TCI6614_GPIO_BASE			0x02320000
#define TCI6614_CPINTC_BASE			0x0260c000
#define TCI6614_CPINTD_BASE			0x02610000
#define TCI6614_BOOT_CFG_BASE			0x02620000
#define TCI6614_AINTC_BASE			0x48200000

#define TCI6614_RSTMUX8				(TCI6614_BOOT_CFG_BASE + 0x0318)
#define RSTMUX8_OMODE_DEVICE_RESET		5
#define RSTMUX8_OMODE_DEVICE_RESET_SHIFT	1
#define RSTMUX8_OMODE_DEVICE_RESET_MASK		(BIT(1) | BIT(2) | BIT(3))
#define RSTMUX8_LOCK_MASK			BIT(0)
#define TCI6614_NUM_IRQS			(TCI6614_N_AINTC_IRQ + \
						TCI6614_N_INTD_IRQ + \
						TCI6614_N_CPINTC_IRQ)
/* Host map for interrupt controller */
static u32 intc_host_map[] = { 0x01010000, 0x01010101, -1 };

#define lpsc_clk(cname)	\
								\
	static struct davinci_clk clk_##cname = {		\
		.name		= #cname,			\
	};

lpsc_clk(timer0);
lpsc_clk(timer1);
lpsc_clk(usim);
lpsc_clk(gpio);

static struct davinci_clk_lookup clks[] = {
	CLK(NULL,		"timer0",		&clk_timer0),
	CLK("watchdog",		NULL,			&clk_timer1),
	CLK(NULL,		"usim",			&clk_usim),
	CLK(NULL,		"gpio",			&clk_gpio),
	CLK(NULL,		NULL,			NULL),
};

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static u8 aintc_irq_prios[TCI6614_N_AINTC_IRQ] = {
	/* fill in default priority 0 */
	[0 ... (TCI6614_N_AINTC_IRQ - 1)]	= 0,
	/* now override as needed, e.g. [xxx] = 5 */
};


/* Contents of JTAG ID register used to identify exact cpu type */
static struct davinci_id ids[] = {
	{
		.variant	= 0x0,
		.part_no	= 0xb962,
		.manufacturer	= 0x017,
		.cpu_id		= DAVINCI_CPU_ID_TCI6614,
		.name		= "tci6614 rev 1.0",
	},
	{
		/* For PG 1.3 */
		.variant	= 0x1,
		.part_no	= 0xb962,
		.manufacturer	= 0x017,
		.cpu_id		= DAVINCI_CPU_ID_TCI6614,
		.name		= "tci6614 rev 1.3",
	},
};

static struct davinci_timer_instance timer_instance[2] = {
	{
		.base		= TCI6614_TIMER7_BASE,
		.bottom_irq	= IRQ_TCI6614_TINT7L,
		.top_irq	= IRQ_TCI6614_TINT7H,
	},
	{
		.base		= TCI6614_TIMER8_BASE,
		.bottom_irq	= IRQ_TCI6614_TINT8L,
		.top_irq	= IRQ_TCI6614_TINT8H,
	},
};

static struct davinci_timer_info timer_info = {
	.timers		= timer_instance,
	.clockevent_id	= T0_BOT,
	.clocksource_id	= T0_TOP,
};

/*
 * TCI6614 platforms do not use the static mappings from Davinci
 * IO_PHYS/IO_VIRT. This SOC's interesting MMRs are at different addresses,
 * and changing IO_PHYS would break away from existing Davinci SOCs.
 *
 * The primary impact of the current model is that IO_ADDRESS() is not to be
 * used to map registers on TCI6614.
 */
static struct map_desc io_desc[] = {
	{
		.virtual	= IO_VIRT,
		.pfn		= __phys_to_pfn(TCI6614_IO_BASE),
		.length		= IO_SIZE,
		.type		= MT_DEVICE
	},
};

static u32 aintc_to_intd_map[TCI6614_N_AINTC_IRQ] = {
	[7]  = IRQ_TCI6614_QM_INT_HIGH_1,
	[8]  = IRQ_TCI6614_IPC_H,
	[9]  = IRQ_TCI6614_QM_INT_HIGH_0,
	[11] = IRQ_TCI6614_QM_INT_HIGH_2,
	[12] = IRQ_TCI6614_QM_INT_HIGH_3,
	[13] = IRQ_TCI6614_QM_INT_HIGH_4,
	[14] = IRQ_TCI6614_QM_INT_HIGH_5,
	[15] = IRQ_TCI6614_QM_INT_HIGH_6,
	[16] = IRQ_TCI6614_QM_INT_HIGH_7,
	[17] = IRQ_TCI6614_QM_INT_HIGH_8,
	[18] = IRQ_TCI6614_QM_INT_HIGH_9,
	[19] = IRQ_TCI6614_QM_INT_HIGH_10,
	[20] = IRQ_TCI6614_QM_INT_HIGH_11,
	[21] = IRQ_TCI6614_QM_INT_HIGH_12,
	[22] = IRQ_TCI6614_QM_INT_HIGH_13,
	[23] = IRQ_TCI6614_QM_INT_HIGH_14,
	[24] = IRQ_TCI6614_QM_INT_HIGH_15,
	[25] = IRQ_TCI6614_QM_INT_HIGH_16,
	[26] = IRQ_TCI6614_QM_INT_HIGH_17,
	[27] = IRQ_TCI6614_QM_INT_HIGH_18,
	[28] = IRQ_TCI6614_QM_INT_HIGH_19,
	[29] = IRQ_TCI6614_QM_INT_HIGH_20,
	[30] = IRQ_TCI6614_QM_INT_HIGH_21,
	[31] = IRQ_TCI6614_QM_INT_HIGH_22,
	[32] = IRQ_TCI6614_QM_INT_HIGH_23,
	[33] = IRQ_TCI6614_QM_INT_HIGH_24,
	[34] = IRQ_TCI6614_QM_INT_HIGH_25,
	[35] = IRQ_TCI6614_QM_INT_HIGH_26,
	[36] = IRQ_TCI6614_QM_INT_HIGH_27,
	[37] = IRQ_TCI6614_QM_INT_HIGH_28,
	[38] = IRQ_TCI6614_QM_INT_HIGH_29,
	[39] = IRQ_TCI6614_QM_INT_HIGH_30,
	[40] = IRQ_TCI6614_QM_INT_HIGH_31,
	[51] = IRQ_TCI6614_VUSR_INT,
	[52] = IRQ_TCI6614_SEMERR7,
	[53] = IRQ_TCI6614_SEMINT7,
	[54] = IRQ_TCI6614_SRIO_INTDST20,
	[55] = IRQ_TCI6614_SRIO_INTDST21,
	[56] = IRQ_TCI6614_SRIO_INTDST22,
	[57] = IRQ_TCI6614_SRIO_INTDST23,
	[58] = IRQ_TCI6614_TINT4L,
	[59] = IRQ_TCI6614_TINT4H,
	[60] = IRQ_TCI6614_TINT5L,
	[61] = IRQ_TCI6614_TINT5H,
	[62] = IRQ_TCI6614_TINT6L,
	[63] = IRQ_TCI6614_TINT6H,
	[64] = IRQ_TCI6614_TINT7L,
	[65] = IRQ_TCI6614_TINT7H,
	[66] = IRQ_TCI6614_PCIE_ERR_INT,
	[67] = IRQ_TCI6614_PCIE_PM_INT,
	[68] = IRQ_TCI6614_PCIE_LEGACY_INTA,
	[69] = IRQ_TCI6614_PCIE_LEGACY_INTB,
	[70] = IRQ_TCI6614_PCIE_LEGACY_INTC,
	[71] = IRQ_TCI6614_PCIE_LEGACY_INTD,
	[72] = IRQ_TCI6614_PCIE_MSI_INT4,
	[73] = IRQ_TCI6614_PCIE_MSI_INT5,
	[74] = IRQ_TCI6614_PCIE_MSI_INT6,
	[75] = IRQ_TCI6614_PCIE_MSI_INT7,
	[76] = IRQ_TCI6614_TINT8L,
	[77] = IRQ_TCI6614_TINT8H,
	[78] = IRQ_TCI6614_TINT9L,
	[79] = IRQ_TCI6614_TINT9H,
	[80] = IRQ_TCI6614_TINT10L,
	[81] = IRQ_TCI6614_TINT10H,
	[82] = IRQ_TCI6614_TINT11L,
	[83] = IRQ_TCI6614_TINT11H,
	[84] = IRQ_TCI6614_TCP3D_A_REVT0,
	[85] = IRQ_TCI6614_TCP3D_A_REVT1,
	[86] = IRQ_TCI6614_TCP3D_B_REVT0,
	[87] = IRQ_TCI6614_TCP3D_B_REVT1,
	[88] = IRQ_TCI6614_CPU_3_1_TPCC_INT2,
	[89] = IRQ_TCI6614_CPU_3_1_TPCC_INT6,
	[90] = IRQ_TCI6614_CPU_3_2_TPCC_INT2,
	[91] = IRQ_TCI6614_CPU_3_2_TPCC_INT6,
	[92] = IRQ_TCI6614_CPU_2_TPCC_INT2,
	[93] = IRQ_TCI6614_CPU_2_TPCC_INT6,
	[127] = IRQ_TCI6614_WATCH_DOG_NMI,
};

static struct davinci_soc_info tci6614_soc_info = {
	.io_desc		= io_desc,
	.io_desc_num		= ARRAY_SIZE(io_desc),
	.ids			= ids,
	.ids_num		= ARRAY_SIZE(ids),
	.jtag_id_reg		= TCI6614_BOOT_CFG_BASE + 0x18,
	.cpu_clks		= clks,
	.intc_type		= DAVINCI_INTC_TYPE_OMAP_AINTC,
	.intc_base		= TCI6614_AINTC_BASE,
	.intc_irq_prios		= aintc_irq_prios,
	.intc_irq_num		= TCI6614_N_AINTC_IRQ,
	.intc_host_map		= intc_host_map,
	.intd_irq_base		= TCI6614_INTD_IRQ_BASE,
	.intc_to_intd_map	= aintc_to_intd_map,
	.intd_irq_num		= TCI6614_N_INTD_IRQ,
	.gpio_base		= TCI6614_GPIO_BASE,
	.gpio_type		= GPIO_TYPE_DAVINCI,
	.gpio_num		= TCI6614_N_GPIO,
	.gpio_unbanked		= TCI6614_N_GPIO,
	.gpio_irq		= IRQ_TCI6614_GPINT0,
	.timer_info		= &timer_info,
};

void __init tci6614_init(void)
{
	void __iomem *rstmux8;
	u32 val;

	davinci_common_init(&tci6614_soc_info);
	/* Configure the RSTMUX8 register so that a WD output will trigger a
	   device reset
	 */
	rstmux8 = ioremap(TCI6614_RSTMUX8, 4);
	WARN_ON(!rstmux8);
	if (rstmux8) {
		val = __raw_readl(rstmux8) & ~RSTMUX8_OMODE_DEVICE_RESET_MASK;
		if (!(val & RSTMUX8_LOCK_MASK)) {
			val |= (RSTMUX8_OMODE_DEVICE_RESET <<
					RSTMUX8_OMODE_DEVICE_RESET_SHIFT);
			__raw_writel(val, rstmux8);
		} else
			printk(KERN_NOTICE "Warning, can't write to RSTMUX8\n");
	}
	iounmap(rstmux8);
}

static void __iomem *cpintc_base;
static bool debug_cpintc;
#define debug_cpintc_irq(irq)	(false)

#define CPINTC_REVISION			0x000
#define CPINTC_GLOBAL_ENABLE		0x010
#define CPINTC_STATUS_IDX_SET		0x020
#define CPINTC_STATUS_IDX_CLEAR		0x024
#define CPINTC_ENABLE_IDX_SET		0x028
#define CPINTC_ENABLE_IDX_CLEAR		0x02c
#define CPINTC_HOST_ENABLE_SET		0x034
#define CPINTC_HOST_ENABLE_CLEAR	0x038
#define CPINTC_GLOBAL_INDEX		0x080
#define CPINTC_STATUS_SET		0x200
#define CPINTC_STATUS_CLEAR		0x280
#define CPINTC_ENABLE_SET		0x300
#define CPINTC_ENABLE_CLEAR		0x380
#define CPINTC_CHANNEL_MAP		0x400
#define CPINTC_HOST_MAP			0x800

#define CPINTC_CHANS			8
#define CPINTC_REG(irq)		(BIT_WORD(irq) << 2)
#define CPINTC_BIT(irq)		(BIT_MASK(irq))

static inline u32 cpintc_readl(int offset)
{
	u32 value = __raw_readl(cpintc_base + offset);
	if (debug_cpintc)
		printk(KERN_NOTICE \
			 "cpintc: read offset %x = %x\n", offset, value);
	return value;
}

static inline void cpintc_writel(u32 value, int offset)
{
	if (debug_cpintc)
		printk(KERN_NOTICE \
			"cpintc: write offset %x = %x\n", offset, value);
	__raw_writel(value, cpintc_base + offset);
}

static void cpintc_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc || debug_cpintc_irq(irq))
		printk(KERN_NOTICE "cpintc: mask %d\n", irq);
	cpintc_writel(irq, CPINTC_ENABLE_IDX_CLEAR);
}

void cpintc_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc || debug_cpintc_irq(irq))
		printk(KERN_NOTICE "cpintc: unmask %d\n", irq);
	cpintc_writel(irq, CPINTC_ENABLE_IDX_SET);
}

void cpintc_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	if (debug_cpintc || debug_cpintc_irq(irq))
		printk(KERN_NOTICE "cpintc: ack %d\n", irq);
	cpintc_writel(irq, CPINTC_STATUS_IDX_CLEAR);
}

bool cpintc_irq_pending(unsigned int irq)
{
	bool enabled, pending;

	irq -= TCI6614_CPINTC_IRQ_BASE;
	enabled = (cpintc_readl(CPINTC_ENABLE_SET + CPINTC_REG(irq)) &
			CPINTC_BIT(irq)) ? true : false;
	pending = (cpintc_readl(CPINTC_STATUS_SET + CPINTC_REG(irq)) &
			CPINTC_BIT(irq)) ? true : false;
	if (debug_cpintc || debug_cpintc_irq(irq)) {
		printk(KERN_NOTICE "cpintc: %d %s, %s\n", irq,
		       pending ? "pending" : "not pending",
		       enabled ? "enabled" : "not enabled");
	}
	return pending;
}

struct irq_chip cpintc_chip = {
	.name	= "CPINTC",
	.irq_ack	= cpintc_ack_irq,
	.irq_mask	= cpintc_mask_irq,
	.irq_unmask	= cpintc_unmask_irq,
};

void cpintc_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);
	int cpintc_irq;

	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: start irq %d\n", irq);

	chip->irq_mask(irq_data);
	chip->irq_ack(irq_data);

	cpintc_irq = cpintc_readl(CPINTC_GLOBAL_INDEX);
	if (cpintc_irq_pending(TCI6614_CPINTC_IRQ(cpintc_irq)))
		generic_handle_irq(TCI6614_CPINTC_IRQ(cpintc_irq));
	chip->irq_unmask(irq_data);

	if (debug_cpintc)
		printk(KERN_NOTICE "cpintc: end irq %d\n", irq);
}

void __init cpintc_init(void __iomem *addr)
{
	int i;
	unsigned long rev;

	cpintc_base = addr;
	BUG_ON(!cpintc_base);

	rev = cpintc_readl(CPINTC_REVISION);
	printk(KERN_INFO "IRQ: cpintc version %ld.%ld at %p\n",
	       (rev >> 8) & 0x3, rev & 0x3f, cpintc_base);

	/* map all irqs to channel 0 */
	for (i = 0; i < DIV_ROUND_UP(TCI6614_N_CPINTC_IRQ, 4); i++)
		cpintc_writel(0, CPINTC_CHANNEL_MAP + (4 * i));

	/* map all channels to host 0 */
	for (i = 0; i < DIV_ROUND_UP(CPINTC_CHANS, 4); i++)
		cpintc_writel(0, CPINTC_HOST_MAP + (4 * i));

	cpintc_writel(0, CPINTC_HOST_ENABLE_SET);

	for (i = 0; i < TCI6614_N_CPINTC_IRQ; i++) {
		irq_set_chip_and_handler(TCI6614_CPINTC_IRQ(i),
					 &cpintc_chip, handle_edge_irq);
		set_irq_flags(TCI6614_CPINTC_IRQ(i), IRQF_VALID | IRQF_PROBE);
	}

	for (i = IRQ_TCI6614_INTC3_OUT0; i <= IRQ_TCI6614_INTC3_OUT32; i++)
		irq_set_chained_handler(i, cpintc_irq_handler);

	cpintc_writel(1, CPINTC_GLOBAL_ENABLE);
}

static int __init tci6614_add_irq_domain(struct device_node *np,
				       struct device_node *interrupt_parent)
{
	void __iomem *base;
	int i = 0;

	do {
		base = of_iomap(np, i);
		if (base) {
			if (i == 0)
				omap_aintc_init(base);
			else if (i == 1)
				cp_intd_init(base);
			else if (i == 2)
				cpintc_init(base);
			else {
				pr_warn(
				"Unknown reg base for virtintc controller\n");
			}
			i++;
		}
	} while (base);

	/* register one domain for all of the controllers */
	irq_domain_add_legacy(np, TCI6614_NUM_IRQS, 0, 0,
				&irq_domain_simple_ops, NULL);
	return 0;
}

static const struct of_device_id tci6614_irq_match[] = {
	{ .compatible = "ti,tci6614-intctrl",
	  .data = tci6614_add_irq_domain, },
	{},
};

void __init tci6614_of_init_irq(void)
{
	of_irq_init(tci6614_irq_match);
}

void tci6614_restart(char mode, const char *cmd)
{
	struct device_node *node;
	void __iomem *rstctrl;
	u32 val;

	node = of_find_compatible_node(NULL, NULL, "ti,pllctrl-reset");
	if (WARN_ON(!node)) {
		pr_warn("ti, pllctrl-reset node undefined\n");
		return;
	}

	rstctrl = of_iomap(node, 0);
	if (WARN_ON(!rstctrl)) {
		pr_warn("ti, pllctrl-reset iomap error\n");
		return;
	}

	val = __raw_readl(rstctrl);
	val &= 0xffff0000;
	val |= 0x5a69;
	__raw_writel(val, rstctrl);

	val = __raw_readl(rstctrl);
	val &= 0xfffe0000;
	__raw_writel(val, rstctrl);
}
