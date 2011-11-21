/*
 * Copyright 2010-2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file is modified version of was the linux/arch/arm/kernel/arch_timer.c
 *
 *  Copyright (C) 2011 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/io.h>

#include <asm/cputype.h>
#include <asm/localtimer.h>
//#include <asm/arch_timer.h>
#include <asm/sched_clock.h>

#define TMR16_IRQ    ( 112 + 32 )
#define TMR16_BASE_PA 0x022F0080UL
#define TIMER_RATE   500

static struct clock_event_device __percpu **tmr16_timer_evt;
static unsigned long tmr16_timer_rate;
static void __iomem *tmr_base;
static struct irqaction irqa = {
	.flags   = IRQF_DISABLED | IRQF_TIMER,
};

static void tmr16_timer_disable(void)
{
	unsigned long tmp;

	tmp = __raw_readl(tmr_base + 0x20);
	tmp &= 0xffffff3f; //disable T12;
	__raw_writel(tmp, tmr_base + 0x20);

}

static irqreturn_t tmr16_timer_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = *(struct clock_event_device **)dev_id;

	tmr16_timer_disable();
	evt->event_handler(evt);
	return IRQ_HANDLED;

//	return IRQ_NONE;
}

static void tmr16_timer_set_mode(enum clock_event_mode mode,
				struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		tmr16_timer_disable();
		break;
	default:
		break;
	}
}

static int tmr16_timer_set_next_event(unsigned long evt,
				     struct clock_event_device *unused)
{
	unsigned long tmp;

//printk(">>>>>>>>>>>>>>>>> tmr16_timer_set_next_event %u", evt);
	// write compare register
	__raw_writel( evt, tmr_base + 0x18 ); // compare
	// enable timer and timer interrupt
	tmp = __raw_readl(tmr_base + 0x20);
//	tmp |= 0x40; //enable one shot T12;
	tmp |= 0x40; //enable periodic interrupts 50% wave T12;
	__raw_writel(tmp, tmr_base + 0x20);
		

	return 0;
}

static int __cpuinit tmr16_timer_setup(struct clock_event_device *clk)
{
	unsigned long tmp;

	/* Be safe... */
	tmr16_timer_disable();

	tmp = __raw_readl(tmr_base + 0x24);
	tmp |= 0x5; // dual mode; tmr12 out of reset 
	__raw_writel(tmp, tmr_base + 0x24);

	tmr16_timer_rate = TIMER_RATE;
	
	clk->features = CLOCK_EVT_FEAT_ONESHOT;
	clk->name = "tmr16_sys_timer";
	clk->rating = 5000;
	clk->set_mode = tmr16_timer_set_mode;
	clk->set_next_event = tmr16_timer_set_next_event;
	clk->irq = TMR16_IRQ;

	clockevents_config_and_register(clk, tmr16_timer_rate,
					0xf, 0x7fffffff);

	*__this_cpu_ptr(tmr16_timer_evt) = clk;

	enable_percpu_irq(clk->irq, 0);

	return 0;
}

static cycle_t tmr16_counter_read(struct clocksource *cs)
{
	return __raw_readl( tmr_base + 0x3c );
}

static u32 tmr16_counter_read1(void)
{
	return (u32) __raw_readl( tmr_base + 0x3c );
}

static struct clocksource clocksource_counter = {
	.name	= "arch_sys_counter",
	.rating	= 500,
	.read	= tmr16_counter_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __cpuinit tmr16_timer_stop(struct clock_event_device *clk)
{
	pr_debug("tmr16_timer_teardown disable IRQ%d cpu #%d\n",
		 clk->irq, smp_processor_id());
	disable_percpu_irq(clk->irq);
	tmr16_timer_set_mode(CLOCK_EVT_MODE_UNUSED, clk);
}

static struct local_timer_ops tmr16_timer_ops __cpuinitdata = {
	.setup	= tmr16_timer_setup,
	.stop	= tmr16_timer_stop,
};

static struct clock_event_device tmr16_timer_global_evt;

int __init tmr16_timer_register(void)
{
	int err;
	struct irq_data *id;	

	tmr_base = ioremap(TMR16_BASE_PA, SZ_4K);
	if (WARN_ON(!tmr_base))
		return -ENOMEM;

	tmr16_timer_rate = TIMER_RATE;

	tmr16_timer_evt = alloc_percpu(struct clock_event_device *);
	if (!tmr16_timer_evt)
		return -ENOMEM;

	clocksource_register_hz(&clocksource_counter, tmr16_timer_rate);

	irqa.name = "tmr16_inerrupt";
	irqa.dev_id = tmr16_timer_evt;
	irqa.handler = tmr16_timer_handler; 

	id = irq_get_irq_data( TMR16_IRQ );
	if (!id)
		return -ENOMEM;

	id->chip->irq_set_type(id, IRQ_TYPE_EDGE_RISING );

	err = setup_irq( TMR16_IRQ, &irqa );
	if (err) {
		pr_err("tmr16_timer: can't register interrupt %d (%d)\n",
		       TMR16_IRQ, err);
		goto out_free;
	}

	err = local_timer_register(&tmr16_timer_ops);
	if (err) {
		/*
		 * We couldn't register as a local timer (could be
		 * because we're on a UP platform, or because some
		 * other local timer is already present...). Try as a
		 * global timer instead.
		 */
		tmr16_timer_global_evt.cpumask = cpumask_of(0);
		err = tmr16_timer_setup(&tmr16_timer_global_evt);
	}

	if (err)
		goto out_free_irq;

	return 0;

out_free_irq:
	//free_percpu_irq(TMR16_IRQ, tmr16_timer_evt);

out_free:
	free_percpu(tmr16_timer_evt);

	return err;
}

int tmr16_timer_sched_clock_init(void)
{
	setup_sched_clock( tmr16_counter_read1, 32, tmr16_timer_rate);
	return 0;
}
