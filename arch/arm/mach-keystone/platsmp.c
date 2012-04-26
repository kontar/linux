/*
 * Copyright 2012 Texas Instruments, Inc.
 *
 * Based on platsmp.c, Copyright 2010-2011 Calxeda, Inc.
 * Based on platsmp.c, Copyright (C) 2002 ARM Ltd.
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
 */
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/smp_plat.h>
#include <asm/smp_scu.h>
#include <asm/hardware/gic.h>

#include <asm/cacheflush.h>

#include "core.h"


#if 0
   0:	ee104fb0 	mrc	15, 0, r4, cr0, cr0, {5}
   4:	e2044003 	and	r4, r4, #3
   8:	e3012234 	movw	r2, #4660	; 0x1234
   c:	e30031f0 	movw	r3, #496	; 0x1f0
  10:	e3482765 	movt	r2, #34661	; 0x8765
  14:	e3483000 	movt	r3, #32768	; 0x8000

00000018 <loop>:
  18:	e7931104 	ldr	r1, [r3, r4, lsl #2]
  1c:	e1510002 	cmp	r1, r2
  20:	0afffffc 	beq	18 <loop>
  24:	e793f104 	ldr	pc, [r3, r4, lsl #2]


void smp_boot(void)
{
	asm volatile ("		mrc	p15, 0, r4, c0, c0, 5\n"
		      "         and	r4, r4, #0x3\n"
		      "		movw    r2, #0x1234\n"
	              "		movw    r3, #0x01f0\n"
 	              "		movt    r2, #0x8765\n"
 	              "		movt    r3, #0x8000\n"
		      " loop:   ldr	r1, [r3, r4, lsl #2]\n"
 		      " 	cmp	r1, r2\n"
 		      " 	beq     loop\n"
 	              "		ldr     pc, [r3, r4, lsl #2]\n"
                      );
}
#endif

extern void secondary_startup(void);

static void __cpuinit keystone_secondary_init(unsigned int cpu)
{
	gic_secondary_init(0);
}

extern unsigned int sec_boot_addr[4];

static int __cpuinit keystone_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned int *jump_secondary_ptr = (unsigned int *)(phys_to_virt(0x800001f0));

	jump_secondary_ptr[cpu] = sec_boot_addr[cpu];
 	__cpuc_flush_dcache_area( ((unsigned int)jump_secondary_ptr) , sizeof(jump_secondary_ptr)*4); 
//	gic_raise_softirq(cpumask_of(cpu), 0);
	return 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init keystone_smp_init_cpus(void)
{
	unsigned int i, ncores;

	ncores = 4; //scu_get_core_count(scu_base_addr);

	/* sanity check */
	if (ncores > NR_CPUS) {
		printk(KERN_WARNING
		       "keystone: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

static void __init keystone_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

//	scu_enable(scu_base_addr);
	/*
	 * Write the address of secondary startup into the jump table
	 * The cores are in wfi and wait until they receive a soft interrupt
	 * and a non-zero value to jump to. Then the secondary CPU branches
	 * to this address.
	 */
	for (i = 1; i < max_cpus; i++)
		keystone_set_cpu_jump(i, secondary_startup);
}

struct arm_soc_smp_init_ops keystone_soc_smp_init_ops __initdata = {
	.smp_init_cpus		= keystone_smp_init_cpus,
	.smp_prepare_cpus	= keystone_smp_prepare_cpus,
};

struct arm_soc_smp_ops keystone_soc_smp_ops __initdata = {
	.smp_secondary_init	= keystone_secondary_init,
	.smp_boot_secondary	= keystone_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= dummy_cpu_kill,
	.cpu_die		= keystone_cpu_die,
	.cpu_disable		= dummy_cpu_disable,
#endif
};
