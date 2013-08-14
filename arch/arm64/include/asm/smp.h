/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SMP_H
#define __ASM_SMP_H

#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/thread_info.h>

#ifndef CONFIG_SMP
# error "<asm/smp.h> included in non-SMP build"
#endif

#define raw_smp_processor_id() (current_thread_info()->cpu)

struct seq_file;

/*
 * generate IPI list text
 */
extern void show_ipi_list(struct seq_file *p, int prec);

/*
 * Called from C code, this handles an IPI.
 */
extern void handle_IPI(int ipinr, struct pt_regs *regs);

/*
 * Setup the set of possible CPUs (via set_cpu_possible)
 */
extern void smp_init_cpus(void);

/*
 * Provide a function to raise an IPI cross call on CPUs in callmap.
 */
extern void set_smp_cross_call(void (*)(const struct cpumask *, unsigned int));

/*
 * Called from the secondary holding pen, this is the secondary CPU entry point.
 */
asmlinkage void secondary_start_kernel(void);

/*
 * Initial data for bringing up a secondary CPU.
 */
struct secondary_data {
	void *stack;
};
extern struct secondary_data secondary_data;
extern void secondary_entry(void);

extern void arch_send_call_function_single_ipi(int cpu);
extern void arch_send_call_function_ipi_mask(const struct cpumask *mask);

struct device_node;

extern int __cpu_disable(void);

extern void __cpu_die(unsigned int cpu);
extern void cpu_die(void);

/**
 * struct smp_operations - Callback operations for hotplugging CPUs.
 *
 * @name:	Name of the property as appears in a devicetree cpu node's
 *		enable-method property.
 * @cpu_init:	Reads any data necessary for a specific enable-method form the
 *		devicetree, for a given cpu node and proposed logical id.
 * @cpu_prepare: Early one-time preparation step for a cpu. If there is a
 *		mechanism for doing so, tests whether it is possible to boot
 *		the given CPU.
 * @cpu_boot:	Boots a cpu into the kernel.
 * @cpu_postboot: Optionally, perform any post-boot cleanup or necesary
 *		synchronisation. Called from the cpu being booted.
 * @cpu_disable: Prepares a cpu to die. May fail for some mechanism-specific
 * 		reason, which will cause the hot unplug to be aborted. Called
 * 		from the cpu to be killed.
 * @cpu_die:	Makes the a leave the kernel. Must not fail. Called from the
 *		cpu being killed.
 */
struct smp_operations {
	const char	*name;
	int		(*cpu_init)(struct device_node *, unsigned int);
	int		(*cpu_prepare)(unsigned int);
	int		(*cpu_boot)(unsigned int);
	void		(*cpu_postboot)(void);
#ifdef CONFIG_HOTPLUG_CPU
	int  (*cpu_disable)(unsigned int cpu);
	void (*cpu_die)(unsigned int cpu);
#endif
};

extern const struct smp_operations smp_spin_table_ops;
extern const struct smp_operations smp_psci_ops;

#endif /* ifndef __ASM_SMP_H */
