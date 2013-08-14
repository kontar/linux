/*
 * PSCI SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
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

#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/psci.h>
#include <asm/smp_plat.h>

static int smp_psci_cpu_init(struct device_node *dn, unsigned int cpu)
{
	return 0;
}

static int smp_psci_cpu_prepare(unsigned int cpu)
{
	int err;

	if (!psci_ops.cpu_on) {
		pr_err("psci: no cpu_on method, not booting CPU%d\n", cpu);
		return -ENODEV;
	}

	err = psci_ops.cpu_on(cpu_logical_map(cpu), __pa(secondary_holding_pen));
	if (err) {
		pr_err("psci: failed to boot CPU%d (%d)\n", cpu, err);
		return err;
	}

	return 0;
}

const struct smp_operations smp_psci_ops = {
	.name		= "psci",
	.cpu_init	= smp_psci_cpu_init,
	.cpu_prepare	= smp_psci_cpu_prepare,
};
