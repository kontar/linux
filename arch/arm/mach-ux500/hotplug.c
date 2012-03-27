/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *	Based on ARM realview platform
 *
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

#include <mach/setup.h>

extern volatile int pen_release;

void ux500_cpu_lowpower(unsigned int cpu, int *spurious)
{
	flush_cache_all();

	/* we put the platform to just WFI */
	for (;;) {
		__asm__ __volatile__("dsb\n\t" "wfi\n\t"
				: : : "memory");
		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}
		(*spurious)++;
	}
}
