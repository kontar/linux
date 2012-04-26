#include <asm/soc.h>

extern void keystone_clocks_init(void);

extern void keystone_set_cpu_jump(int cpu, void *jump_addr);
// extern void keystone_restart(char, const char *);
extern void __iomem *scu_base_addr;
extern void keystone_cpu_die(unsigned int cpu);

extern struct arm_soc_smp_ops keystone_soc_smp_ops;
extern struct arm_soc_smp_init_ops keystone_soc_smp_init_ops;


