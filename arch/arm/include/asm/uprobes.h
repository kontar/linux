#ifndef _ASM_UPROBES_H
#define _ASM_UPROBES_H

#include <asm/probes.h>

typedef u32 uprobe_opcode_t;

#define MAX_UINSN_BYTES		4
#define UPROBE_XOL_SLOT_BYTES	64

#define UPROBE_SWBP_INSN	0x07f001f9
#define UPROBE_SS_INSN		0x07f001fa
#define UPROBE_SWBP_INSN_SIZE	4

struct arch_uprobe_task {
	u32 backup;
};

struct arch_uprobe {
	u8 insn[MAX_UINSN_BYTES];
	uprobe_opcode_t modinsn;
	uprobe_opcode_t bpinsn;
	bool simulate;
	u32 pcreg;
	void (*prehandler)(struct arch_uprobe *auprobe,
			   struct arch_uprobe_task *autask,
			   struct pt_regs *regs);
	void (*posthandler)(struct arch_uprobe *auprobe,
			    struct arch_uprobe_task *autask,
			    struct pt_regs *regs);
	struct arch_specific_insn asi;
};

#endif
