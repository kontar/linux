/*
 * Copyright (C) 2012 Rabin Vincent <rabin at rab.in>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/uprobes.h>
#include <linux/notifier.h>

#include <asm/opcodes.h>
#include <asm/traps.h>

#include "probes.h"
#include "probes-arm.h"
#include "uprobes.h"

bool is_swbp_insn(uprobe_opcode_t *insn)
{
	return (__mem_to_opcode_arm(*insn) & 0x0fffffff) == UPROBE_SWBP_INSN;
}

bool arch_uprobe_ignore(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	if (!auprobe->asi.insn_check_cc(regs->ARM_cpsr)) {
		regs->ARM_pc += 4;
		return true;
	}

	return false;
}

bool arch_uprobe_skip_sstep(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	void *addr;
	probes_opcode_t opcode;

	if (!auprobe->simulate)
		return false;

	addr = (void *) regs->ARM_pc;
	opcode = __mem_to_opcode_arm(*(unsigned int *) auprobe->insn);

	auprobe->asi.insn_singlestep(opcode, addr, &auprobe->asi, regs);

	return true;
}

unsigned long
arch_uretprobe_hijack_return_addr(unsigned long trampoline_vaddr,
				  struct pt_regs *regs)
{
	regs->ARM_pc = trampoline_vaddr;
	return trampoline_vaddr;
}

int arch_uprobe_analyze_insn(struct arch_uprobe *auprobe, struct mm_struct *mm,
			     unsigned long addr)
{
	unsigned int insn;
	unsigned int bpinsn;
	enum probes_insn ret;

	/* Thumb not yet support */
	if (addr & 0x3)
		return -EINVAL;

	insn = __mem_to_opcode_arm(*(unsigned int *)auprobe->insn);
	auprobe->modinsn = insn;

	ret = arm_probes_decode_insn(insn, &auprobe->asi, true,
				     uprobes_probes_actions);
	switch (ret) {
	case INSN_REJECTED:
		return -EINVAL;

	case INSN_GOOD_NO_SLOT:
		auprobe->simulate = true;
		break;

	case INSN_GOOD:
	default:
		break;
	}

	bpinsn = UPROBE_SWBP_INSN;
	if (insn >= 0xe0000000)
		bpinsn |= 0xe0000000;  /* Unconditional instruction */
	else
		bpinsn |= insn & 0xf0000000;  /* Copy condition from insn */

	auprobe->bpinsn = bpinsn;

	return 0;
}

void arch_uprobe_write_opcode(struct arch_uprobe *auprobe, void *vaddr,
			      uprobe_opcode_t opcode)
{
	unsigned long *addr = vaddr;

	if (opcode == UPROBE_SWBP_INSN)
		opcode = __opcode_to_mem_arm(auprobe->bpinsn);

	*addr = opcode;
}

void arch_uprobe_xol_copy(struct arch_uprobe *auprobe, void *vaddr)
{
	unsigned long *addr = vaddr;

	addr[0] = __opcode_to_mem_arm(auprobe->modinsn);
	addr[1] = __opcode_to_mem_arm(0xe0000000 | UPROBE_SS_INSN);
}

int arch_uprobe_pre_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	struct uprobe_task *utask = current->utask;

	if (auprobe->prehandler)
		auprobe->prehandler(auprobe, &utask->autask, regs);

	regs->ARM_pc = utask->xol_vaddr;

	return 0;
}

int arch_uprobe_post_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	struct uprobe_task *utask = current->utask;

	regs->ARM_pc = utask->vaddr + 4;

	if (auprobe->posthandler)
		auprobe->posthandler(auprobe, &utask->autask, regs);

	return 0;
}

bool arch_uprobe_xol_was_trapped(struct task_struct *t)
{
	/* TODO: implement */
	return false;
}

void arch_uprobe_abort_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	/* TODO: implement */
}

int arch_uprobe_exception_notify(struct notifier_block *self,
				 unsigned long val, void *data)
{
	return NOTIFY_DONE;
}

static int uprobe_trap_handler(struct pt_regs *regs, unsigned int instr)
{
	unsigned long flags;

	local_irq_save(flags);
	if ((instr & 0x0fffffff) == UPROBE_SWBP_INSN)
		uprobe_pre_sstep_notifier(regs);
	else
		uprobe_post_sstep_notifier(regs);
	local_irq_restore(flags);

	return 0;
}

unsigned long uprobe_get_swbp_addr(struct pt_regs *regs)
{
	return instruction_pointer(regs);
}

static struct undef_hook uprobes_arm_break_hook = {
	.instr_mask	= 0x0fffffff,
	.instr_val	= UPROBE_SWBP_INSN,
	.cpsr_mask	= MODE_MASK,
	.cpsr_val	= USR_MODE,
	.fn		= uprobe_trap_handler,
};

static struct undef_hook uprobes_arm_ss_hook = {
	.instr_mask	= 0x0fffffff,
	.instr_val	= UPROBE_SS_INSN,
	.cpsr_mask	= MODE_MASK,
	.cpsr_val	= USR_MODE,
	.fn		= uprobe_trap_handler,
};

int arch_uprobes_init(void)
{
	register_undef_hook(&uprobes_arm_break_hook);
	register_undef_hook(&uprobes_arm_ss_hook);

	return 0;
}
