/*
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/serial_reg.h>

#define UART_SHIFT	2


	.macro	addruart,rp,rv,tmp
	movw	\rv, #0x0c00
	movt	\rv, #0xfed3
	movw	\rp, #0x0c00
	movt	\rp, #0x0253
	.endm


	.macro	senduart,rd,rx
	str	\rd, [\rx, #UART_TX << UART_SHIFT]
	.endm

	.macro	busyuart,rd,rx
1002:	ldr	\rd, [\rx, #UART_LSR << UART_SHIFT]
	and	\rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
	teq	\rd, #UART_LSR_TEMT | UART_LSR_THRE
	bne	1002b
	.endm

	.macro	waituart,rd,rx
#ifdef FLOW_CONTROL
1001:	ldr	\rd, [\rx, #UART_MSR << UART_SHIFT]
	tst	\rd, #UART_MSR_CTS
	beq	1001b
#endif
	.endm

