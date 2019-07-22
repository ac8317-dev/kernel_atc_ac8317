/*
 * Hibernation support specific for ARM
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2006 Rafael J. Wysocki <rjw at sisk.pl>
 *
 * Contact: Hiroshi DOYU <Hiroshi.DOYU at nokia.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __ASM_ARCH_HIBERNATE_H
#define __ASM_ARCH_HIBERNATE_H

#include <linux/stringify.h>

/*
 * Image of the saved processor state
 *
 * coprocessor 15 registers(RW) - SMDK6450 (ARM1176)
 */
struct saved_context {
	u32 cr;
	u32 cacr;
	u32 ttb0;
	u32 ttb1;
	u32 ttbcr;
	u32 dacr;
	u32 dfsr;
	u32 ifsr;
	u32 dfar;
	u32 wfar;
	u32 ifar;
	u32 par;
	u32 dclr;
	u32 iclr;
	u32 dtcmr;
	u32 itcmr;
	u32 tcmsel;
	u32 cbor;
	u32 tlblr;
	u32 prrr;
	u32 nrrr;
	u32 snsvbar;
	u32 mvbar;
	u32 fcse;
	u32 cid;
	u32 urwtpid;
	u32 urotpid;
	u32 potpid;
	u32 pmrr;
	u32 pmcr;
	u32 pmcc;
	u32 pmc0;
	u32 pmc1;
};

__inline__ static void __save_processor_state(struct saved_context *ctxt)
{
	asm volatile (SAVE_CPREG(p15, 0, c1, c0, 0, ctxt->cr));
	asm volatile (SAVE_CPREG(p15, 0, c1, c0, 2, ctxt->cacr));
	asm volatile (SAVE_CPREG(p15, 0, c2, c0, 0, ctxt->ttb0));
	asm volatile (SAVE_CPREG(p15, 0, c2, c0, 1, ctxt->ttb1));
	asm volatile (SAVE_CPREG(p15, 0, c2, c0, 2, ctxt->ttbcr));
	asm volatile (SAVE_CPREG(p15, 0, c3, c0, 0, ctxt->dacr));
	asm volatile (SAVE_CPREG(p15, 0, c5, c0, 0, ctxt->dfsr));
	asm volatile (SAVE_CPREG(p15, 0, c5, c0, 1, ctxt->ifsr));
	asm volatile (SAVE_CPREG(p15, 0, c6, c0, 0, ctxt->dfar));
	asm volatile (SAVE_CPREG(p15, 0, c6, c0, 1, ctxt->wfar));
	asm volatile (SAVE_CPREG(p15, 0, c6, c0, 2, ctxt->ifar));
	asm volatile (SAVE_CPREG(p15, 0, c9, c0, 0, ctxt->dclr));
	asm volatile (SAVE_CPREG(p15, 0, c9, c0, 1, ctxt->iclr));
	asm volatile (SAVE_CPREG(p15, 0, c9, c1, 0, ctxt->dtcmr));
	asm volatile (SAVE_CPREG(p15, 0, c9, c1, 1, ctxt->itcmr));
	asm volatile (SAVE_CPREG(p15, 0, c9, c2, 0, ctxt->tcmsel));
	asm volatile (SAVE_CPREG(p15, 0, c9, c8, 0, ctxt->cbor));
	asm volatile (SAVE_CPREG(p15, 0, c10, c0, 0, ctxt->tlblr));
	asm volatile (SAVE_CPREG(p15, 0, c10, c2, 0, ctxt->prrr));
	asm volatile (SAVE_CPREG(p15, 0, c10, c2, 1, ctxt->nrrr));
	asm volatile (SAVE_CPREG(p15, 0, c12, c0, 0, ctxt->snsvbar));
	asm volatile (SAVE_CPREG(p15, 0, c12, c0, 1, ctxt->mvbar));
	asm volatile (SAVE_CPREG(p15, 0, c13, c0, 0, ctxt->fcse));
	asm volatile (SAVE_CPREG(p15, 0, c13, c0, 1, ctxt->cid));
	asm volatile (SAVE_CPREG(p15, 0, c13, c0, 2, ctxt->urwtpid));
	asm volatile (SAVE_CPREG(p15, 0, c13, c0, 3, ctxt->urotpid));
	asm volatile (SAVE_CPREG(p15, 0, c13, c0, 4, ctxt->potpid));
	asm volatile (SAVE_CPREG(p15, 0, c15, c2, 4, ctxt->pmrr));
	asm volatile (SAVE_CPREG(p15, 0, c15, c12, 0, ctxt->pmcr));
	asm volatile (SAVE_CPREG(p15, 0, c15, c12, 1, ctxt->pmcc));
	asm volatile (SAVE_CPREG(p15, 0, c15, c12, 2, ctxt->pmc0));
	asm volatile (SAVE_CPREG(p15, 0, c15, c12, 3, ctxt->pmc1));
}

__inline__ static void __restore_processor_state(struct saved_context *ctxt)
{
	asm volatile (LOAD_CPREG(p15, 0, c1, c0, 0, ctxt->cr));
	asm volatile (LOAD_CPREG(p15, 0, c1, c0, 2, ctxt->cacr));
	asm volatile (LOAD_CPREG(p15, 0, c2, c0, 0, ctxt->ttb0));
	asm volatile (LOAD_CPREG(p15, 0, c2, c0, 1, ctxt->ttb1));
	asm volatile (LOAD_CPREG(p15, 0, c2, c0, 2, ctxt->ttbcr));
	asm volatile (LOAD_CPREG(p15, 0, c3, c0, 0, ctxt->dacr));
	asm volatile (LOAD_CPREG(p15, 0, c5, c0, 0, ctxt->dfsr));
	asm volatile (LOAD_CPREG(p15, 0, c5, c0, 1, ctxt->ifsr));
	asm volatile (LOAD_CPREG(p15, 0, c6, c0, 0, ctxt->dfar));
	asm volatile (LOAD_CPREG(p15, 0, c6, c0, 1, ctxt->wfar));
	asm volatile (LOAD_CPREG(p15, 0, c6, c0, 2, ctxt->ifar));
	asm volatile (LOAD_CPREG(p15, 0, c9, c0, 0, ctxt->dclr));
	asm volatile (LOAD_CPREG(p15, 0, c9, c0, 1, ctxt->iclr));
	asm volatile (LOAD_CPREG(p15, 0, c9, c1, 0, ctxt->dtcmr));
	asm volatile (LOAD_CPREG(p15, 0, c9, c1, 1, ctxt->itcmr));
	asm volatile (LOAD_CPREG(p15, 0, c9, c2, 0, ctxt->tcmsel));
	asm volatile (LOAD_CPREG(p15, 0, c9, c8, 0, ctxt->cbor));
	asm volatile (LOAD_CPREG(p15, 0, c10, c0, 0, ctxt->tlblr));
	asm volatile (LOAD_CPREG(p15, 0, c10, c2, 0, ctxt->prrr));
	asm volatile (LOAD_CPREG(p15, 0, c10, c2, 1, ctxt->nrrr));
	asm volatile (LOAD_CPREG(p15, 0, c12, c0, 0, ctxt->snsvbar));
	asm volatile (LOAD_CPREG(p15, 0, c12, c0, 1, ctxt->mvbar));
	asm volatile (LOAD_CPREG(p15, 0, c13, c0, 0, ctxt->fcse));
	asm volatile (LOAD_CPREG(p15, 0, c13, c0, 1, ctxt->cid));
	asm volatile (LOAD_CPREG(p15, 0, c13, c0, 2, ctxt->urwtpid));
	asm volatile (LOAD_CPREG(p15, 0, c13, c0, 3, ctxt->urotpid));
	asm volatile (LOAD_CPREG(p15, 0, c13, c0, 4, ctxt->potpid));
	asm volatile (LOAD_CPREG(p15, 0, c15, c2, 4, ctxt->pmrr));
	asm volatile (LOAD_CPREG(p15, 0, c15, c12, 0, ctxt->pmcr));
	asm volatile (LOAD_CPREG(p15, 0, c15, c12, 1, ctxt->pmcc));
	asm volatile (LOAD_CPREG(p15, 0, c15, c12, 2, ctxt->pmc0));
	asm volatile (LOAD_CPREG(p15, 0, c15, c12, 3, ctxt->pmc1));
}
#endif

