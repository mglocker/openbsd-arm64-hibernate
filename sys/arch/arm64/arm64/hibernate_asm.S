/*	$OpenBSD$ */

/*
 * Copyright (c) 2026 Marcus Glocker <mglocker@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <machine/asm.h>
#include <machine/param.h>
#include <machine/hibernate_var.h>

	.text

/*
 * Restore the suspending kernel's CPU context saved by
 * hibernate_save_state_machdep(), then long-jump back into that call
 * with x0 = 1 (the resume-path return value).
 *
 * x0 in: piglet KVA of the saved context struct
 *        (global_piglet_va + HIBERNATE_SAVED_CTX_PAGE).
 *
 * All ctx values are loaded into registers before the MSR sequence:
 * once ttbr1_el1 is swapped, the table walker uses the suspending
 * kernel's pmap, which is not guaranteed to map the address we were
 * reading from.
 *
 * DAIF is intentionally NOT restored -- the outer caller
 * re-establishes IRQ mask state.
 */
ENTRY(hibernate_resume_machdep)
	mov	x16, x0

	ldr	x1,  [x16, #0x60]	/* SP    */
	ldr	x8,  [x16, #0x78]	/* MAIR  */
	ldr	x9,  [x16, #0x70]	/* TCR   */
	ldr	x10, [x16, #0x68]	/* TTBR1 */
	ldr	x11, [x16, #0xa0]	/* TTBR0 */
	ldr	x12, [x16, #0x88]	/* VBAR  */
	ldr	x13, [x16, #0x90]	/* TPIDR */
	ldr	x14, [x16, #0x80]	/* SCTLR */

	ldp	x19, x20, [x16, #0x00]
	ldp	x21, x22, [x16, #0x10]
	ldp	x23, x24, [x16, #0x20]
	ldp	x25, x26, [x16, #0x30]
	ldp	x27, x28, [x16, #0x40]
	ldp	x29, x30, [x16, #0x50]

	mov	sp, x1

	msr	mair_el1, x8
	msr	tcr_el1, x9
	msr	ttbr1_el1, x10
	msr	ttbr0_el1, x11
	msr	vbar_el1, x12
	msr	tpidr_el1, x13
	isb

	dsb	ish
	tlbi	vmalle1is
	dsb	ish
	isb

	ic	ialluis
	dsb	ish
	isb

	msr	sctlr_el1, x14
	isb

	mov	x0, #1
	ret
END(hibernate_resume_machdep)

/*
 * Switch to the resume-time page table.  Force T0SZ = 16 to match the
 * 4-level walk the resume PT was built with.
 */
ENTRY(hibernate_activate_resume_pt_machdep)
	mrs	x1, tcr_el1
	bic	x1, x1, #0x3f
	orr	x1, x1, #16
	msr	tcr_el1, x1
	isb

	adrp	x0, hibernate_resume_pt0_pa
	ldr	x0, [x0, :lo12:hibernate_resume_pt0_pa]
	dsb	ish
	msr	ttbr0_el1, x0
	isb

	adrp	x0, hibernate_resume_pt1_pa
	ldr	x0, [x0, :lo12:hibernate_resume_pt1_pa]
	dsb	ish
	msr	ttbr1_el1, x0
	isb

	dsb	ish
	tlbi	vmalle1is
	dsb	ish
	isb
	ret
END(hibernate_activate_resume_pt_machdep)

/* Switch to the private resume-time hibernate stack */
ENTRY(hibernate_switch_stack_machdep)
	adrp	x0, hibernate_stack_top_pa
	ldr	x0, [x0, :lo12:hibernate_stack_top_pa]
	mov	sp, x0
	ret
END(hibernate_switch_stack_machdep)

/* Broadcast TLBI (no ASID-less single-page primitive on arm64) */
ENTRY(hibernate_flush)
	dsb	ish
	tlbi	vmalle1is
	dsb	ish
	isb
	ret
END(hibernate_flush)

/*
 * setjmp-style save of callee-saved GPs, SP, and the sysregs that
 * hibernate_resume_machdep() will restore.  Returns 0; long-jump returns 1.
 */
ENTRY(hibernate_save_state_machdep)
	stp	x19, x20, [x0, #0x00]
	stp	x21, x22, [x0, #0x10]
	stp	x23, x24, [x0, #0x20]
	stp	x25, x26, [x0, #0x30]
	stp	x27, x28, [x0, #0x40]
	stp	x29, x30, [x0, #0x50]

	mov	x1, sp
	str	x1, [x0, #0x60]

	mrs	x1, ttbr1_el1
	str	x1, [x0, #0x68]
	mrs	x1, tcr_el1
	str	x1, [x0, #0x70]
	mrs	x1, mair_el1
	str	x1, [x0, #0x78]
	mrs	x1, sctlr_el1
	str	x1, [x0, #0x80]
	mrs	x1, vbar_el1
	str	x1, [x0, #0x88]
	mrs	x1, tpidr_el1
	str	x1, [x0, #0x90]
	mrs	x1, daif
	str	x1, [x0, #0x98]
	mrs	x1, ttbr0_el1
	str	x1, [x0, #0xa0]

	dsb	sy

	mov	x0, #0
	ret
END(hibernate_save_state_machdep)

/*
 * Copy the suspender's retguard snapshot from its piglet holding slot
 * back into the retguard PAs that HIB_MOVE preserved during unpack.
 * Called from hibernate_unpack_image after the inflate loop and before
 * hibernate_resume_machdep().  Mirrors the inline block inside amd64's
 * hibernate_resume_machdep (acpi_wakecode.S:365-400).
 *
 * x0 in: source VA (global_piglet_va + 110*PAGE_SIZE).
 *
 * Runs with the resume PT active: retguard PAs are identity-mapped R/W
 * via TTBR0 (see hibernate_populate_resume_pt), so retguard_start_phys
 * is used directly as the write address.
 */
ENTRY(hibernate_retguard_copy_machdep)
	adrp	x1, retguard_start_phys
	ldr	x1, [x1, :lo12:retguard_start_phys]	/* dest PA (VA==PA) */
	adrp	x2, retguard_end_phys
	ldr	x2, [x2, :lo12:retguard_end_phys]	/* end  PA (VA==PA) */
	sub	x2, x2, x1				/* byte count */
	lsr	x2, x2, #3				/* quadword count */
	cbz	x2, 2f
1:	ldr	x3, [x0], #8
	str	x3, [x1], #8
	subs	x2, x2, #1
	b.ne	1b
2:	dsb	sy
	isb
	ret
END(hibernate_retguard_copy_machdep)
