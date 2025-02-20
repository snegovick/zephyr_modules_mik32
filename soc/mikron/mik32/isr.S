/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 * Copyright (c) 2018 Foundries.io Ltd
 * Copyright (c) 2020 BayLibre, SAS
 * Copyright (c) 2025 Excave.ru
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/syscall.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/arch/riscv/syscall.h>
#include "asm_macros.inc"

/* Convenience macro for loading/storing register states. */
#define DO_CALLER_SAVED(op) \
	RV_E(	op t0, __struct_arch_esf_t0_OFFSET(sp)	);\
	RV_E(	op t1, __struct_arch_esf_t1_OFFSET(sp)	);\
	RV_E(	op t2, __struct_arch_esf_t2_OFFSET(sp)	);\
	RV_I(	op t3, __struct_arch_esf_t3_OFFSET(sp)	);\
	RV_I(	op t4, __struct_arch_esf_t4_OFFSET(sp)	);\
	RV_I(	op t5, __struct_arch_esf_t5_OFFSET(sp)	);\
	RV_I(	op t6, __struct_arch_esf_t6_OFFSET(sp)	);\
	RV_E(	op a0, __struct_arch_esf_a0_OFFSET(sp)	);\
	RV_E(	op a1, __struct_arch_esf_a1_OFFSET(sp)	);\
	RV_E(	op a2, __struct_arch_esf_a2_OFFSET(sp)	);\
	RV_E(	op a3, __struct_arch_esf_a3_OFFSET(sp)	);\
	RV_E(	op a4, __struct_arch_esf_a4_OFFSET(sp)	);\
	RV_E(	op a5, __struct_arch_esf_a5_OFFSET(sp)	);\
	RV_I(	op a6, __struct_arch_esf_a6_OFFSET(sp)	);\
	RV_I(	op a7, __struct_arch_esf_a7_OFFSET(sp)	);\
	RV_E(	op ra, __struct_arch_esf_ra_OFFSET(sp)	)

#ifdef CONFIG_EXCEPTION_DEBUG
/* Convenience macro for storing callee saved register [s0 - s11] states. */
#define STORE_CALLEE_SAVED() \
	RV_E(	sr s0, ___callee_saved_t_s0_OFFSET(sp)		);\
	RV_E(	sr s1, ___callee_saved_t_s1_OFFSET(sp)		);\
	RV_I(	sr s2, ___callee_saved_t_s2_OFFSET(sp)		);\
	RV_I(	sr s3, ___callee_saved_t_s3_OFFSET(sp)		);\
	RV_I(	sr s4, ___callee_saved_t_s4_OFFSET(sp)		);\
	RV_I(	sr s5, ___callee_saved_t_s5_OFFSET(sp)		);\
	RV_I(	sr s6, ___callee_saved_t_s6_OFFSET(sp)		);\
	RV_I(	sr s7, ___callee_saved_t_s7_OFFSET(sp)		);\
	RV_I(	sr s8, ___callee_saved_t_s8_OFFSET(sp)		);\
	RV_I(	sr s9, ___callee_saved_t_s9_OFFSET(sp)		);\
	RV_I(	sr s10, ___callee_saved_t_s10_OFFSET(sp)	);\
	RV_I(	sr s11, ___callee_saved_t_s11_OFFSET(sp)	)
#endif /* CONFIG_EXCEPTION_DEBUG */

	.macro get_current_cpu dst
	la \dst, _kernel + ___kernel_t_cpus_OFFSET
	.endm

/* imports */
GDATA(_sw_isr_table)
GTEXT(__soc_handle_irq)
GTEXT(z_riscv_fault)
#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
GTEXT(__soc_save_context)
GTEXT(__soc_restore_context)
#endif /* CONFIG_RISCV_SOC_CONTEXT_SAVE */

#ifdef CONFIG_EXCEPTION_DEBUG
GTEXT(z_riscv_fatal_error_csf)
#else
GTEXT(z_riscv_fatal_error)
#endif /* CONFIG_EXCEPTION_DEBUG */

GTEXT(z_get_next_switch_handle)
GTEXT(z_riscv_switch)
GTEXT(z_riscv_thread_start)

#ifdef CONFIG_TRACING
GTEXT(sys_trace_isr_enter)
GTEXT(sys_trace_isr_exit)
#endif

#ifdef CONFIG_RISCV_SOC_HAS_CUSTOM_IRQ_HANDLING
GTEXT(__soc_handle_all_irqs)
#endif

/* exports */
GTEXT(_isr_wrapper)

/* use ABI name of registers for the sake of simplicity */

/*
 * Generic architecture-level IRQ handling, along with callouts to
 * SoC-specific routines.
 *
 * Architecture level IRQ handling includes basic context save/restore
 * of standard registers and calling ISRs registered at Zephyr's driver
 * level.
 *
 * Since RISC-V does not completely prescribe IRQ handling behavior,
 * implementations vary (some implementations also deviate from
 * what standard behavior is defined). Hence, the arch level code expects
 * the following functions to be provided at the SOC level:
 *
 *     - __soc_is_irq (optional): decide if we're handling an interrupt or an
         exception
 *     - __soc_handle_irq: handle SoC-specific details for a pending IRQ
 *       (e.g. clear a pending bit in a SoC-specific register)
 *
 * If CONFIG_RISCV_SOC_CONTEXT_SAVE=y, calls to SoC-level context save/restore
 * routines are also made here. For details, see the Kconfig help text.
 */

/*
 * Handler called upon each exception/interrupt/fault
 */
#ifdef CONFIG_NON_POWER_OF_2_ISR_ALIGNMENT_FIXUP
.section .alignment_fixup, "ax"
  .globl _alignment_fixup_lbl
  .org CONFIG_ISR_FIXUP_SECTION_OFFSET
_alignment_fixup_lbl:
  j _isr_wrapper
#endif

SECTION_FUNC(exception.entry, _isr_wrapper)

/* Provide requested alignment, which depends e.g. on MTVEC format */
.balign CONFIG_RISCV_TRAP_HANDLER_ALIGNMENT

	/* Save caller-saved registers on current thread stack. */
	addi sp, sp, -__struct_arch_esf_SIZEOF
	DO_CALLER_SAVED(sr)		;

	/* Save s0 in the esf and load it with &_current_cpu. */
	sr s0, __struct_arch_esf_s0_OFFSET(sp)
	get_current_cpu s0

	/* Save MEPC register */
	csrr t0, mepc
	sr t0, __struct_arch_esf_mepc_OFFSET(sp)

	/* Save MSTATUS register */
	csrr t2, mstatus
	sr t2, __struct_arch_esf_mstatus_OFFSET(sp)

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	/* Handle context saving at SOC level. */
	addi a0, sp, __struct_arch_esf_soc_context_OFFSET
	jal ra, __soc_save_context
#endif /* CONFIG_RISCV_SOC_CONTEXT_SAVE */

	/*
	 * Check if exception is the result of an interrupt or not.
	 * (SOC dependent). Following the RISC-V architecture spec, the MSB
	 * of the mcause register is used to indicate whether an exception
	 * is the result of an interrupt or an exception/fault. But for some
	 * SOCs (like pulpino or riscv-qemu), the MSB is never set to indicate
	 * interrupt. Hence, check for interrupt/exception via the __soc_is_irq
	 * function (that needs to be implemented by each SOC). The result is
	 * returned via register a0 (1: interrupt, 0 exception)
	 */
	csrr t0, mcause
	srli t0, t0, RISCV_MCAUSE_IRQ_POS
	bnez t0, is_interrupt

	/*
	 * If the exception is the result of an ECALL, check whether to
	 * perform a context-switch or an IRQ offload. Otherwise call z_riscv_fault
	 * to report the exception.
	 */
	csrr t0, mcause
	li t2, CONFIG_RISCV_MCAUSE_EXCEPTION_MASK
	and t0, t0, t2

	/*
	 * If mcause == RISCV_EXC_ECALLM, handle system call from
	 * kernel thread.
	 */
	li t1, RISCV_EXC_ECALLM
	beq t0, t1, is_kernel_syscall

	/*
	 * Call z_riscv_fault to handle exception.
	 * Stack pointer is pointing to a struct_arch_esf structure, pass it
	 * to z_riscv_fault (via register a0).
	 * If z_riscv_fault shall return, set return address to
	 * no_reschedule to restore stack.
	 */
	mv a0, sp
	la ra, no_reschedule
	tail z_riscv_fault

is_kernel_syscall:
	/*
	 * A syscall is the result of an ecall instruction, in which case the
	 * MEPC will contain the address of the ecall instruction.
	 * Increment saved MEPC by 4 to prevent triggering the same ecall
	 * again upon exiting the ISR.
	 *
	 * It's safe to always increment by 4, even with compressed
	 * instructions, because the ecall instruction is always 4 bytes.
	 */
	lr t0, __struct_arch_esf_mepc_OFFSET(sp)
	addi t0, t0, 4
	sr t0, __struct_arch_esf_mepc_OFFSET(sp)

	/* Determine what to do. Operation code is in t0. */
	lr t0, __struct_arch_esf_t0_OFFSET(sp)

	.if RV_ECALL_RUNTIME_EXCEPT != 0; .err; .endif
	beqz t0, do_fault

#ifdef CONFIG_RISCV_ALWAYS_SWITCH_THROUGH_ECALL
	li t1, RV_ECALL_SCHEDULE
	bne t0, t1, skip_schedule
	lr a0, __struct_arch_esf_a0_OFFSET(sp)
	lr a1, __struct_arch_esf_a1_OFFSET(sp)

	j reschedule
skip_schedule:
#endif

	/* default fault code is K_ERR_KERNEL_OOPS */
	li a0, 3
	j 1f

do_fault:
	/* Handle RV_ECALL_RUNTIME_EXCEPT. Retrieve reason in a0, esf in A1. */
	lr a0, __struct_arch_esf_a0_OFFSET(sp)
1:	mv a1, sp

#ifdef CONFIG_EXCEPTION_DEBUG
	/*
	 * Restore the s0 we saved early in ISR entry
	 * so it shows up properly in the CSF.
	 */
	lr s0, __struct_arch_esf_s0_OFFSET(sp)

	/* Allocate space for caller-saved registers on current thread stack */
	addi sp, sp, -__callee_saved_t_SIZEOF

	/* Save callee-saved registers to be passed as 3rd arg */
	STORE_CALLEE_SAVED()		;
	mv a2, sp

#ifdef CONFIG_EXTRA_EXCEPTION_INFO
	/* Store csf's addr into esf (a1 still holds the pointer to the esf at this point) */
	sr a2 __struct_arch_esf_csf_OFFSET(a1)
#endif /* CONFIG_EXTRA_EXCEPTION_INFO */

	tail z_riscv_fatal_error_csf
#else
	tail z_riscv_fatal_error
#endif /* CONFIG_EXCEPTION_DEBUG */

is_interrupt:

	/* Increment _current_cpu->nested */
	lw t1, ___cpu_t_nested_OFFSET(s0)
	addi t2, t1, 1
	sw t2, ___cpu_t_nested_OFFSET(s0)
	bnez t1, on_irq_stack

	/* Switch to interrupt stack */
	mv t0, sp
	lr sp, ___cpu_t_irq_stack_OFFSET(s0)

	/*
	 * Save thread stack pointer on interrupt stack
	 * In RISC-V, stack pointer needs to be 16-byte aligned
	 */
	addi sp, sp, -16
	sr t0, 0(sp)

on_irq_stack:
	call __soc_handle_all_irqs

irq_done:
	/* Decrement _current_cpu->nested */
	lw t2, ___cpu_t_nested_OFFSET(s0)
	addi t2, t2, -1
	sw t2, ___cpu_t_nested_OFFSET(s0)
	bnez t2, no_reschedule

	/* nested count is back to 0: Return to thread stack */
	lr sp, 0(sp)

#ifdef CONFIG_STACK_SENTINEL
	call z_check_stack_sentinel
#endif

check_reschedule:

#ifdef CONFIG_MULTITHREADING

	/* Get pointer to current thread on this CPU */
	lr a1, ___cpu_t_current_OFFSET(s0)

	/*
	 * Get next thread to schedule with z_get_next_switch_handle().
	 * We pass it a NULL as we didn't save the whole thread context yet.
	 * If no scheduling is necessary then NULL will be returned.
	 */
	addi sp, sp, -16
	sr a1, 0(sp)
	mv a0, zero
	call z_get_next_switch_handle
	lr a1, 0(sp)
	addi sp, sp, 16
	beqz a0, no_reschedule

reschedule:

	/*
	 * Perform context switch:
	 * a0 = new thread
	 * a1 = old thread
	 */
	call z_riscv_switch

z_riscv_thread_start:
might_have_rescheduled:
	/* reload s0 with &_current_cpu as it might have changed or be unset */
	get_current_cpu s0

#endif /* CONFIG_MULTITHREADING */

no_reschedule:

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	/* Restore context at SOC level */
	addi a0, sp, __struct_arch_esf_soc_context_OFFSET
	jal ra, __soc_restore_context
#endif /* CONFIG_RISCV_SOC_CONTEXT_SAVE */

	/* Restore MEPC and MSTATUS registers */
	lr t0, __struct_arch_esf_mepc_OFFSET(sp)
	lr t2, __struct_arch_esf_mstatus_OFFSET(sp)
	csrw mepc, t0
	csrw mstatus, t2

	/* Restore s0 (it is no longer ours) */
	lr s0, __struct_arch_esf_s0_OFFSET(sp)

	/* Restore caller-saved registers from thread stack */
	DO_CALLER_SAVED(lr)

	/* remove esf from the stack */
	addi sp, sp, __struct_arch_esf_SIZEOF

	mret
