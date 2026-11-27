#if !defined(USE_SYSCALL_INTERRUPT_PRIORITY) || !USE_SYSCALL_INTERRUPT_PRIORITY || \
    (defined(USE_NONVECTOR_MODE) && USE_NONVECTOR_MODE) || !defined(__riscv)
#error This overlay requires the HPM vectored PLIC FreeRTOS port
#endif

#include "../../../cfg/cycle_config.h"

/* Vectored IRQ entry may save the IRQ priority rather than the task's threshold.
 * The HPM port masks MTIE/MSIE before incrementing xCriticalNesting. An IRQ
 * returning in that window must not unmask the kernel's notification lists. */
.macro dcu_restore_task_irq_mask
    csrr t1, mscratch
    bnez t1, .Lmask_done\@
    bnez t0, .Lkernel_mask\@
    csrr t1, mie
    andi t1, t1, 0x88  /* MTIE | MSIE */
    bnez t1, .Lset_mask\@  /* t0 = zero: normal task */
.Lkernel_mask\@:
    li t0, DCU_IRQ_KERNEL_PRIORITY
.Lset_mask\@:
    li t1, HPMICRO_PLIC_M_THRESHOLD
    sw t0, 0(t1)
.Lmask_done\@:
.endm
