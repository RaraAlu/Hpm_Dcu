#if !defined(USE_SYSCALL_INTERRUPT_PRIORITY) || !USE_SYSCALL_INTERRUPT_PRIORITY || \
    (defined(USE_NONVECTOR_MODE) && USE_NONVECTOR_MODE) || !defined(__riscv)
#error This overlay requires the HPM vectored PLIC FreeRTOS port
#endif

/* A task outside a critical section must not inherit an ISR's PLIC threshold.
 * Preserve the SDK's saved threshold for nested ISRs and critical tasks. */
.macro dcu_restore_task_irq_mask
    bnez t0, .Lmask_done\@
    csrr t0, mscratch
    bnez t0, .Lmask_done\@
    li t1, HPMICRO_PLIC_M_THRESHOLD
    sw zero, 0(t1)
.Lmask_done\@:
.endm
