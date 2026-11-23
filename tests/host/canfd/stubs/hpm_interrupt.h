#ifndef TEST_CANFD_INTERRUPT_H
#define TEST_CANFD_INTERRUPT_H
#define SDK_DECLARE_EXT_ISR_M(irq, handler) \
    static void (*const test_isr_##irq)(void) __attribute__((used)) = handler
void intc_m_disable_irq(unsigned irq);
void intc_m_enable_irq(unsigned irq);
void intc_m_enable_irq_with_priority(unsigned irq, unsigned priority);
#define CSR_MSTATUS_MIE_MASK 8U
#define fenceiorw() ((void)0)
static inline uint32_t disable_global_irq(uint32_t mask) { return mask; }
static inline void restore_global_irq(uint32_t mask) { (void)mask; }
#endif
