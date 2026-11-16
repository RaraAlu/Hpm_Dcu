#ifndef TEST_CANFD_INTERRUPT_H
#define TEST_CANFD_INTERRUPT_H
#define SDK_DECLARE_EXT_ISR_M(irq, handler) \
    static void (*const test_isr_##irq)(void) __attribute__((used)) = handler
void intc_m_disable_irq(unsigned irq);
void intc_m_enable_irq(unsigned irq);
void intc_m_enable_irq_with_priority(unsigned irq, unsigned priority);
#endif
