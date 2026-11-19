#ifndef TEST_LOOPBACK_INTERRUPT_H
#define TEST_LOOPBACK_INTERRUPT_H
#define SDK_DECLARE_EXT_ISR_M(irq, handler) \
    static void (*const registered_handler)(void) = handler
void intc_m_disable_irq(unsigned irq);
void intc_m_enable_irq(unsigned irq);
void intc_m_enable_irq_with_priority(unsigned irq, unsigned priority);
#endif
