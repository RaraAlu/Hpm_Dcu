#ifndef TEST_CANFD_FREERTOS_H
#define TEST_CANFD_FREERTOS_H
typedef unsigned UBaseType_t;
void test_enter_critical(void);
void test_exit_critical(void);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
UBaseType_t test_set_interrupt_mask(void);
void test_clear_interrupt_mask(UBaseType_t saved);
#define portSET_INTERRUPT_MASK_FROM_ISR() test_set_interrupt_mask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(saved) test_clear_interrupt_mask(saved)
#endif
