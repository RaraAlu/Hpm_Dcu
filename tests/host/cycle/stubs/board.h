#ifndef TEST_BOARD_H
#define TEST_BOARD_H
#include <stdint.h>
#define HPM_GPTMR2 ((void *)2)
typedef struct { struct { uint32_t CNT; } CHANNEL[4]; } test_gptmr_t;
extern test_gptmr_t test_timebase;
#define HPM_GPTMR1 (&test_timebase)
typedef struct { volatile uint64_t MTIME; } test_machine_timer_t;
extern test_machine_timer_t test_machine_timer;
#define HPM_MCHTMR (&test_machine_timer)
#define clock_mchtmr0 3U
#define IRQn_ESC_SYNC0 10U
#define IRQn_GPTMR2 11U
#define clock_gptmr2 2U
#define clock_gptmr1 1U
#define clk_src_osc24m 0U
#define status_success 0
#define CSR_MSTATUS_MIE_MASK 8U
void restore_global_irq(uint32_t mask);
#define ATTR_RAMFUNC
#define SDK_DECLARE_EXT_ISR_M(irq, handler)
void intc_m_disable_irq(uint32_t irq);
void intc_m_enable_irq_with_priority(uint32_t irq, uint32_t priority);
uint32_t disable_global_irq(uint32_t mask);
void clock_add_to_group(uint32_t clock, uint32_t group);
uint32_t clock_get_frequency(uint32_t clock);
int clock_set_source_divider(uint32_t clock, uint32_t source, uint32_t divider);
#endif
