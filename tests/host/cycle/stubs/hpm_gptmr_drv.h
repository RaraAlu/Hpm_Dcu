#ifndef TEST_GPTMR_H
#define TEST_GPTMR_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t reload; bool enable_cmp_output; } gptmr_channel_config_t;
#define gptmr_counter_type_normal 0U
#define GPTMR_CHANNEL_CNT_COUNTER_GET(value) (value)
uint32_t gptmr_channel_get_counter(void *timer, uint32_t channel, uint32_t type);
#define GPTMR_CH_RLD_STAT_MASK(channel) (1U << (channel))
#define GPTMR_CH_RLD_IRQ_MASK(channel) (1U << (channel))
bool gptmr_check_status(void *timer, uint32_t mask);
void gptmr_clear_status(void *timer, uint32_t mask);
void gptmr_enable_irq(void *timer, uint32_t mask);
void gptmr_start_counter(void *timer, uint32_t channel);
void gptmr_stop_counter(void *timer, uint32_t channel);
void gptmr_channel_reset_count(void *timer, uint32_t channel);
void gptmr_channel_config_update_reload(void *timer, uint32_t channel, uint32_t reload);
void gptmr_channel_get_default_config(void *timer, gptmr_channel_config_t *config);
void gptmr_channel_config(void *timer, uint32_t channel, gptmr_channel_config_t *config, bool start);
#endif
