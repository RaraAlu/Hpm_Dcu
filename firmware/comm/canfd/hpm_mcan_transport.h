#ifndef DCU_HPM_MCAN_TRANSPORT_H
#define DCU_HPM_MCAN_TRANSPORT_H

#include "hpm_common.h"
#include "can_transport.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board composition only; the millisecond timebase must already be running. */
hpm_stat_t dcu_mcan_init(const dcu_can_channel_config_t *channels, size_t count);
/* Optional 1 ms indicator callback; it does not maintain application time. */
void dcu_mcan_led_tick(void);

#ifdef __cplusplus
}
#endif
#endif
