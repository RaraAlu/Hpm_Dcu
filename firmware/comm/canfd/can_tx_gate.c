#include "realtime_port.h"

static uint32_t inhibited;

bool dcu_tx_inhibited(void)
{
    return __atomic_load_n(&inhibited, __ATOMIC_ACQUIRE) != 0U;
}
void dcu_tx_inhibit(void)
{
    __atomic_store_n(&inhibited, 1U, __ATOMIC_RELEASE);
}
void dcu_tx_allow(void)
{
    __atomic_store_n(&inhibited, 0U, __ATOMIC_RELEASE);
}
