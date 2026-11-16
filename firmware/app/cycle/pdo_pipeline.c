#include "pdo_pipeline.h"
#include "cycle_config.h"
#include <string.h>

enum { SLOT_FREE, SLOT_WRITING, SLOT_READY, SLOT_LATCHED, SLOT_READING };

_Static_assert(__GCC_ATOMIC_INT_LOCK_FREE == 2 && sizeof(uint32_t) == sizeof(unsigned int),
               "PDO handoff requires lock-free 32-bit atomics");

static inline __attribute__((always_inline)) uint32_t load(const uint32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static inline __attribute__((always_inline)) void store(uint32_t *value, uint32_t next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

static inline __attribute__((always_inline)) uint32_t exchange(uint32_t *value, uint32_t next)
{
    return __atomic_exchange_n(value, next, __ATOMIC_ACQ_REL);
}

static inline __attribute__((always_inline)) bool claim(uint32_t *state, uint32_t from, uint32_t to)
{
    return __atomic_compare_exchange_n(state, &from, to, false,
                                        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

DCU_FAST_CODE void pdo_pipeline_fault(pdo_pipeline_t *p, uint32_t fault)
{
    __atomic_fetch_or(&p->faults, fault, __ATOMIC_RELEASE);
}

bool pdo_pipeline_configure(pdo_pipeline_t *p, uint32_t period_ns,
                            uint32_t advance_us, uint16_t output_size)
{
    memset(p, 0, sizeof(*p));
    p->ready = p->latched = PDO_PIPELINE_NO_SLOT;
    if (period_ns < DCU_SYNC_MIN_PERIOD_NS || period_ns > DCU_SYNC_MAX_PERIOD_NS ||
        !output_size || output_size > PDO_PIPELINE_RX_MAX ||
        advance_us < DCU_WORKER_PERIOD_US ||
        advance_us >= period_ns / 1000U - DCU_TX_BUILD_BUDGET_US)
        return false;
    p->period_ns = period_ns;
    p->period_us = period_ns / 1000U;
    p->advance_us = advance_us;
    p->output_size = output_size;
    return true;
}

void pdo_pipeline_enable(pdo_pipeline_t *p, bool enabled)
{
    store(&p->enabled, enabled && p->period_us != 0U);
}

bool pdo_pipeline_stage(pdo_pipeline_t *p, const void *data,
                        uint16_t size, uint32_t sequence)
{
    if (!load(&p->enabled) || size != p->output_size || data == NULL)
        return false;
    for (uint32_t i = 0; i < PDO_PIPELINE_SLOT_COUNT; ++i) {
        pdo_pipeline_slot_t *slot = &p->slot[i];
        if (!claim(&slot->state, SLOT_FREE, SLOT_WRITING))
            continue;
        memcpy(slot->data, data, size);
        slot->sequence = sequence;
        store(&slot->state, SLOT_READY);
        uint32_t old = exchange(&p->ready, i);
        if (old < PDO_PIPELINE_SLOT_COUNT)
            (void)claim(&p->slot[old].state, SLOT_READY, SLOT_FREE);
        return true;
    }
    __atomic_fetch_add(&p->rx_drops, 1U, __ATOMIC_RELAXED);
    pdo_pipeline_fault(p, PDO_PIPELINE_RX_OVERRUN);
    return false;
}

DCU_FAST_CODE void pdo_pipeline_sync(pdo_pipeline_t *p, uint32_t now_us)
{
    if (!load(&p->enabled))
        return;
    uint32_t epoch = load(&p->epoch) + 1U;
    if (load(&p->started) &&
        (!load(&p->published) || load(&p->published_epoch) != load(&p->epoch)))
        pdo_pipeline_fault(p, PDO_PIPELINE_TX_LATE);
    store(&p->sync_us, now_us);
    store(&p->published, 0U);

    uint32_t index = exchange(&p->ready, PDO_PIPELINE_NO_SLOT);
    if (index < PDO_PIPELINE_SLOT_COUNT &&
        claim(&p->slot[index].state, SLOT_READY, SLOT_LATCHED)) {
        p->slot[index].epoch = epoch;
        p->slot[index].sync_us = now_us;
        uint32_t old = exchange(&p->latched, index);
        if (old < PDO_PIPELINE_SLOT_COUNT) {
            (void)claim(&p->slot[old].state, SLOT_LATCHED, SLOT_FREE);
            pdo_pipeline_fault(p, PDO_PIPELINE_RX_OVERRUN);
        }
        store(&p->received, 1U);
    } else if (load(&p->received)) {
        pdo_pipeline_fault(p, PDO_PIPELINE_RX_MISSING);
    }
    store(&p->epoch, epoch);
    store(&p->started, 1U);
}

bool pdo_pipeline_acquire(pdo_pipeline_t *p, pdo_pipeline_view_t *view)
{
    uint32_t index = exchange(&p->latched, PDO_PIPELINE_NO_SLOT);
    if (index >= PDO_PIPELINE_SLOT_COUNT ||
        !claim(&p->slot[index].state, SLOT_LATCHED, SLOT_READING))
        return false;
    const pdo_pipeline_slot_t *slot = &p->slot[index];
    *view = (pdo_pipeline_view_t){slot->data, index, slot->sequence,
                                  slot->epoch, slot->sync_us};
    return true;
}

void pdo_pipeline_release(pdo_pipeline_t *p, const pdo_pipeline_view_t *view)
{
    if (view->slot < PDO_PIPELINE_SLOT_COUNT)
        (void)claim(&p->slot[view->slot].state, SLOT_READING, SLOT_FREE);
}

bool pdo_pipeline_publish_due(const pdo_pipeline_t *p, uint32_t now_us)
{
    uint32_t epoch = load(&p->epoch);
    uint32_t sync_us = load(&p->sync_us);
    if (!load(&p->enabled) || !load(&p->started) || epoch != load(&p->epoch) ||
        (int32_t)(now_us - sync_us) < 0 ||
        (load(&p->published) && load(&p->published_epoch) == epoch))
        return false;
    return (uint32_t)(now_us - sync_us) >=
           p->period_us - p->advance_us - DCU_TX_BUILD_BUDGET_US;
}

bool pdo_pipeline_mark_published(pdo_pipeline_t *p, uint32_t epoch, uint32_t now_us)
{
    if (!load(&p->enabled) || epoch != load(&p->epoch) ||
        (uint32_t)(now_us - load(&p->sync_us)) > p->period_us - p->advance_us) {
        pdo_pipeline_fault(p, PDO_PIPELINE_TX_LATE);
        return false;
    }
    store(&p->published_epoch, epoch);
    store(&p->published, 1U);
    if (epoch != load(&p->epoch)) {
        pdo_pipeline_fault(p, PDO_PIPELINE_TX_LATE);
        return false;
    }
    return true;
}

uint32_t pdo_pipeline_check(pdo_pipeline_t *p, uint32_t now_us)
{
    if (load(&p->enabled) && load(&p->started)) {
        uint32_t epoch = load(&p->epoch);
        uint32_t elapsed = now_us - load(&p->sync_us);
        if (epoch != load(&p->epoch) || (int32_t)elapsed < 0)
            return load(&p->faults);
        if (elapsed > p->period_us * 2U)
            pdo_pipeline_fault(p, PDO_PIPELINE_SYNC_LOST);
        if ((!load(&p->published) || load(&p->published_epoch) != epoch) &&
            elapsed > p->period_us - p->advance_us)
            pdo_pipeline_fault(p, PDO_PIPELINE_TX_LATE);
    }
    return load(&p->faults);
}
