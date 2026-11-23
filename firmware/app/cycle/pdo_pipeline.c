#include "pdo_pipeline.h"
#include "detail/pdo_pipeline_slots.h"
#include "cycle_config.h"
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
#include "first_pdo_trace.h"
#include "realtime_port.h"
#endif
#include <string.h>

_Static_assert(__GCC_ATOMIC_INT_LOCK_FREE == 2 && sizeof(uint32_t) == sizeof(unsigned int),
               "PDO handoff requires lock-free 32-bit atomics");
_Static_assert(_Alignof(pdo_pipeline_slot_t) >= 4 && offsetof(pdo_pipeline_slot_t, data) % 4 == 0,
               "PDO slot data must support aligned ESC word copies");

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
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
    const uint32_t before = __atomic_fetch_or(&p->faults, fault, __ATOMIC_RELEASE);
    if (fault & ~before)
        DCU_FIRST_TRACE(DCU_FIRST_FAULT_BIT, fault & ~before, load(&p->epoch),
                        load(&p->published), load(&p->published_epoch));
#else
    __atomic_fetch_or(&p->faults, fault, __ATOMIC_RELEASE);
#endif
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

DCU_FAST_CODE uint8_t *pdo_pipeline_reserve(pdo_pipeline_t *p, uint16_t size, uint32_t *index)
{
    if (!load(&p->enabled) || size != p->output_size)
        return NULL;
    for (uint32_t i = 0; i < PDO_PIPELINE_SLOT_COUNT; ++i) {
        pdo_pipeline_slot_t *slot = &p->slot[i];
        if (claim(&slot->state, SLOT_FREE, SLOT_WRITING)) {
            *index = i;
            return slot->data;
        }
    }
    return NULL;
}

DCU_FAST_CODE bool pdo_pipeline_commit(pdo_pipeline_t *p, uint32_t index, uint32_t sequence,
                                      uint32_t image_complete_us)
{
    if (index >= PDO_PIPELINE_SLOT_COUNT ||
        __atomic_load_n(&p->slot[index].state, __ATOMIC_RELAXED) != SLOT_WRITING)
        return false;
    pdo_pipeline_slot_t *slot = &p->slot[index];
    if (!load(&p->enabled)) {
        store(&slot->state, SLOT_FREE);
        return false;
    }
    slot->sequence = sequence;
    slot->ready_us = image_complete_us;
    /* Sync0 only discovers this slot through the release exchange of ready. */
    __atomic_store_n(&slot->state, SLOT_READY, __ATOMIC_RELAXED);
    uint32_t old = exchange(&p->ready, index);
    if (old < PDO_PIPELINE_SLOT_COUNT)
        (void)claim(&p->slot[old].state, SLOT_READY, SLOT_FREE);
    return true;
}

bool pdo_pipeline_stage(pdo_pipeline_t *p, const void *data,
                        uint16_t size, uint32_t sequence)
{
    if (!load(&p->enabled) || size != p->output_size || data == NULL)
        return false;
    uint32_t index;
    uint8_t *destination = pdo_pipeline_reserve(p, size, &index);
    if (destination) {
        memcpy(destination, data, size);
        uint32_t complete_us = p->stage_clock_us ? p->stage_clock_us() : 0U;
        return pdo_pipeline_commit(p, index, sequence, complete_us);
    }
    __atomic_fetch_add(&p->rx_drops, 1U, __ATOMIC_RELAXED);
    pdo_pipeline_fault(p, PDO_PIPELINE_RX_OVERRUN);
    return false;
}

DCU_FAST_CODE bool pdo_pipeline_sync(pdo_pipeline_t *p, uint32_t now_us)
{
    if (!load(&p->enabled))
        return false;
    bool first_missing = false;
    uint32_t epoch = load(&p->epoch) + 1U;
    if (load(&p->started) &&
        (!load(&p->published) || load(&p->published_epoch) != load(&p->epoch))) {
        const bool missing = load(&p->cyclic_active) && load(&p->cyclic_epoch) == load(&p->epoch) &&
                             load(&p->cyclic_missing);
        pdo_pipeline_fault(p, missing ? PDO_PIPELINE_CAN_FEEDBACK_MISSING : PDO_PIPELINE_TX_LATE);
    }
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
        const uint32_t old = __atomic_fetch_or(&p->faults, PDO_PIPELINE_RX_MISSING, __ATOMIC_RELEASE);
        first_missing = !(old & PDO_PIPELINE_RX_MISSING);
    }
    store(&p->epoch, epoch);
    store(&p->started, 1U);
    return first_missing;
}

bool pdo_pipeline_acquire(pdo_pipeline_t *p, pdo_pipeline_view_t *view)
{
    uint32_t index = exchange(&p->latched, PDO_PIPELINE_NO_SLOT);
    if (index >= PDO_PIPELINE_SLOT_COUNT ||
        !claim(&p->slot[index].state, SLOT_LATCHED, SLOT_READING))
        return false;
    const pdo_pipeline_slot_t *slot = &p->slot[index];
    *view = (pdo_pipeline_view_t){slot->data, index, slot->sequence,
                                  slot->epoch, slot->sync_us, slot->ready_us};
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
    bool already_published = load(&p->published) && load(&p->published_epoch) == epoch;
    if (!load(&p->enabled) || !load(&p->started) || epoch != load(&p->epoch) ||
        (int32_t)(now_us - sync_us) < 0 || already_published)
        return false;
    if (load(&p->cyclic_active) && load(&p->cyclic_epoch) == epoch)
        return load(&p->cyclic_missing) == 0;
    return (uint32_t)(now_us - sync_us) >=
           p->period_us - p->advance_us - DCU_TX_BUILD_BUDGET_US;
}

void pdo_pipeline_update_cyclic(pdo_pipeline_t *p, uint32_t epoch, uint32_t missing)
{
    if (epoch != load(&p->epoch)) return;
    store(&p->cyclic_missing, missing);
    store(&p->cyclic_epoch, epoch);
    store(&p->cyclic_active, 1U);
}

uint32_t pdo_pipeline_cyclic_deadline(const pdo_pipeline_t *p, uint32_t sync_us)
{
    const uint32_t advance = p->advance_us > DCU_CYCLIC_SM3_ADVANCE_US ? p->advance_us : DCU_CYCLIC_SM3_ADVANCE_US;
    return sync_us + p->period_us - advance;
}

bool pdo_pipeline_mark_published(pdo_pipeline_t *p, uint32_t epoch, uint32_t now_us)
{
    const bool cyclic = load(&p->cyclic_active) && load(&p->cyclic_epoch) == epoch;
    const uint32_t sync_us = load(&p->sync_us);
    const uint32_t deadline = cyclic ? pdo_pipeline_cyclic_deadline(p, sync_us) : sync_us + p->period_us - p->advance_us;
    if (cyclic && load(&p->cyclic_missing)) {
        pdo_pipeline_fault(p, PDO_PIPELINE_CAN_FEEDBACK_MISSING);
        return false;
    }
    if (!load(&p->enabled) || epoch != load(&p->epoch) ||
        (int32_t)(now_us - sync_us) < 0 || (int32_t)(now_us - deadline) > 0) {
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
        const uint32_t sync_us = load(&p->sync_us);
        uint32_t elapsed = now_us - sync_us;
        const bool cyclic = load(&p->cyclic_active) && load(&p->cyclic_epoch) == epoch;
        const uint32_t missing = cyclic ? load(&p->cyclic_missing) : 0U;
        const uint32_t deadline = cyclic ? pdo_pipeline_cyclic_deadline(p, sync_us) : sync_us + p->period_us - p->advance_us;
        bool already_published = load(&p->published) && load(&p->published_epoch) == epoch;
        /* Sync0 can preempt this check and clear publication for the next epoch. */
        if (epoch != load(&p->epoch) || (int32_t)elapsed < 0)
            return load(&p->faults);
        if (elapsed > p->period_us * 2U)
            pdo_pipeline_fault(p, PDO_PIPELINE_SYNC_LOST);
        if (!already_published && (int32_t)(now_us - deadline) > 0)
            pdo_pipeline_fault(p, missing ? PDO_PIPELINE_CAN_FEEDBACK_MISSING : PDO_PIPELINE_TX_LATE);
    }
    return load(&p->faults);
}
