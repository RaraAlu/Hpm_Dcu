#ifndef DCU_PDO_PIPELINE_H
#define DCU_PDO_PIPELINE_H

#include "cycle_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PDO_PIPELINE_RX_MAX 784U
#define PDO_PIPELINE_SLOT_COUNT 3U
#define PDO_PIPELINE_NO_SLOT UINT32_MAX

enum {
    PDO_PIPELINE_RX_MISSING = 1U,
    PDO_PIPELINE_RX_OVERRUN = 2U,
    PDO_PIPELINE_TX_LATE = 4U,
    PDO_PIPELINE_SYNC_LOST = 8U,
    PDO_PIPELINE_CONFIG_CHANGED = 16U,
    PDO_PIPELINE_CAN_FEEDBACK_MISSING = 32U
};

typedef struct {
    uint32_t state;
    uint32_t sequence;
    uint32_t epoch;
    uint32_t sync_us;
    uint32_t ready_us;
    uint8_t data[PDO_PIPELINE_RX_MAX];
} pdo_pipeline_slot_t;

typedef struct {
    pdo_pipeline_slot_t slot[PDO_PIPELINE_SLOT_COUNT];
    uint32_t ready;
    uint32_t latched;
    uint32_t enabled;
    uint32_t started;
    uint32_t received;
    uint32_t epoch;
    uint32_t sync_us;
    uint32_t published_epoch;
    uint32_t published;
    uint32_t cyclic_epoch, cyclic_missing, cyclic_active;
    uint32_t faults;
    uint32_t rx_drops;
    uint32_t period_ns;
    uint32_t period_us;
    uint32_t advance_us;
    uint16_t output_size;
    bool ordered_outputs;
    uint32_t (*stage_clock_us)(void);
    uint32_t (*latch_clock_ticks)(void);
    uint32_t latch_ticks;
} pdo_pipeline_t;

typedef struct {
    const uint8_t *data;
    uint32_t slot;
    uint32_t sequence;
    uint32_t epoch;
    uint32_t sync_us;
    uint32_t ready_us;
} pdo_pipeline_view_t;

/* Offsets from the epoch's sync time; causal feedback has a separate deadline. */
static inline uint32_t pdo_pipeline_normal_deadline_offset(const pdo_pipeline_t *p)
{
    return p->period_us - p->advance_us;
}

static inline uint32_t pdo_pipeline_build_offset(const pdo_pipeline_t *p)
{
    return pdo_pipeline_normal_deadline_offset(p) - DCU_TX_BUILD_BUDGET_US;
}

/* Configure/reset only with the producer and Sync0 interrupts quiescent. */
bool pdo_pipeline_configure(pdo_pipeline_t *pipeline, uint32_t period_ns,
                            uint32_t advance_us, uint16_t output_size);
void pdo_pipeline_enable(pdo_pipeline_t *pipeline, bool enabled);
/* Single producer: a reservation is invisible to Sync0 until committed. */
uint8_t *pdo_pipeline_reserve(pdo_pipeline_t *pipeline, uint16_t size, uint32_t *index);
bool pdo_pipeline_commit(pdo_pipeline_t *pipeline, uint32_t index, uint32_t sequence,
                         uint32_t image_complete_us);
bool pdo_pipeline_stage(pdo_pipeline_t *pipeline, const void *data,
                        uint16_t size, uint32_t sequence);
/* True only on the first missing RxPDO of this run. */
bool pdo_pipeline_sync(pdo_pipeline_t *pipeline, uint32_t now_us);
bool pdo_pipeline_acquire(pdo_pipeline_t *pipeline, pdo_pipeline_view_t *view);
void pdo_pipeline_release(pdo_pipeline_t *pipeline, const pdo_pipeline_view_t *view);
bool pdo_pipeline_publish_due(const pdo_pipeline_t *pipeline, uint32_t now_us);
void pdo_pipeline_update_cyclic(pdo_pipeline_t *pipeline, uint32_t epoch, uint32_t missing);
uint32_t pdo_pipeline_cyclic_deadline(const pdo_pipeline_t *pipeline, uint32_t sync_us);
bool pdo_pipeline_mark_published(pdo_pipeline_t *pipeline, uint32_t epoch,
                                 uint32_t now_us);
uint32_t pdo_pipeline_check(pdo_pipeline_t *pipeline, uint32_t now_us);
void pdo_pipeline_fault(pdo_pipeline_t *pipeline, uint32_t fault);

#ifdef __cplusplus
}
#endif
#endif
