#ifndef DCU_GATEWAY_IDENTITY_H
#define DCU_GATEWAY_IDENTITY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* Offsets are relative to the start of the dedicated two-sector journal. */
    bool (*read)(uint32_t offset, void *data, size_t size);
    bool (*program)(uint32_t offset, const void *data, size_t size);
    bool (*erase_sector)(unsigned sector);
} dcu_gateway_id_storage_t;

#ifdef __cplusplus
extern "C" {
#endif

const dcu_gateway_id_storage_t *dcu_gateway_flash_storage(void);
/* Single owner. Boot reserves and verifies a durable epoch before returning it.
 * Zero means no ID can be used; an erased journal is never auto-provisioned. */
uint64_t dcu_gateway_ids_boot(const dcu_gateway_id_storage_t *storage);
/* Allocate an ID in the reserved boot epoch without touching storage. */
uint64_t dcu_gateway_ids_next(void);

#ifdef __cplusplus
}
#endif

#endif
