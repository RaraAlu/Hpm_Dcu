#include "gateway_identity.h"
#include "cfg/gateway_id_journal_config.h"
#include "detail/gateway_identity_format.h"

#include <string.h>

_Static_assert(DCU_ID_SECTORS == 2, "Rollover requires two sectors");
_Static_assert(DCU_ID_SECTOR_BYTES % DCU_ID_PAGE_BYTES == 0 && DCU_ID_RECORD_BYTES == 16 &&
               DCU_ID_RECORD_BYTES <= DCU_ID_PAGE_BYTES, "Invalid journal geometry");

static uint64_t s_boot_id;

static uint32_t slot_offset(unsigned sector, unsigned slot)
{
    return sector * DCU_ID_SECTOR_BYTES + slot * DCU_ID_PAGE_BYTES;
}

static bool erased(const uint8_t *bytes, size_t size)
{
    for (size_t i = 0; i < size; ++i)
        if (bytes[i] != UINT8_MAX) return false;
    return true;
}

static uint32_t load_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 |
           (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

static void store_le32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0; i < 4U; ++i) bytes[i] = (uint8_t)(value >> (8U * i));
}

static bool scan(const dcu_gateway_id_storage_t *storage, dcu_gateway_id_scan_t *result)
{
    uint8_t bytes[DCU_ID_RECORD_BYTES];
    for (unsigned sector = 0; sector < DCU_ID_SECTORS; ++sector) {
        bool seen = false;
        uint32_t previous = 0;
        for (unsigned slot = 0; slot < DCU_ID_SLOTS; ++slot) {
            if (!storage->read(slot_offset(sector, slot), bytes, sizeof(bytes))) return false;
            if (erased(bytes, sizeof(bytes))) continue;
            result->next_slot[sector] = slot + 1U;
            uint32_t epoch = load_le32(bytes + 4);
            /* A torn program (1->0) or erase (0->1) cannot form another
             * valid epoch: every bit must agree with its paired complement. */
            if (load_le32(bytes) != DCU_ID_MAGIC ||
                load_le32(bytes + 8) != ~epoch || load_le32(bytes + 12) != ~DCU_ID_MAGIC)
                continue;
            if ((seen && epoch <= previous) || (result->found && epoch == result->epoch))
                return false;
            seen = true;
            previous = epoch;
            if (!result->found || epoch > result->epoch) {
                result->epoch = epoch;
                result->active_sector = sector;
                result->found = true;
            }
        }
    }
    return result->found;
}

static bool erase_and_check(const dcu_gateway_id_storage_t *storage, unsigned sector)
{
    if (!storage->erase_sector(sector)) return false;
    uint8_t page[DCU_ID_PAGE_BYTES];
    for (unsigned slot = 0; slot < DCU_ID_SLOTS; ++slot) {
        if (!storage->read(slot_offset(sector, slot), page, sizeof(page)) ||
            !erased(page, sizeof(page))) return false;
    }
    return true;
}

uint64_t dcu_gateway_ids_boot(const dcu_gateway_id_storage_t *storage)
{
    if (s_boot_id || !storage || !storage->read || !storage->program || !storage->erase_sector)
        return 0;
    dcu_gateway_id_scan_t journal = {0};
    if (!scan(storage, &journal) || journal.epoch == UINT32_MAX) return 0;
    unsigned sector = journal.active_sector;
    unsigned slot = journal.next_slot[sector];
    if (slot == DCU_ID_SLOTS) {
        /* The sector containing the highest valid epoch is never erased here. */
        sector ^= 1U;
        if (!erase_and_check(storage, sector)) return 0;
        slot = 0;
    }
    const uint32_t epoch = journal.epoch + 1U;
    const uint32_t offset = slot_offset(sector, slot);
    uint32_t aligned[DCU_ID_RECORD_BYTES / 4U];
    uint8_t *record = (uint8_t *)aligned;
    uint8_t verified[DCU_ID_RECORD_BYTES];
    store_le32(record, DCU_ID_MAGIC);
    store_le32(record + 4, epoch);
    store_le32(record + 8, ~epoch);
    store_le32(record + 12, ~DCU_ID_MAGIC);
    if (!storage->program(offset, record, DCU_ID_RECORD_BYTES) ||
        !storage->read(offset, verified, sizeof(verified)) ||
        memcmp(record, verified, sizeof(verified)) != 0)
        return 0;
    s_boot_id = (uint64_t)epoch << 32;
    return s_boot_id;
}
