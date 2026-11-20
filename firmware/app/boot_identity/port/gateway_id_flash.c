#include "gateway_identity.h"
#include "cfg/gateway_id_flash_config.h"
#include "cfg/gateway_id_journal_config.h"
#include "board.h"
#include "ecat_config.h"
#include "hpm_nor_flash.h"

_Static_assert(DCU_SESSION_JOURNAL_FLASH_OFFSET % DCU_ID_SECTOR_BYTES == 0U, "Journal is not aligned");
_Static_assert(DCU_SESSION_JOURNAL_FLASH_OFFSET >= ECAT_EEPROM_FLASH_OFFSET + FLASH_EEPROM_SIZE,
               "Journal overlaps reserved SII");
_Static_assert(DCU_SESSION_JOURNAL_FLASH_OFFSET + DCU_ID_JOURNAL_BYTES <= BOARD_FLASH_SIZE,
               "Journal exceeds Flash");

static nor_flash_config_t s_flash = {
    .xpi_base = BOARD_APP_XPI_NOR_XPI_BASE,
    .base_addr = BOARD_FLASH_BASE_ADDRESS,
    .opt_header = BOARD_APP_XPI_NOR_CFG_OPT_HDR,
    .opt0 = BOARD_APP_XPI_NOR_CFG_OPT_OPT0,
    .opt1 = BOARD_APP_XPI_NOR_CFG_OPT_OPT1,
};

static uint32_t address(uint32_t offset)
{
    return BOARD_FLASH_BASE_ADDRESS + DCU_SESSION_JOURNAL_FLASH_OFFSET + offset;
}

static bool read(uint32_t offset, void *data, size_t size)
{
    return offset < DCU_ID_JOURNAL_BYTES && size <= DCU_ID_JOURNAL_BYTES - offset &&
           nor_flash_read(&s_flash, data, address(offset), size) == status_success;
}

static bool program(uint32_t offset, const void *data, size_t size)
{
    if (offset >= DCU_ID_JOURNAL_BYTES || offset % DCU_ID_PAGE_BYTES || size != DCU_ID_RECORD_BYTES)
        return false;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t result = nor_flash_write(&s_flash, data, address(offset), size);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    return result == status_success;
}

static bool erase_sector(unsigned sector)
{
    if (sector >= DCU_ID_SECTORS) return false;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t result = nor_flash_erase(&s_flash, address(sector * DCU_ID_SECTOR_BYTES), DCU_ID_SECTOR_BYTES);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    return result == status_success;
}

const dcu_gateway_id_storage_t *dcu_gateway_flash_storage(void)
{
    static const dcu_gateway_id_storage_t storage = {read, program, erase_sector};
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t result = nor_flash_init(&s_flash);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    if (result != status_success || s_flash.sector_size != DCU_ID_SECTOR_BYTES) return NULL;
    uint32_t page_bytes = 0, capacity = 0;
    if (rom_xpi_nor_get_property(s_flash.xpi_base, &s_flash.nor_config,
                                 xpi_nor_property_page_size, &page_bytes) != status_success ||
        page_bytes != DCU_ID_PAGE_BYTES ||
        rom_xpi_nor_get_property(s_flash.xpi_base, &s_flash.nor_config,
                                 xpi_nor_property_total_size, &capacity) != status_success ||
        capacity < DCU_SESSION_JOURNAL_FLASH_OFFSET + DCU_ID_JOURNAL_BYTES)
        return NULL;
    return &storage;
}
