/*
 * Copyright (c) 2024,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_ecat_e2p_emulation.h"
#include "cfg/ethercat_eeprom_config.h"
#include "hpm_romapi.h"
#include "hpm_ecat_hw.h"
#include "ecatslv.h"
#include "hpm_nor_flash.h"
#include <string.h>

extern unsigned char aEepromData[];

static nor_flash_config_t g_nor_cfg;
static e2p_t g_e2p_ctx;
static uint8_t g_sii[ESC_EEPROM_SIZE];
static uint8_t g_legacy[0x800];
static uint64_t g_dirty_pages;
static bool g_initialized;

static hpm_stat_t flash_read(uint8_t *buf, uint32_t addr, uint32_t size)
{
    return nor_flash_read(&g_nor_cfg, buf, addr, size);
}

static hpm_stat_t flash_write(const uint8_t *buf, uint32_t addr, uint32_t size)
{
    return nor_flash_write(&g_nor_cfg, buf, addr, size);
}

static hpm_stat_t flash_erase(uint32_t start_addr, uint32_t size)
{
    return nor_flash_erase(&g_nor_cfg, start_addr, size);
}

static bool valid_configuration_area(const uint8_t *data)
{
    return ecat_calculate_eeprom_config_data_checksum(data, 14U) == data[14]
        && data[15] == 0U;
}

static uint32_t get_u32(const uint8_t *data)
{
    uint32_t value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static hpm_stat_t read_pages(void)
{
    for (uint32_t page = 0; page < SII_PAGE_COUNT; page++) {
        hpm_stat_t stat = e2p_read(SII_PAGE_KEY + page, SII_PAGE_SIZE,
                                   g_sii + page * SII_PAGE_SIZE);
        if (stat != E2P_STATUS_OK) {
            return stat;
        }
    }
    return E2P_STATUS_OK;
}

static hpm_stat_t write_image(void)
{
    hpm_stat_t stat;
    /* Page zero carries ProductCode/Revision and is committed last. */
    for (uint32_t page = 1; page < SII_PAGE_COUNT; page++) {
        stat = e2p_write(SII_PAGE_KEY + page, SII_PAGE_SIZE,
                         g_sii + page * SII_PAGE_SIZE);
        if (stat != E2P_STATUS_OK) {
            return stat;
        }
    }
    return e2p_write(SII_PAGE_KEY, SII_PAGE_SIZE, g_sii);
}

static hpm_stat_t flush_dirty_pages(void)
{
    for (uint32_t page = 0; page < SII_PAGE_COUNT; page++) {
        uint64_t mask = UINT64_C(1) << page;
        if ((g_dirty_pages & mask) != 0U) {
            hpm_stat_t stat = e2p_write(SII_PAGE_KEY + page, SII_PAGE_SIZE,
                                        g_sii + page * SII_PAGE_SIZE);
            if (stat != E2P_STATUS_OK) {
                return stat;
            }
            g_dirty_pages &= ~mask;
        }
    }
    return E2P_STATUS_OK;
}

static bool read_legacy_image(void)
{
    for (uint32_t word = 0; word < sizeof(g_legacy) / EEPROM_WRITE_SIZE; word++) {
        if (e2p_read(word, EEPROM_WRITE_SIZE,
                     g_legacy + word * EEPROM_WRITE_SIZE) != E2P_STATUS_OK) {
            return false;
        }
    }
    return valid_configuration_area(g_legacy);
}

static hpm_stat_t retire_legacy_words(void)
{
    for (uint32_t word = 0; word < sizeof(g_legacy) / EEPROM_WRITE_SIZE; word++) {
        (void)e2p_delete(word);
    }
    return e2p_flush(E2P_FLUSH_FORCE);
}

hpm_stat_t ecat_flash_eeprom_init(void)
{
    g_nor_cfg.xpi_base = BOARD_APP_XPI_NOR_XPI_BASE;
    g_nor_cfg.base_addr = BOARD_FLASH_BASE_ADDRESS;
    g_nor_cfg.opt_header = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    g_nor_cfg.opt0 = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    g_nor_cfg.opt1 = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;

    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    hpm_stat_t stat = nor_flash_init(&g_nor_cfg);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    if (stat != status_success) {
        return status_fail;
    }

    g_e2p_ctx.config.start_addr = g_nor_cfg.base_addr + ECAT_EEPROM_FLASH_OFFSET;
    g_e2p_ctx.config.erase_size = ECAT_EEPROM_FLASH_SECTOR_SIZE;
    g_e2p_ctx.config.sector_cnt = ECAT_EEPROM_FLASH_SECTOR_CNT;
    g_e2p_ctx.config.version = 0x4553;
    g_e2p_ctx.config.flash_read = flash_read;
    g_e2p_ctx.config.flash_write = flash_write;
    g_e2p_ctx.config.flash_erase = flash_erase;

    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    stat = e2p_config(&g_e2p_ctx);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    if (stat != E2P_STATUS_OK) {
        return status_fail;
    }

    bool current = read_pages() == E2P_STATUS_OK && valid_configuration_area(g_sii);
    bool legacy = e2p_read(LEGACY_BACKUP_KEY, sizeof(g_legacy), g_legacy) == E2P_STATUS_OK
               && valid_configuration_area(g_legacy);

    if (!legacy && read_legacy_image()) {
        /* Save one compact rollback copy before deleting the old per-word records. */
        stat = e2p_write(LEGACY_BACKUP_KEY, sizeof(g_legacy), g_legacy);
        if (stat != E2P_STATUS_OK) {
            return status_fail;
        }
        legacy = true;
    }

    if (!current && legacy) {
        stat = retire_legacy_words();
        if (stat != E2P_STATUS_OK) {
            return status_fail;
        }
    }

    uint16_t alias = 0U;
    uint32_t serial = 0U;
    const uint8_t *identity = current ? g_sii : (legacy ? g_legacy : NULL);
    if (identity != NULL) {
        memcpy(&alias, identity + 8U, sizeof(alias));
        memcpy(&serial, identity + 28U, sizeof(serial));
    }

    bool update = !current
               || get_u32(g_sii + 20U) != get_u32(aEepromData + 20U)
               || get_u32(g_sii + 24U) < get_u32(aEepromData + 24U)
               || memcmp(g_sii + 32U, aEepromData + 32U,
                         ESC_EEPROM_SIZE - 32U) != 0;
    if (update) {
        memcpy(g_sii, aEepromData, sizeof(g_sii));
        memcpy(g_sii + 8U, &alias, sizeof(alias));
        memcpy(g_sii + 28U, &serial, sizeof(serial));
        g_sii[14] = ecat_calculate_eeprom_config_data_checksum(g_sii, 14U);
        g_sii[15] = 0U;
        stat = write_image();
        if (stat != E2P_STATUS_OK) {
            return status_fail;
        }
    }

    g_initialized = true;
    return status_success;
}

hpm_stat_t ecat_flash_eeprom_write(uint32_t word_addr, uint8_t *data)
{
    if ((nAlStatus & STATE_MASK) == STATE_OP || (nAlStatus & STATE_MASK) == STATE_SAFEOP)
        return status_fail;
    if (!g_initialized || word_addr >= ESC_EEPROM_SIZE / EEPROM_WRITE_SIZE) {
        return status_invalid_argument;
    }
    uint32_t byte_addr = word_addr * EEPROM_WRITE_SIZE;
    /* Runtime SII access is owned by the SSC task; flash writes are deferred
     * below, never hidden in an OP-time critical section. */
    if (memcmp(g_sii + byte_addr, data, EEPROM_WRITE_SIZE) != 0) {
        memcpy(g_sii + byte_addr, data, EEPROM_WRITE_SIZE);
        g_dirty_pages |= UINT64_C(1) << (byte_addr / SII_PAGE_SIZE);
    }
    return status_success;
}

hpm_stat_t ecat_flash_eeprom_read(uint32_t word_addr, uint8_t *data)
{
    if (!g_initialized || word_addr >= ESC_EEPROM_SIZE / EEPROM_WRITE_SIZE) {
        return status_invalid_argument;
    }
    memcpy(data, g_sii + word_addr * EEPROM_WRITE_SIZE, EEPROM_WRITE_SIZE);
    return status_success;
}

void ecat_flash_eeprom_format(void)
{
    /* Factory reset is intentionally inert so it cannot erase per-board identity. */
}

void ecat_flash_eeprom_flush(void)
{
    if (g_initialized && (nAlStatus & STATE_MASK) != STATE_OP &&
        (nAlStatus & STATE_MASK) != STATE_SAFEOP) {
        (void)flush_dirty_pages();
    }
}

void ecat_flash_eeprom_show_info(void)
{
    e2p_show_info();
}

uint32_t ecat_flash_eeprom_serial(void)
{
    return g_initialized ? get_u32(g_sii + 28U) : 0U;
}
