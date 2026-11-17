#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "hpm_ecat_e2p_emulation.h"
#include "hpm_nor_flash.h"
#include "ecatslv.h"
#include "cfg/ethercat_eeprom_config.h"
#include "eeprom.h"

UINT8 nAlStatus = STATE_PREOP;
static uint8_t pages[ESC_EEPROM_SIZE], legacy[2048], backup[2048];
static bool present[SII_PAGE_COUNT], have_legacy, have_backup;
static unsigned writes, last_key, deletes;
static int fail_page = -1;

uint32_t disable_global_irq(uint32_t mask) { return mask; }
void restore_global_irq(uint32_t mask) { (void)mask; }
hpm_stat_t nor_flash_init(nor_flash_config_t *cfg) { (void)cfg; return status_success; }
hpm_stat_t nor_flash_read(nor_flash_config_t *cfg, uint8_t *data, uint32_t addr, uint32_t size)
{ (void)cfg; (void)data; (void)addr; (void)size; abort(); }
hpm_stat_t nor_flash_write(nor_flash_config_t *cfg, const uint8_t *data, uint32_t addr, uint32_t size)
{ (void)cfg; (void)data; (void)addr; (void)size; abort(); }
hpm_stat_t nor_flash_erase(nor_flash_config_t *cfg, uint32_t addr, uint32_t size)
{ (void)cfg; (void)addr; (void)size; abort(); }

uint8_t ecat_calculate_eeprom_config_data_checksum(const uint8_t *data, size_t size)
{
    unsigned crc = 0xFFU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = ((crc * 2U) ^ ((crc & 128U) ? 7U : 0U)) & 255U;
    }
    return (uint8_t)crc;
}

hpm_stat_t e2p_config(e2p_t *context) { assert(context); return E2P_STATUS_OK; }
hpm_stat_t e2p_read(uint32_t key, uint16_t length, uint8_t *data)
{
    if (key >= SII_PAGE_KEY && key < SII_PAGE_KEY + SII_PAGE_COUNT) {
        unsigned page = key - SII_PAGE_KEY;
        assert(length == SII_PAGE_SIZE);
        if (!present[page]) return E2P_ERROR_NOT_FOUND;
        memcpy(data, pages + page * SII_PAGE_SIZE, length);
    } else if (key == LEGACY_BACKUP_KEY && have_backup) {
        assert(length == sizeof(backup));
        memcpy(data, backup, length);
    } else if (key < sizeof(legacy) / 2 && have_legacy) {
        assert(length == 2);
        memcpy(data, legacy + key * 2, length);
    } else {
        return E2P_ERROR_NOT_FOUND;
    }
    return E2P_STATUS_OK;
}
hpm_stat_t e2p_write(uint32_t key, uint16_t length, const uint8_t *data)
{
    if (key == LEGACY_BACKUP_KEY) {
        assert(length == sizeof(backup));
        memcpy(backup, data, length);
        have_backup = true;
    } else {
        assert(key >= SII_PAGE_KEY && key < SII_PAGE_KEY + SII_PAGE_COUNT);
        unsigned page = key - SII_PAGE_KEY;
        assert(length == SII_PAGE_SIZE);
        if ((int)page == fail_page) return E2P_ERROR;
        memcpy(pages + page * SII_PAGE_SIZE, data, length);
        present[page] = true;
    }
    ++writes;
    last_key = key;
    return E2P_STATUS_OK;
}
hpm_stat_t e2p_delete(uint32_t key) { assert(key < 1024); ++deletes; return E2P_STATUS_OK; }
hpm_stat_t e2p_flush(uint8_t mode) { assert(mode == E2P_FLUSH_FORCE); have_legacy = false; return E2P_STATUS_OK; }
void e2p_show_info(void) {}

static void identify(uint8_t *image)
{
    const uint16_t alias = 0x1234;
    const uint32_t serial = 0x89ABCDEF;
    memcpy(image + 8, &alias, sizeof(alias));
    memcpy(image + 28, &serial, sizeof(serial));
    image[14] = ecat_calculate_eeprom_config_data_checksum(image, 14);
}

static void check_image(bool identified)
{
    uint8_t expected[ESC_EEPROM_SIZE], actual[ESC_EEPROM_SIZE];
    memcpy(expected, aEepromData, sizeof(expected));
    if (identified) identify(expected);
    for (unsigned word = 0; word < ESC_EEPROM_SIZE / 2; ++word)
        assert(ecat_flash_eeprom_read(word, actual + word * 2) == status_success);
    assert(memcmp(actual, expected, sizeof(expected)) == 0);
    assert(memcmp(pages, expected, sizeof(expected)) == 0);
    assert(ecat_flash_eeprom_serial() == (identified ? 0x89ABCDEFU : 0));
    assert(last_key == SII_PAGE_KEY);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "fresh")) {
        assert(ecat_flash_eeprom_init() == status_success);
        assert(writes == SII_PAGE_COUNT);
        check_image(false);
    } else if (!strcmp(argv[1], "update")) {
        memcpy(pages, aEepromData, sizeof(pages));
        identify(pages);
        memset(present, 1, sizeof(present));
        pages[150] ^= 1U;
        assert(ecat_flash_eeprom_init() == status_success);
        assert(writes == SII_PAGE_COUNT);
        check_image(true);
        assert(ecat_flash_eeprom_init() == status_success);
        assert(writes == SII_PAGE_COUNT);
        ecat_flash_eeprom_format();
        check_image(true);
    } else if (!strcmp(argv[1], "legacy")) {
        memcpy(legacy, aEepromData, sizeof(legacy));
        identify(legacy);
        have_legacy = true;
        assert(ecat_flash_eeprom_init() == status_success);
        assert(have_backup && !have_legacy && deletes == 1024);
        check_image(true);
    } else if (!strcmp(argv[1], "failure")) {
        fail_page = 10;
        assert(ecat_flash_eeprom_init() == status_fail);
        assert(!present[0]);
        uint8_t data[2];
        assert(ecat_flash_eeprom_read(0, data) == status_invalid_argument);
        fail_page = -1;
        assert(ecat_flash_eeprom_init() == status_success);
        check_image(false);
    } else if (!strcmp(argv[1], "io")) {
        assert(ecat_flash_eeprom_init() == status_success);
        uint8_t data[2] = {0x12, 0x34}, readback[2];
        nAlStatus = STATE_OP;
        assert(ecat_flash_eeprom_write(4, data) == status_fail);
        nAlStatus = STATE_SAFEOP;
        assert(ecat_flash_eeprom_write(4, data) == status_fail);
        nAlStatus = STATE_PREOP;
        assert(ecat_flash_eeprom_read(8191, readback) == status_success);
        assert(ecat_flash_eeprom_read(8192, readback) == status_invalid_argument);
        assert(ecat_flash_eeprom_write(8192, data) == status_invalid_argument);
        assert(ecat_flash_eeprom_write(8191, data) == status_success);
        assert(writes == SII_PAGE_COUNT);
        assert(ecat_flash_eeprom_read(8191, readback) == status_success);
        assert(memcmp(data, readback, 2) == 0);
        fail_page = 63;
        ecat_flash_eeprom_flush();
        assert(writes == SII_PAGE_COUNT);
        fail_page = -1;
        ecat_flash_eeprom_flush();
        assert(writes == SII_PAGE_COUNT + 1);
        assert(memcmp(pages + 16382, data, 2) == 0);
    } else {
        abort();
    }
    return 0;
}
