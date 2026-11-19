#include "gateway_identity.h"
#include "cfg/gateway_id_journal_config.h"

#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int s_file = -1;
static uint32_t s_programmed_at = UINT32_MAX;

static bool fault(const char *name)
{
    const char *mode = getenv("DCU_TEST_ID_FAULT");
    return mode && !strcmp(mode, name);
}

static bool file_read(uint32_t offset, void *data, size_t size)
{
    assert(offset < DCU_ID_JOURNAL_BYTES && size <= DCU_ID_JOURNAL_BYTES - offset);
    if (offset == s_programmed_at && fault("readback_failed")) return false;
    return pread(s_file, data, size, offset) == (ssize_t)size;
}

static bool durable_write(uint32_t offset, const void *data, size_t size)
{
    return pwrite(s_file, data, size, offset) == (ssize_t)size && fsync(s_file) == 0;
}

static bool file_program(uint32_t offset, const void *data, size_t size)
{
    assert(size == DCU_ID_RECORD_BYTES && offset % DCU_ID_PAGE_BYTES == 0);
    uint8_t previous[DCU_ID_RECORD_BYTES];
    if (!file_read(offset, previous, size)) return false;
    for (unsigned i = 0; i < size; ++i) assert(previous[i] == UINT8_MAX);
    if (fault("torn_program")) {
        assert(durable_write(offset, data, size / 2U));
        _exit(90);
    }
    if (!durable_write(offset, data, size)) return false;
    s_programmed_at = offset;
    if (fault("committed_then_reset")) _exit(90);
    return true;
}

static bool file_erase(unsigned sector)
{
    assert(sector < DCU_ID_SECTORS);
    uint8_t erased[DCU_ID_SECTOR_BYTES];
    memset(erased, 0xff, sizeof(erased));
    if (fault("torn_erase")) {
        assert(durable_write(sector * DCU_ID_SECTOR_BYTES, erased, sizeof(erased) / 2U));
        _exit(90);
    }
    return durable_write(sector * DCU_ID_SECTOR_BYTES, erased, sizeof(erased));
}

const dcu_gateway_id_storage_t *dcu_gateway_flash_storage(void)
{
    static const dcu_gateway_id_storage_t storage = {file_read, file_program, file_erase};
    const char *path = getenv("DCU_TEST_ID_JOURNAL");
    if (!path) return NULL;
    s_file = open(path, O_RDWR);
    struct stat info;
    if (s_file < 0 || fstat(s_file, &info) || info.st_size != DCU_ID_JOURNAL_BYTES) return NULL;
    return &storage;
}
