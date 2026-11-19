#include "data_contract.h"
#include "detail/data_sii_layout.h"

#include <string.h>

static const uint8_t s_sii[DCU_DATA_SII_BYTES] = {
    /* Board PDI configuration and CRC (not the production identity). */
    [0] = 0x80, 0x0c, 0x81, 0x66,
    [14] = 0x62,
    [16] = DCU_DATA_LE32(DCU_DATA_VENDOR), DCU_DATA_LE32(DCU_DATA_PRODUCT), DCU_DATA_LE32(DCU_DATA_REVISION),
    [48] = DCU_DATA_LE16(DCU_DATA_SM0_ADDR), DCU_DATA_LE16(DCU_DATA_MBX_BYTES),
           DCU_DATA_LE16(DCU_DATA_SM1_ADDR), DCU_DATA_LE16(DCU_DATA_MBX_BYTES), 4, 0,
    [124] = DCU_DATA_LE16(DCU_DATA_SII_BYTES / 128U - 1U), 1, 0,
    /* General: CoE with SDO + SDO Info and uploadable fixed maps, no DC category. */
    [128] = 30, 0, 16, 0, 0, 0, 0, 0, 0, 0x13, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0x11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    40, 0, 2, 0, 1, 2, 3, 0xff,
    41, 0, 16, 0,
    DCU_DATA_SM(DCU_DATA_SM0_ADDR, DCU_DATA_MBX_BYTES, DCU_DATA_SM0_CTRL, 1),
    DCU_DATA_SM(DCU_DATA_SM1_ADDR, DCU_DATA_MBX_BYTES, DCU_DATA_SM1_CTRL, 2),
    DCU_DATA_SM(DCU_DATA_SM2_ADDR, DCU_DATA_RX_BYTES, DCU_DATA_SM2_CTRL, 3),
    DCU_DATA_SM(DCU_DATA_SM3_ADDR, DCU_DATA_TX_BYTES, DCU_DATA_SM3_CTRL, 4),
    DCU_DATA_PDO_HEADER(50, DCU_DATA_TX_MAP, DCU_DATA_TX_WORDS, 3),
    DCU_DATA_TX_ENTRIES(DCU_DATA_SII_TX)
    DCU_DATA_PDO_HEADER(51, DCU_DATA_RX_MAP, DCU_DATA_RX_WORDS, 2),
    DCU_DATA_RX_ENTRIES(DCU_DATA_SII_RX)
    0xff, 0xff,
};
static const uint8_t s_vector[DCU_DATA_VECTOR_BYTES] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x00, 0xff, 0x5a, 0xa5,
};

static uint8_t s_tx[DCU_DATA_TX_BYTES];
static uint32_t s_samples;

_Static_assert(DCU_DATA_TX_BYTES == DCU_DATA_RX_BYTES + 16U, "Echo tail must be four words");
_Static_assert(DCU_DATA_SM0_ADDR + DCU_DATA_MBX_BYTES <= DCU_DATA_SM1_ADDR, "SM0 overlaps SM1");
_Static_assert(DCU_DATA_SM1_ADDR + DCU_DATA_MBX_BYTES <= DCU_DATA_SM2_ADDR, "SM1 overlaps SM2");
_Static_assert(DCU_DATA_SM2_ADDR + 3U * DCU_DATA_RX_BYTES <= DCU_DATA_SM3_ADDR, "SM2 overlaps SM3");

static void put32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) data[i] = (uint8_t)(value >> (8U * i));
}

bool dcu_data_sii_read(uint32_t word, unsigned size, uint64_t *data)
{
    if (!data || (size != 4U && size != 8U) || word > (sizeof(s_sii) - size) / 2U) return false;
    uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) value |= (uint64_t)s_sii[word * 2U + i] << (8U * i);
    *data = value;
    return true;
}

void dcu_data_reset(void)
{
    memset(s_tx, 0, sizeof(s_tx));
    s_samples = 0;
    put32(s_tx + DCU_DATA_RX_BYTES + 8U, DCU_DATA_LAYOUT);
}

bool dcu_data_capture(const void *rx, size_t size)
{
    if (!rx || size != DCU_DATA_RX_BYTES) return false;
    memcpy(s_tx, rx, size);
    put32(s_tx + size, ++s_samples);
    put32(s_tx + size + 4U, DCU_DATA_RX_BYTES);
    put32(s_tx + size + 8U, DCU_DATA_LAYOUT);
    put32(s_tx + size + 12U, 1U);
    return true;
}

bool dcu_data_copy(void *tx, size_t size)
{
    if (!tx || size != sizeof(s_tx)) return false;
    memcpy(tx, s_tx, size);
    return true;
}

size_t dcu_data_read(uint16_t index, uint8_t sub, void *data, size_t capacity)
{
    if (!data) return 0;
    const uint8_t *bytes = NULL;
    size_t size = 0;
    uint8_t local[4] = {0};
    if (index == DCU_DATA_VECTOR_OBJECT && sub == 0) {
        bytes = s_vector;
        size = sizeof(s_vector);
    } else if (index == DCU_DATA_INFO_OBJECT && sub < 4U) {
        static const uint16_t values[] = {3U, DCU_DATA_LAYOUT, DCU_DATA_RX_BYTES, DCU_DATA_TX_BYTES};
        put32(local, values[sub]);
        bytes = local;
        size = sub ? 2U : 1U;
    } else if (index == DCU_DATA_RX_OBJECT || index == DCU_DATA_TX_OBJECT) {
        uint8_t words = index == DCU_DATA_RX_OBJECT ? DCU_DATA_RX_WORDS : DCU_DATA_TX_WORDS;
        if (sub > words) return 0;
        if (!sub) {
            local[0] = words;
            bytes = local;
            size = 1U;
        } else {
            bytes = s_tx + 4U * (sub - 1U);
            size = 4U;
        }
    }
    if (size == 0 || size > capacity) return 0;
    memcpy(data, bytes, size);
    return size;
}
