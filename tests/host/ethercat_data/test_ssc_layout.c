#include "digital_io.h"
#include "ecatslv.h"
#include "applInterface.h"
#include "data_contract.h"

#include <assert.h>

#ifdef TEST_ALTERNATE_PROFILE
static const uint16_t rx_map = 0x1603U, tx_map = 0x1a07U, rx_obj = 0x7005U, tx_obj = 0x6009U;
static const uint8_t rx_words = 4U, tx_words = 8U;
#else
static const uint16_t rx_map = 0x1600U, tx_map = 0x1a00U, rx_obj = 0x7000U, tx_obj = 0x6000U;
static const uint8_t rx_words = 8U, tx_words = 12U;
#endif

/* The host exercises the stock OBJ_Read path without touching ESC registers. */
UINT8 nAlStatus = STATE_PREOP;
UINT16 nEscAddrOutputData, nEscAddrInputData, nPdOutputSize, nPdInputSize;
BOOL bDcSyncActive;
void (*pAPPL_CoeReadInd)(UINT16, UINT8, UINT8);

static void expect_read(uint16_t index, uint8_t sub, const void *expected, size_t size)
{
    TOBJECT *obj = ApplicationObjDic;
    while (obj->Index != index && obj->Index != 0xffff) ++obj;
    assert(obj->Index == index);
    assert(OBJ_GetObjectLength(index, sub, obj, 0) == size);
    UINT16 words[DCU_DATA_VECTOR_BYTES / 2] = {0};
    assert(OBJ_Read(index, sub, size, obj, words, 0) == 0);
    assert(memcmp(words, expected, size) == 0);
    assert(OBJ_Write(index, sub, size, obj, words, 0) == ABORTIDX_READ_ONLY_ENTRY);
}

static void check_map(uint16_t assign, uint16_t map, uint16_t object, uint8_t count)
{
    const uint8_t one = 1;
    expect_read(assign, 0, &one, 1);
    expect_read(assign, 1, &map, 2);
    expect_read(map, 0, &count, 1);
    for (uint8_t sub = 1; sub <= count; ++sub) {
        const uint8_t entry[] = {32, sub, object & 0xff, object >> 8};
        expect_read(map, sub, entry, sizeof(entry));
    }
}

int main(void)
{
    const uint8_t vector[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0, 0xff, 0x5a, 0xa5};
    const uint8_t version[] = {1, 0};
    const uint8_t rx_size[] = {4U * rx_words, 0};
    const uint8_t tx_size[] = {4U * tx_words, 0};
    expect_read(0x2100, 1, version, sizeof(version));
    expect_read(0x2100, 2, rx_size, sizeof(rx_size));
    expect_read(0x2100, 3, tx_size, sizeof(tx_size));
    expect_read(0x2101, 0, vector, sizeof(vector));
    check_map(0x1c12, rx_map, rx_obj, rx_words);
    check_map(0x1c13, tx_map, tx_obj, tx_words);

    UINT16 mask = 0;
    assert(APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize) == 0);
    assert(nPdInputSize == tx_size[0] && nPdOutputSize == rx_size[0]);
    nEscAddrOutputData = 0x1100;
    nEscAddrInputData = 0x1200;
    assert(APPL_StartInputHandler(&mask) == 0);
    nEscAddrInputData += 4;
    assert(APPL_StartInputHandler(&mask) == ALSTATUSCODE_INVALIDSMINCFG);
    nEscAddrInputData -= 4;
    nPdOutputSize -= 4;
    assert(APPL_StartInputHandler(&mask) == ALSTATUSCODE_INVALIDSMOUTCFG);
    nPdOutputSize += 4;
    bDcSyncActive = 1;
    assert(APPL_StartInputHandler(&mask) == ALSTATUSCODE_DCINVALIDSYNCCFG);
    bDcSyncActive = 0;

    nAlStatus = STATE_OP;
    UINT16 rx[DCU_DATA_RX_WORDS * 2], tx[DCU_DATA_TX_WORDS * 2];
    for (unsigned i = 0; i < sizeof(rx); ++i) ((uint8_t *)rx)[i] = i;
    assert(APPL_StartOutputHandler() == 0);
    APPL_OutputMapping(rx);
    APPL_InputMapping(tx);
    assert(memcmp(tx, rx, sizeof(rx)) == 0);
    const uint8_t tail[] = {1, 0, 0, 0, rx_size[0], 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    assert(memcmp((const uint8_t *)tx + sizeof(rx), tail, sizeof(tail)) == 0);
    expect_read(rx_obj, rx_words, (const uint8_t *)rx + sizeof(rx) - 4U, 4);
    expect_read(tx_obj, tx_words, tail + 12, 4);
    assert(APPL_StopOutputHandler() == 0);
    APPL_InputMapping(tx);
    assert(tx[2U * tx_words - 2U] == 0 && tx[2U * tx_words - 1U] == 0);
}
