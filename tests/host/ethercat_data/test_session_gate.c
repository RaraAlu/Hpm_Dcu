#include "digital_io.h"
#include "ecatslv.h"
#include "applInterface.h"
#include "data_contract.h"
#include "gateway_session.h"
#include "mcan_echo.h"
#include "session_bridge.h"
#include "gateway_identity.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

UINT8 nAlStatus = STATE_PREOP;
UINT16 nEscAddrOutputData, nEscAddrInputData, nPdOutputSize, nPdInputSize;
BOOL bDcSyncActive, bEcatOutputUpdateRunning;
void (*pAPPL_CoeReadInd)(UINT16, UINT8, UINT8);

unsigned test_mcan_sent(void);
void test_mcan_advance(unsigned us);
void test_mcan_finish_frame(bool matches);

static TOBJECT *object(uint16_t index)
{
    TOBJECT *obj = ApplicationObjDic;
    while (obj->Index != index && obj->Index != 0xffffU) ++obj;
    assert(obj->Index == index);
    return obj;
}

static void upload(uint16_t index, UINT16 *data, unsigned size)
{
    TOBJECT *obj = object(index);
    assert(OBJ_GetObjectLength(index, 0, obj, 0) == size);
    assert(OBJ_Read(index, 0, size, obj, data, 0) == 0);
}

static void write_byte(UINT16 *data, unsigned index, uint8_t value)
{
    ((uint8_t *)data)[index] = value;
}

static uint64_t read_u64(const UINT16 *data, unsigned offset)
{
    const uint8_t *bytes = (const uint8_t *)data + offset;
    uint64_t value = 0;
    for (unsigned i = 0; i < 8U; ++i) value |= (uint64_t)bytes[i] << (8U * i);
    return value;
}

static void write_session(UINT16 *pdo, uint64_t session)
{
    for (unsigned i = 0; i < 8U; ++i) write_byte(pdo, i, (uint8_t)(session >> (8U * i)));
}

static void status_record(UINT16 status[40], unsigned state)
{
    upload(0x2102, status, 80);
    const uint8_t *bytes = (const uint8_t *)status;
    assert(bytes[0] == 2 && bytes[1] == 0 && bytes[2] == state);
    assert(bytes[3] == (state == DCU_GATEWAY_READY || state == DCU_GATEWAY_RUNNING));
}

static void request_from_status(UINT16 request[32], const UINT16 status[40])
{
    memset(request, 0, 64U);
    memcpy(request, status + 2, 40U);        /* ID, revision and digest */
    memcpy(request + 20, status + 22, 12U); /* boot ID and generation */
    write_byte(request, 52, 1);
    write_byte(request, 56, 1);
}

static uint64_t confirm(UINT16 request[32], unsigned code, unsigned state)
{
    TOBJECT *obj = object(0x2103);
    assert(OBJ_GetObjectLength(0x2103, 0, obj, 0) == 64);
    assert(OBJ_Write(0x2103, 0, 64, obj, request, 0) == 0);
    UINT16 result[8];
    upload(0x2104, result, 16);
    const uint8_t *bytes = (const uint8_t *)result;
    assert(bytes[0] == ((uint8_t *)request)[52]);
    assert(bytes[4] == code && bytes[5] == 0 && bytes[6] == state && bytes[7] == 0);
    const uint64_t session = read_u64(result, 8);
    assert((session != 0) == (code == DCU_GATEWAY_ACCEPTED));
    return session;
}

static void finish_sample(void)
{
    for (unsigned i = 0; i < DCU_DATA_RX_BYTES / 8U; ++i) {
        const unsigned previous = test_mcan_sent();
        dcu_data_mcan_poll();
        assert(test_mcan_sent() == previous + 1U);
        test_mcan_finish_frame(true);
    }
}

static void check_feedback(const UINT16 *rx)
{
    UINT16 tx[DCU_DATA_TX_BYTES / 2];
    APPL_InputMapping(tx);
    const uint8_t *bytes = (const uint8_t *)tx;
    assert(bytes[DCU_DATA_RX_BYTES + 12] == (rx ? 1U : 0U));
    if (rx) assert(memcmp(tx, rx, DCU_DATA_RX_BYTES) == 0);
}

static void enter_op(void)
{
    assert(APPL_StartOutputHandler() == 0);
    nAlStatus = STATE_OP;
    bEcatOutputUpdateRunning = true;
    dcu_data_session_step(true);
}

static void assert_not_staged(UINT16 *rx)
{
    const unsigned previous = test_mcan_sent();
    APPL_OutputMapping(rx);
    dcu_data_mcan_poll();
    assert(test_mcan_sent() == previous);
}

static void replay_file(bool save, UINT16 request[32], UINT16 rx[DCU_DATA_RX_BYTES / 2])
{
    const char *path = getenv("DCU_TEST_REPLAY");
    assert(path);
    FILE *fp = fopen(path, save ? "wb" : "rb");
    assert(fp);
    if (save) {
        assert(fwrite(request, 1, 64, fp) == 64);
        assert(fwrite(rx, 1, DCU_DATA_RX_BYTES, fp) == DCU_DATA_RX_BYTES);
    } else {
        assert(fread(request, 1, 64, fp) == 64);
        assert(fread(rx, 1, DCU_DATA_RX_BYTES, fp) == DCU_DATA_RX_BYTES);
    }
    assert(fclose(fp) == 0);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "allocate")) {
        const uint64_t boot = dcu_gateway_ids_boot(dcu_gateway_flash_storage());
        const uint64_t one = dcu_gateway_ids_next(), two = dcu_gateway_ids_next();
        printf("%" PRIu64 " %" PRIu64 " %" PRIu64 "\n", boot, one, two);
        return 0;
    }
    uint64_t identity = 0;
    assert(dcu_data_sii_read(10U, 4U, &identity) && identity == 0x7fff0005U);
    assert(dcu_data_mcan_init());
    if (!strcmp(argv[1], "unprovisioned")) {
        uint8_t payload[DCU_DATA_RX_BYTES] = {0};
        assert(!dcu_data_session_boot());
        assert(!dcu_data_mcan_stage(payload, sizeof(payload)));
        dcu_data_mcan_poll();
        assert(test_mcan_sent() == 0);
        return 0;
    }
    assert(dcu_data_session_boot());
    dcu_data_reset();
    assert(APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize) == 0);
    UINT16 sm_mask = 0;
    nEscAddrOutputData = DCU_DATA_SM2_ADDR;
    nEscAddrInputData = DCU_DATA_SM3_ADDR;
    assert(APPL_StartInputHandler(&sm_mask) == 0);
    UINT16 status[40], request[32], rx[DCU_DATA_RX_BYTES / 2] = {0};
    status_record(status, DCU_GATEWAY_PREPARING);
    request_from_status(request, status);
    assert(OBJ_Write(0x2103, 0, 63, object(0x2103), request, 0) == ABORTIDX_PARAM_LENGTH_ERROR);
    assert(OBJ_Write(0x2103, 0, 64, object(0x2103), request, 1) == ABORTIDX_UNSUPPORTED_ACCESS);
    assert(OBJ_Write(0x2102, 0, 80, object(0x2102), status, 0) == ABORTIDX_READ_ONLY_ENTRY);
    nAlStatus = STATE_PREOP | STATE_CHANGE;
    assert(OBJ_Write(0x2103, 0, 64, object(0x2103), request, 0) ==
           ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    nAlStatus = STATE_PREOP;
    assert(confirm(request, DCU_GATEWAY_NOT_READY, DCU_GATEWAY_PREPARING) == 0);
    assert(test_mcan_sent() == 0U);

    dcu_data_session_step(false);
    if (!strcmp(argv[1], "missing_probe") || !strcmp(argv[1], "wrong_probe")) {
        dcu_data_mcan_poll();
        assert(test_mcan_sent() == 1U);
        status_record(status, DCU_GATEWAY_PREPARING);
        if (!strcmp(argv[1], "wrong_probe")) {
            test_mcan_finish_frame(false);
        } else {
            test_mcan_advance(5000U);
            dcu_data_mcan_poll();
        }
        dcu_data_session_step(false);
        status_record(status, DCU_GATEWAY_HOLD);
        assert(!dcu_data_mcan_healthy());
        check_feedback(NULL);
        return 0;
    }
    finish_sample();
    status_record(status, DCU_GATEWAY_READY);
    check_feedback(NULL);

    if (!strcmp(argv[1], "ready_timeout")) {
        test_mcan_advance(50000U);
        dcu_data_session_step(false);
        status_record(status, DCU_GATEWAY_HOLD);
        assert(((uint8_t *)status)[68] == DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT);
        request_from_status(request, status);
        assert(confirm(request, DCU_GATEWAY_NOT_READY, DCU_GATEWAY_HOLD) == 0);
        check_feedback(NULL);
        return 0;
    }
    if (!strcmp(argv[1], "op_first")) {
        enter_op();
        write_session(rx, read_u64(status, 44) | 1U);
        rx[4] = 1;
        assert_not_staged(rx);
        check_feedback(NULL);
    }
    request_from_status(request, status);
    write_byte(request, 8, ((uint8_t *)request)[8] ^ 1U);
    assert(confirm(request, DCU_GATEWAY_CONFIG_MISMATCH, DCU_GATEWAY_READY) == 0);
    request_from_status(request, status);
    write_byte(request, 40, ((uint8_t *)request)[40] ^ 2U);
    assert(confirm(request, DCU_GATEWAY_STALE_CONTEXT, DCU_GATEWAY_READY) == 0);
    request_from_status(request, status);
    UINT16 stale_request[32], stale_rx[DCU_DATA_RX_BYTES / 2];
    const bool replay = !strcmp(argv[1], "replay_after_reset");
    if (replay) {
        replay_file(false, stale_request, stale_rx);
        assert(read_u64(stale_request, 40) < read_u64(status, 44));
        assert(confirm(stale_request, DCU_GATEWAY_STALE_CONTEXT, DCU_GATEWAY_READY) == 0);
    }
    const uint64_t session = confirm(request, DCU_GATEWAY_ACCEPTED, DCU_GATEWAY_READY);
    assert(session == (read_u64(status, 44) | 1U));
    assert(session != 0 && confirm(request, DCU_GATEWAY_ACCEPTED, DCU_GATEWAY_READY) == session);

    if (!strcmp(argv[1], "activation_timeout")) {
        /* Refresh real diagnostic samples so the confirmation deadline is what expires. */
        for (unsigned i = 0; i < 499U; ++i) {
            test_mcan_advance(9600U);
            dcu_data_session_step(false);
            finish_sample();
        }
        test_mcan_advance(10001U);
        dcu_data_session_step(false);
        status_record(status, DCU_GATEWAY_HOLD);
        check_feedback(NULL);
        return 0;
    }

    write_session(rx, session);
    if (nAlStatus != STATE_OP) {
        rx[4] = 1;
        assert_not_staged(rx);
        enter_op();
    }
    if (replay) {
        /* Same ordinal and fresh sequence, differing only in the persisted boot epoch. */
        const uint64_t old_session = read_u64(stale_rx, 0);
        assert((uint32_t)old_session == (uint32_t)session && old_session < session);
        assert_not_staged(stale_rx);
        check_feedback(NULL);
    }
    rx[4] = 0;
    assert_not_staged(rx);
    write_session(rx, session + 1U);
    rx[4] = 1;
    assert_not_staged(rx);
    write_session(rx, session);
    rx[6] = 0xa55aU;
    APPL_OutputMapping(rx);
    status_record(status, DCU_GATEWAY_RUNNING);
    check_feedback(NULL);
    finish_sample();
    check_feedback(rx);
    if (!strcmp(argv[1], "record_previous")) {
        replay_file(true, request, rx);
        return 0;
    }
    if (replay) {
        stale_rx[4] = 2U;
        assert_not_staged(stale_rx);
        check_feedback(rx);
    }
    assert_not_staged(rx);
    assert(confirm(request, DCU_GATEWAY_ACCEPTED, DCU_GATEWAY_RUNNING) == session);
    write_byte(request, 52, 2);
    write_byte(request, 56, 2);
    assert(confirm(request, DCU_GATEWAY_BUSY, DCU_GATEWAY_RUNNING) == 0);

    if (!strcmp(argv[1], "command_timeout")) {
        test_mcan_advance(49600U);
        dcu_data_session_step(true);
        status_record(status, DCU_GATEWAY_HOLD);
        assert(((uint8_t *)status)[68] == DCU_GATEWAY_REASON_MASTER_TIMEOUT);
        check_feedback(NULL);
        assert_not_staged(rx);
        return 0;
    }
    rx[4] = 2;
    APPL_OutputMapping(rx);
    dcu_data_mcan_poll();
    const unsigned sent = test_mcan_sent();
    assert(APPL_StopOutputHandler() == 0);
    nAlStatus = STATE_SAFEOP;
    bEcatOutputUpdateRunning = false;
    test_mcan_finish_frame(true);
    dcu_data_mcan_poll();
    status_record(status, DCU_GATEWAY_HOLD);
    check_feedback(NULL);
    assert(read_u64(status, 60) == 0);
    assert(test_mcan_sent() == sent);
    write_byte(request, 52, 1);
    write_byte(request, 56, 1);
    assert(confirm(request, DCU_GATEWAY_NOT_READY, DCU_GATEWAY_HOLD) == 0);
    enter_op();
    assert_not_staged(rx);
}
