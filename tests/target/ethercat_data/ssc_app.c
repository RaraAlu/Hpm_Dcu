#include "digital_io.h"
#include "ecatslv.h"
#include "data_contract.h"
#include "detail/ssc_dictionary.h"
#ifdef DCU_DATA_BACKEND_MCAN
#include "mcan_echo.h"
#endif
#ifdef DCU_DATA_SESSION_GATE
#include "session_bridge.h"
#endif
#ifdef DCU_DATA_CIH408_MIT_ONCE
#include "mit_once.h"
#endif

static const TSDOINFOENTRYDESC s_rx_map_desc[] = {
    DCU_DATA_COUNT_DESC, DCU_DATA_RX_ENTRIES(DCU_DATA_MAP_DESC)
};
static const TSDOINFOENTRYDESC s_tx_map_desc[] = {
    DCU_DATA_COUNT_DESC, DCU_DATA_TX_ENTRIES(DCU_DATA_MAP_DESC)
};
static const TSDOINFOENTRYDESC s_assign_desc[] = {DCU_DATA_COUNT_DESC, DCU_DATA_U16_DESC};
static const TSDOINFOENTRYDESC s_rx_desc[] = {
    DCU_DATA_COUNT_DESC, DCU_DATA_RX_ENTRIES(DCU_DATA_RX_DESC)
};
static const TSDOINFOENTRYDESC s_tx_desc[] = {
    DCU_DATA_COUNT_DESC, DCU_DATA_TX_ENTRIES(DCU_DATA_TX_DESC)
};
static const TSDOINFOENTRYDESC s_info_desc[] = {
    DCU_DATA_COUNT_DESC, DCU_DATA_U16_DESC, DCU_DATA_U16_DESC, DCU_DATA_U16_DESC
};
static const TSDOINFOENTRYDESC s_vector_desc[] = {
    {DEFTYPE_OCTETSTRING, 8U * DCU_DATA_VECTOR_BYTES, ACCESS_READ}
};

/* SSC default reader stores the count in a word, then DWORDs as word pairs. */
static UINT16 s_rx_map[] = {DCU_DATA_RX_WORDS, DCU_DATA_RX_ENTRIES(DCU_DATA_RX_MAP_WORDS)};
static UINT16 s_tx_map[] = {DCU_DATA_TX_WORDS, DCU_DATA_TX_ENTRIES(DCU_DATA_TX_MAP_WORDS)};
static UINT16 s_rx_assign[] = {1, DCU_DATA_RX_MAP};
static UINT16 s_tx_assign[] = {1, DCU_DATA_TX_MAP};
static UINT16 s_rx_count = DCU_DATA_RX_WORDS;
static UINT16 s_tx_count = DCU_DATA_TX_WORDS;
static UINT16 s_info_count = 3;
static UINT16 s_vector_placeholder;

static const UCHAR s_rx_map_name[] = "RxPDO mapping\x00\xff";
static const UCHAR s_tx_map_name[] = "TxPDO mapping\x00\xff";
static const UCHAR s_rx_assign_name[] = "RxPDO assignment\x00\xff";
static const UCHAR s_tx_assign_name[] = "TxPDO assignment\x00\xff";
static const UCHAR s_rx_name[] = "RxPDO test words\x00\xff";
static const UCHAR s_tx_name[] = "TxPDO echo and status\x00\xff";
static const UCHAR s_info_name[] = "Loopback layout\x00Layout version\x00Rx bytes\x00Tx bytes\x00\xff";
static const UCHAR s_vector_name[] = "Read-only byte vector\x00\xff";

static UINT8 read_value(UINT16 index, UINT8 sub, UINT32 size, UINT16 MBXMEM *data, UINT8 complete)
{
    if (complete) return ABORTIDX_UNSUPPORTED_ACCESS;
    return dcu_data_read(index, sub, data, size) == size && size != 0U ?
        0 : ABORTIDX_PARAM_LENGTH_ERROR;
}

#ifdef DCU_DATA_SESSION_GATE
static UINT16 s_status_placeholder, s_activate_placeholder, s_result_placeholder;
static const UCHAR s_status_name[] = "Gateway status\x00\xff";
static const UCHAR s_activate_name[] = "Gateway activation\x00\xff";
static const UCHAR s_result_name[] = "Activation result\x00\xff";
static const TSDOINFOENTRYDESC s_status_desc[] = {{DEFTYPE_OCTETSTRING, 8U * DCU_SESSION_STATUS_BYTES, ACCESS_READ}};
static const TSDOINFOENTRYDESC s_activate_desc[] = {{DEFTYPE_OCTETSTRING, 8U * DCU_SESSION_ACTIVATE_BYTES, ACCESS_WRITE}};
static const TSDOINFOENTRYDESC s_result_desc[] = {{DEFTYPE_OCTETSTRING, 8U * DCU_SESSION_RESULT_BYTES, ACCESS_READ}};

static UINT8 read_session(UINT16 index, UINT8 sub, UINT32 size, UINT16 MBXMEM *data, UINT8 complete)
{
    if (complete) return ABORTIDX_UNSUPPORTED_ACCESS;
    if (sub) return ABORTIDX_SUBINDEX_NOT_EXISTING;
    return size && dcu_data_session_read(index, data, size) == size ? 0 : ABORTIDX_PARAM_LENGTH_ERROR;
}
#endif

TOBJECT ApplicationObjDic[] = {
    DCU_DATA_OD(DCU_DATA_RX_MAP, DEFTYPE_PDOMAPPING, DCU_DATA_RX_WORDS, OBJCODE_REC,
                s_rx_map_desc, s_rx_map_name, s_rx_map, NULL),
    DCU_DATA_OD(DCU_DATA_TX_MAP, DEFTYPE_PDOMAPPING, DCU_DATA_TX_WORDS, OBJCODE_REC,
                s_tx_map_desc, s_tx_map_name, s_tx_map, NULL),
    DCU_DATA_OD(0x1c12, DEFTYPE_UNSIGNED16, 1, OBJCODE_ARR,
                s_assign_desc, s_rx_assign_name, s_rx_assign, NULL),
    DCU_DATA_OD(0x1c13, DEFTYPE_UNSIGNED16, 1, OBJCODE_ARR,
                s_assign_desc, s_tx_assign_name, s_tx_assign, NULL),
    DCU_DATA_OD(DCU_DATA_INFO_OBJECT, 0, 3, OBJCODE_REC,
                s_info_desc, s_info_name, &s_info_count, read_value),
    DCU_DATA_OD(DCU_DATA_VECTOR_OBJECT, DEFTYPE_OCTETSTRING, 0, OBJCODE_VAR,
                s_vector_desc, s_vector_name, &s_vector_placeholder, read_value),
#ifdef DCU_DATA_SESSION_GATE
    DCU_DATA_OD(DCU_SESSION_STATUS_INDEX, DEFTYPE_OCTETSTRING, 0, OBJCODE_VAR,
                s_status_desc, s_status_name, &s_status_placeholder, read_session),
    DCU_DATA_OD(DCU_SESSION_ACTIVATE_INDEX, DEFTYPE_OCTETSTRING, 0, OBJCODE_VAR,
                s_activate_desc, s_activate_name, &s_activate_placeholder, NULL),
    DCU_DATA_OD(DCU_SESSION_RESULT_INDEX, DEFTYPE_OCTETSTRING, 0, OBJCODE_VAR,
                s_result_desc, s_result_name, &s_result_placeholder, read_session),
#endif
    DCU_DATA_OD(DCU_DATA_TX_OBJECT, 0, DCU_DATA_TX_WORDS, OBJCODE_REC,
                s_tx_desc, s_tx_name, &s_tx_count, read_value),
    DCU_DATA_OD(DCU_DATA_RX_OBJECT, 0, DCU_DATA_RX_WORDS, OBJCODE_REC,
                s_rx_desc, s_rx_name, &s_rx_count, read_value),
    {.Index = 0xffff},
};

/* Also reject downloads to the stock SSC communication objects. */
UINT8 __wrap_OBJ_Write(UINT16 index, UINT8 sub, UINT32 size, OBJCONST TOBJECT OBJMEM *object,
                      UINT16 MBXMEM *data, UINT8 complete)
{
#ifdef DCU_DATA_SESSION_GATE
    if (index == DCU_SESSION_ACTIVATE_INDEX) {
        if (!object || object->Index != index) return ABORTIDX_OBJECT_NOT_EXISTING;
        if (complete) return ABORTIDX_UNSUPPORTED_ACCESS;
        if (sub) return ABORTIDX_SUBINDEX_NOT_EXISTING;
        const UINT8 state = nAlStatus & (STATE_MASK | STATE_CHANGE);
        if (state != STATE_PREOP && state != STATE_SAFEOP && state != STATE_OP)
            return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        return dcu_data_session_activate(data, size) ? 0 : ABORTIDX_PARAM_LENGTH_ERROR;
    }
#endif
    (void)index;
    (void)sub;
    (void)size;
    (void)object;
    (void)data;
    (void)complete;
    return ABORTIDX_READ_ONLY_ENTRY;
}

UINT16 APPL_GenerateMapping(UINT16 *input_size, UINT16 *output_size)
{
    _Static_assert(sizeof(s_rx_map) - sizeof(UINT16) == DCU_DATA_RX_BYTES, "Rx map size");
    _Static_assert(sizeof(s_tx_map) - sizeof(UINT16) == DCU_DATA_TX_BYTES, "Tx map size");
    *input_size = DCU_DATA_TX_BYTES;
    *output_size = DCU_DATA_RX_BYTES;
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StartMailboxHandler(void) { return ALSTATUSCODE_NOERROR; }
UINT16 APPL_StopMailboxHandler(void) { return ALSTATUSCODE_NOERROR; }
void APPL_AckErrorInd(UINT16 transition) { (void)transition; }

UINT16 APPL_StartInputHandler(UINT16 *int_mask)
{
    (void)int_mask;
    if (bDcSyncActive) return ALSTATUSCODE_DCINVALIDSYNCCFG;
    if (nEscAddrOutputData != DCU_DATA_SM2_ADDR || nPdOutputSize != DCU_DATA_RX_BYTES)
        return ALSTATUSCODE_INVALIDSMOUTCFG;
    if (nEscAddrInputData != DCU_DATA_SM3_ADDR || nPdInputSize != DCU_DATA_TX_BYTES)
        return ALSTATUSCODE_INVALIDSMINCFG;
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StopInputHandler(void) { return ALSTATUSCODE_NOERROR; }
UINT16 APPL_StartOutputHandler(void)
{
    dcu_data_reset();
#if defined(DCU_DATA_SESSION_GATE)
    if (!dcu_data_mcan_healthy()) return ALSTATUSCODE_EXT_HARDWARE_NOT_READY;
#elif defined(DCU_DATA_BACKEND_MCAN)
    if (!dcu_data_mcan_enable()) return ALSTATUSCODE_EXT_HARDWARE_NOT_READY;
#endif
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StopOutputHandler(void)
{
#if defined(DCU_DATA_SESSION_GATE)
    dcu_data_session_step(false);
#elif defined(DCU_DATA_BACKEND_MCAN)
    dcu_data_mcan_disable();
#elif defined(DCU_DATA_CIH408_MIT_ONCE)
    mit_once_stop();
#endif
    dcu_data_reset();
    return ALSTATUSCODE_NOERROR;
}

void APPL_OutputMapping(UINT16 *data)
{
#if defined(DCU_DATA_SESSION_GATE)
    if (bEcatOutputUpdateRunning && (nAlStatus & (STATE_MASK | STATE_CHANGE)) == STATE_OP)
        dcu_data_session_pdo(data, nPdOutputSize);
#elif defined(DCU_DATA_BACKEND_MCAN)
    (void)dcu_data_mcan_stage(data, nPdOutputSize);
#elif defined(DCU_DATA_CIH408_MIT_ONCE)
    mit_once_stage(data, nPdOutputSize);
#else
    (void)dcu_data_capture(data, nPdOutputSize);
#endif
}

void APPL_InputMapping(UINT16 *data)
{
#ifdef DCU_DATA_CIH408_MIT_ONCE
    mit_once_input(data, nPdInputSize);
#else
    (void)dcu_data_copy(data, nPdInputSize);
#endif
}
void APPL_Application(void) {}
UINT16 APPL_GetDeviceID(void) { return 0; }
