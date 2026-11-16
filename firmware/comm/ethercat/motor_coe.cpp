#include "motor_ethercat.h"
#include "motor_dictionary.hpp"
#include "motor_pdo_layout.hpp"
#include "motor_application.hpp"
#include "ethercat_canfd_gateway.h"
extern "C" {
#include "digital_io.h"
#include "ecatslv.h"
#include "sdoserv.h"
}
#include <cstring>

using namespace dcu::ethercat::motor_dictionary;
using namespace dcu::app::motor_application;
namespace pdo_layout = dcu::ethercat::pdo_layout;

static bool preop()
{
    return (nAlStatus & STATE_MASK) == STATE_PREOP && configuration_writable();
}

UINT8 MC_AssignWrite(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    if (!preop())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    TMC_ASSIGN *v = index == 0x1C12 ? &gRxAssign : &gTxAssign;
    if (sub > pdo_layout::assignment_capacity)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (bytes != (sub ? 2u : 1u))
        return ABORTIDX_PARAM_LENGTH_ERROR;
    uint16_t value = 0;
    memcpy(&value, data, bytes);
    if (!sub) {
        if (value > pdo_layout::assignment_capacity)
            return ABORTIDX_VALUE_EXCEEDED;
        v->u16SubIndex0 = value;
    } else {
        if (v->u16SubIndex0)
            return ABORTIDX_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0;
        v->aEntries[sub - 1] = value;
    }
    return 0;
}
UINT8 MC_OD_Read(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub) {
        if (bytes != 1)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        *((uint8_t *)data) = (uint8_t)o->count;
        return 0;
    }
    for (unsigned i = 0; i < mc_field_count; i++) {
        const mc_field_t *f = &mc_fields[i];
        if (f->index != index || f->sub != sub)
            continue;
        if (!(f->access & 1))
            return ABORTIDX_WRITE_ONLY_ENTRY;
        if (bytes != f->bytes)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        memcpy(data, o->data + f->offset, bytes);
        return 0;
    }
    return ABORTIDX_SUBINDEX_NOT_EXISTING;
}
UINT8 MC_OD_Write(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub)
        return ABORTIDX_READ_ONLY_ENTRY;
    const mc_field_t *field = NULL;
    for (unsigned i = 0; i < mc_field_count; i++)
        if (mc_fields[i].index == index && mc_fields[i].sub == sub) {
            field = &mc_fields[i];
            break;
        }
    if (!field)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (!(field->access & 2))
        return ABORTIDX_READ_ONLY_ENTRY;
    if (bytes != field->bytes)
        return ABORTIDX_PARAM_LENGTH_ERROR;
    bool command = index >= 0x2200 && index <= 0x23B0, owner = index == 0x2003;
    if (!command && !owner && !preop())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (owner && !owner_change_allowed())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (index >= 0x3000 && index <= 0x31B8 && transaction_busy((index - 0x3000) / 16))
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    memcpy(o->data + field->offset, data, bytes);
    uint32_t now = ethercat_canfd_now_ms();
    UINT8 result = 0;
    if (index == 0x2003) {
        mc_owner_t *v = object<mc_owner_t>(index);
        if (v->RequestedOwner > 2) {
            result = ABORTIDX_VALUE_EXCEEDED;
            v->Result = RESULT_INVALID;
        } else {
            owner_changed();
            v->Owner = v->RequestedOwner;
            v->Result = 0;
        }
    } else if (index == 0x2002) {
        mc_action_t *v = object<mc_action_t>(index);
        if (sub == 1) {
            v->Result = RESULT_BUSY;
            if (apply_configuration()) {
                v->AppliedGeneration = v->ApplyGeneration;
                v->Result = 0;
                object<mc_board_info_t>(0x2000)->ConfigGeneration = v->AppliedGeneration;
            } else
                v->Result = RESULT_INVALID;
        }
        if (sub == 2)
            v->Result = RESULT_UNSUPPORTED;
    } else if (command && sub == 11) {
        unsigned a = (index - 0x2200) / 16;
        mc_sdo_command_t *v = object<mc_sdo_command_t>(index);
        if (v->CommitSeq != v->AcceptedSeq)
            commit_sdo_command(a, *v, now);
    } else if (index >= 0x3000) {
        unsigned a = (index - 0x3000) / 16, part = (index - 0x3000) % 16;
        if ((part == 7 && sub == 3) || (part == 8 && sub == 4))
            result = commit_parameter(a, part == 8, now, preop()) ? 0 : ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    }
    return result;
}
