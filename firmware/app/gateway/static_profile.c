#include "static_profile.h"
#include "dcu_static_profile_generated.h"

static const dcu_static_profile_t s_profile = {
    .bus = DCU_PROFILE_CAN_BUS,
    .nominal_bitrate_bps = DCU_PROFILE_CAN_BITRATE_KBPS * 1000U,
    .frame_timeout_ms = DCU_PROFILE_FRAME_TIMEOUT_MS,
    .device_id = DCU_PROFILE_DEVICE_ID,
};

const dcu_static_profile_t *dcu_static_profile(void)
{
    return &s_profile;
}

bool dcu_static_profile_can_id(unsigned bus, unsigned frame, uint16_t *id)
{
    if (!id || bus != s_profile.bus || frame >= DCU_PROFILE_CAN_FRAMES) return false;
    *id = DCU_PROFILE_CAN_ID_BASE + frame;
    return true;
}
