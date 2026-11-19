#ifndef DCU_PRODUCTION_PROFILE_H
#define DCU_PRODUCTION_PROFILE_H

#include <stddef.h>
#include "can_transport.h"
#include "encos_motor.h"
#include "gateway_readiness.h"

typedef struct {
    const char *id;
    unsigned bus;
    uint16_t can_id, model_profile_id, power_query_period_ms;
    int32_t position_min_urad, position_max_urad;
    uint32_t velocity_limit_urad_s, torque_limit_mNm, current_limit_dA;
    encos_settings_t encos;
    uint16_t status_poll_period_ms, status_reply_timeout_ms;
} dcu_motor_device_profile_t;

typedef struct {
    const dcu_can_channel_config_t *channels;
    size_t channel_count;
    const dcu_motor_device_profile_t *devices;
    size_t device_count;
    uint16_t max_command_lease_ms, feedback_timeout_ms;
    dcu_gateway_profile_t readiness;
} dcu_production_profile_t;

#ifdef __cplusplus
extern "C" {
#endif
const dcu_production_profile_t *dcu_production_profile(void);
#ifdef __cplusplus
}
#endif
#endif
