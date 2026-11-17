#ifndef ENCOS_MOTOR_H
#define ENCOS_MOTOR_H
#include <stdbool.h>
#include <stdint.h>
#include "encos_service.h"
/* Physical encoding ranges: rad, rad/s, Nm, A and Nm/A. */
typedef struct { double kp[2], kd[2], pos[2], vel[2], torque[2], current[2], kt; } encos_profile_t;
typedef struct {
    encos_profile_t profile;
    uint16_t timeout_ms;
    uint8_t frame_format;
} encos_settings_t;
#ifdef __cplusplus
extern "C" {
#endif

bool encos_profile_valid(const encos_profile_t *);
#ifdef __cplusplus
}
#endif
#endif
