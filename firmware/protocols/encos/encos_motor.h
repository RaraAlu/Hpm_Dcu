#ifndef ENCOS_MOTOR_H
#define ENCOS_MOTOR_H
#include <stdbool.h>
#include "encos_interface.h"
typedef struct { uint32_t id; uint8_t dlc, flags, data[8]; } encos_frame_t;
/* Explicitly confirmed custom profile (0x8000). No guessed installed motor model. */
typedef struct { double kp[2], kd[2], pos[2], vel[2], torque[2], current[2], kt; } encos_profile_t;
#ifdef __cplusplus
extern "C" {
#endif

bool encos_profile(const mc_range_t *, const mc_basic_t *, encos_profile_t *);
bool encos_encode(const mc_axis_rx_t *, const mc_routing_t *, const encos_profile_t *, encos_frame_t *);
bool encos_feedback(const encos_frame_t *, const encos_profile_t *, mc_axis_tx_t *);
bool encos_query(uint8_t code, uint8_t group, encos_frame_t *);
bool encos_parameter(uint16_t parameter, const mc_basic_t *, const mc_range_t *, const mc_gain_t *, const mc_kt_t *, encos_frame_t *);
uint8_t encos_parameter_query(uint16_t parameter);
uint16_t encos_be16(const uint8_t *);
uint32_t encos_be32(const uint8_t *);
bool encos_float(const uint8_t *, double scale, int32_t *);
#ifdef __cplusplus
}
#endif
#endif
