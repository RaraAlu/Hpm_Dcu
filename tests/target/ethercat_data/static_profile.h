#ifndef DCU_GATEWAY_STATIC_PROFILE_H
#define DCU_GATEWAY_STATIC_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    unsigned bus;
    uint32_t nominal_bitrate_bps;
    uint32_t frame_timeout_ms;
    const char *device_id;
} dcu_static_profile_t;

const dcu_static_profile_t *dcu_static_profile(void);
bool dcu_static_profile_can_id(unsigned bus, unsigned frame, uint16_t *id);

#endif
