#ifndef DCU_READINESS_PROFILE_H
#define DCU_READINESS_PROFILE_H

#include <stdint.h>

enum { DCU_GATEWAY_DIGEST_BYTES = 32 };

typedef struct {
    uint32_t profile_id;
    uint32_t revision;
    uint8_t digest[DCU_GATEWAY_DIGEST_BYTES];
    uint32_t readiness_ttl_ms;
} dcu_gateway_profile_t;

#endif
