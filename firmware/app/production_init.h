#ifndef DCU_PRODUCTION_INIT_H
#define DCU_PRODUCTION_INIT_H

#include "production_profile.h"
#include "gateway_flow.h"

#ifdef __cplusplus
extern "C" {
#endif

bool dcu_production_boot(dcu_pdo_mode_t boot_mode);
/* Called by dcu_production_boot after CAN has applied the active Profile. */
bool dcu_gateway_profile_init(const dcu_production_profile_t *profile);
bool dcu_pdo_layout_init(const dcu_production_profile_t *profile, dcu_pdo_mode_t boot_mode);
bool dcu_gateway_readiness_boot(const dcu_gateway_profile_t *profile, uint64_t boot_id);

#ifdef __cplusplus
}
#endif
#endif
