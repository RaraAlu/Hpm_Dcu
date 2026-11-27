#include "production_init.h"
#include "head_health.hpp"
#include "gateway_application.hpp"
#include "motor_adapter.hpp"

namespace gateway = dcu::app::gateway_application;

bool dcu_motor_axes_init(const dcu_production_profile_t *profile)
{
    return gateway::initialize_profile(*profile, dcu::head::profile_observer());
}

bool dcu_pdo_layout_init(const dcu_production_profile_t *profile, dcu_pdo_mode_t boot_mode)
{
    return static_cast<unsigned>(boot_mode) <= DCU_PDO_SINGLE_FD &&
           dcu::ethercat::motor_adapter::initialize(*profile) &&
           gateway::select_mode(static_cast<gateway::OperatingMode>(boot_mode));
}

bool dcu_gateway_readiness_boot(const dcu_gateway_profile_t *profile, uint64_t boot_id)
{
    return gateway::boot_startup(*profile, boot_id);
}
