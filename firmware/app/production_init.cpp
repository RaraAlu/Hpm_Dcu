#include "production_init.h"
#include "gateway_application.hpp"
#include "motor_adapter.hpp"
#include "production_profile.h"

namespace gateway = dcu::app::gateway_application;

bool motor_control_init(void)
{
    const auto &profile = *dcu_production_profile();
    return gateway::initialize_profile(profile) && dcu::ethercat::motor_adapter::initialize(profile) &&
           gateway::select_mode(gateway::OperatingMode::MotorWithRaw);
}

bool motor_control_boot_startup(uint64_t boot_id)
{
    return gateway::boot_startup(dcu_production_profile()->readiness, boot_id);
}
