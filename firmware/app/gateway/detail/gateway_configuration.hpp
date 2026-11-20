#ifndef DCU_GATEWAY_CONFIGURATION_HPP
#define DCU_GATEWAY_CONFIGURATION_HPP

#include "gateway_application.hpp"

namespace dcu::app::gateway_application::detail {

struct BoardConfiguration {
    uint16_t max_command_lease_ms{}, feedback_timeout_ms{};
};

// The device list is borrowed from the Profile for validation and binding only.
struct Configuration {
    BoardConfiguration board{};
    ChannelConfiguration channels[DCU_DEVICE_CAN_BUS_COUNT]{};
    const dcu_motor_device_profile_t *devices{};
    size_t device_count{};
};

#if defined(DCU_HOST_ENGINE_TEST)
// Native fixtures can exercise Raw roles that the production Profile does not grant.
bool initialize_fixture(const Configuration &config);
#endif

}
#endif
