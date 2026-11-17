#ifndef DCU_MOTOR_ADAPTER_STATE_HPP
#define DCU_MOTOR_ADAPTER_STATE_HPP

#include "motor_application.hpp"
#include "motor_pdo_layout.hpp"

namespace dcu::ethercat::motor_adapter::detail {
struct InputState {
    uint8_t wire[pdo_layout::max_output_bytes]{};
    app::motor_application::CycleCommand command{};
    app::motor_application::OperatingMode mode{};
    bool pending{};
};
}
#endif
