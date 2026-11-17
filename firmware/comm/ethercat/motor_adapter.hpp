#ifndef DCU_MOTOR_ADAPTER_HPP
#define DCU_MOTOR_ADAPTER_HPP

#include <cstdint>

namespace dcu::ethercat::motor_adapter {
bool apply_configuration();
void refresh_axis_feedback(unsigned axis);
void commit_sdo_command(unsigned axis, uint32_t now_ms);
}
#endif
