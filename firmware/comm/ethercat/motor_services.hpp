#ifndef DCU_MOTOR_SERVICES_HPP
#define DCU_MOTOR_SERVICES_HPP

#include <cstdint>

namespace dcu::ethercat::motor_services {
void initialize();
bool commit(unsigned axis, bool maintenance, uint32_t now_ms, bool preop);
}
#endif
