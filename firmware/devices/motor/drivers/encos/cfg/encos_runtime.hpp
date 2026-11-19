#ifndef DCU_ENCOS_RUNTIME_HPP
#define DCU_ENCOS_RUNTIME_HPP

#include <array>
#include <cstdint>

namespace dcu::motor::encos_runtime {
inline constexpr uint32_t query_online_ms = 300;
inline constexpr uint32_t query_position_ms = 100;
inline constexpr uint32_t query_velocity_ms = 150;
inline constexpr uint32_t query_current_ms = 300;
inline constexpr uint32_t parameter_timeout_ms = 100;
inline constexpr uint32_t parameter_hold_ms = 500;
inline constexpr std::array<uint16_t, 3> query_period_ms{100, 100, 200};
}
#endif
