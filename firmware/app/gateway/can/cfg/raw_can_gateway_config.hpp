#ifndef DCU_RAW_CAN_GATEWAY_CONFIG_HPP
#define DCU_RAW_CAN_GATEWAY_CONFIG_HPP

#include <cstdint>

namespace dcu::app::config::raw_can {

inline constexpr unsigned bus_capacity = 4;
inline constexpr unsigned protected_id_capacity = 8;
inline constexpr unsigned batch_capacity = 7;
inline constexpr unsigned queue_capacity = 32;
inline constexpr uint32_t rate_window_ms = 2;

static_assert(queue_capacity > 1 && queue_capacity <= 256 &&
              (queue_capacity & (queue_capacity - 1)) == 0);

}
#endif
