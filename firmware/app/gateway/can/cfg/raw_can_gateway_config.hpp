#ifndef DCU_RAW_CAN_GATEWAY_CONFIG_HPP
#define DCU_RAW_CAN_GATEWAY_CONFIG_HPP

#include <cstdint>

namespace dcu::app::config::raw_can {

inline constexpr unsigned bus_capacity = 4;
inline constexpr unsigned protected_id_capacity = 8;
inline constexpr unsigned batch_capacity = 7;
inline constexpr unsigned single_fd_capacity = 1;
inline constexpr unsigned queue_capacity = 32;
inline constexpr uint32_t rate_window_ms = 2;
// 单次透传会话最多容许八帧丢弃；第九帧闭锁，不影响电机指令。
inline constexpr uint32_t drop_limit = 8;

static_assert(queue_capacity > 1 && queue_capacity <= 256 &&
              (queue_capacity & (queue_capacity - 1)) == 0);

}
#endif
