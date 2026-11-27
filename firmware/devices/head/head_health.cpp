#include "head_health.hpp"
#include <dcu_profile_pdo.hpp>

namespace dcu::head {
namespace {
Monitor monitor;
}

const Health &health(uint32_t now)
{
    return monitor.sample(now, dcu_profile_pdo::head_heartbeat_timeout_ms, dcu_profile_pdo::head_feedback_periods);
}

const CyclicCanObserver *profile_observer()
{
    if constexpr (!dcu_profile_pdo::head_heartbeat_timeout_ms || dcu_profile_pdo::sideband_bus < 0) return nullptr;
    static constexpr CyclicCanObserver observer{
        static_cast<unsigned>(dcu_profile_pdo::sideband_bus),
        [](uint32_t now) { monitor.start(now); }, [] { monitor.stop(); },
        [](const dcu_can_frame_t &f, uint32_t at) { monitor.receive(f, at); },
        [](uint32_t now) -> uint16_t {
            const auto &h = health(now);
            return h.monitoring ? ((h.offline ? 0x20 : 0) | (h.stale ? 0x40 : 0) | (h.device_errors ? 0x80 : 0)) : 0;
        }
    };
    return &observer;
}
}
