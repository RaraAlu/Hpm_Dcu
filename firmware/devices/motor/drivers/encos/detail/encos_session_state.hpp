#ifndef DCU_ENCOS_SESSION_STATE_HPP
#define DCU_ENCOS_SESSION_STATE_HPP

#include <cstdint>

namespace dcu::motor::detail {

struct EncosSessionState {
    unsigned device{}, bus{};
    uint16_t power_period_ms{};
    uint8_t frame_format{};
    uint32_t online_ms{}, position_ms{}, velocity_ms{}, current_ms{}, power_ms{};
    uint32_t request_ms[4]{};
    uint8_t requested_mask{};
    int32_t position{}, velocity{}, power_mW{};
    bool online_seen{}, position_seen{}, velocity_seen{}, current_seen{}, power_seen{}, new_power{};
};

}
#endif
