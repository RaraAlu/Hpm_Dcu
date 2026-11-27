#ifndef DCU_MOTOR_SESSION_DRIVER_HPP
#define DCU_MOTOR_SESSION_DRIVER_HPP

#include "motor_driver.hpp"

namespace dcu::motor {

struct QuerySample {
    uint32_t valid_fields{};
    uint32_t position_age_us{UINT32_MAX}, velocity_age_us{UINT32_MAX};
    uint16_t feedback_age_ms{UINT16_MAX}, power_age_ms{UINT16_MAX};
    int32_t power_mW{};
    bool online{}, has_motion_data{}, power_valid{}, new_power{};
};

enum class ObservationResult : uint8_t { Invalid, Acknowledged, Valid };

struct StatusPoll {
    CanFrame frame{};
    uint16_t timeout_ms{};
    uint8_t reply_bytes{};
};

// One driver per configured device. The module owns permission, deadlines,
// and TX cancellation; the driver owns protocol readiness and query samples.
class MotorSessionDriver : public MotorDriver {
public:
    virtual bool query_ready(uint32_t now_ms) const = 0;
    virtual bool prepare_start(const Command &, uint32_t now_ms, Command &first) = 0;
    virtual bool start_query(uint32_t now_ms, bool active) = 0;
    virtual ObservationResult observe(const CanFrame &, uint32_t stamp_ms, uint32_t now_ms,
                                      bool active, Feedback &, int32_t &position, int32_t &velocity) = 0;
    virtual void observe_cyclic(uint32_t stamp_ms, bool first) = 0;
    virtual QuerySample query_sample(uint32_t now_ms) const = 0;
    virtual void acknowledge_queries() = 0;
    // Preparing a frame does not consume the poll period; a successful TX submission does.
    virtual bool prepare_status_poll(uint32_t, StatusPoll &) const { return false; }
    virtual void status_poll_submitted(uint32_t) {}
    virtual bool neutral_mit_only() const { return false; }
    virtual bool encode_neutral_mit(CanFrame &, uint8_t) const { return false; }
    virtual uint8_t max_rx_bytes() const { return DCU_CAN_CLASSIC_MAX_PAYLOAD; }
    virtual bool observe_status_reply(const CanFrame &, uint32_t, Feedback &, int32_t &, int32_t &) { return false; }
};

}
#endif
