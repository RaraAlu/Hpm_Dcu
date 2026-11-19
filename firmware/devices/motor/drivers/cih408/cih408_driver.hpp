#ifndef DCU_CIH408_DRIVER_HPP
#define DCU_CIH408_DRIVER_HPP

#include "motor_session_driver.hpp"

namespace dcu::motor {

class Cih408MotorDriver final : public MotorSessionDriver {
public:
    static bool validate(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms) noexcept;
    bool configure(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms) noexcept;
    const Capabilities &capabilities() const noexcept override { return capabilities_; }
    Result encode(const Command &, CanFrame &) const noexcept override { return Result::Unsupported; }
    Result decode(const CanFrame &, Feedback &) const noexcept override;
    bool query_ready(uint32_t) const override { return false; }
    bool prepare_start(const Command &, uint32_t, Command &) override { return false; }
    bool start_query(uint32_t, bool) override { return false; }
    ObservationResult observe(const CanFrame &, uint32_t, uint32_t, bool, Feedback &, int32_t &, int32_t &) override
    { return ObservationResult::Invalid; }
    void observe_cyclic(uint32_t, bool) override {}
    QuerySample query_sample(uint32_t now_ms) const override;
    void acknowledge_queries() override {}
    bool prepare_status_poll(uint32_t now_ms, StatusPoll &) override;
    uint8_t max_rx_bytes() const override { return 16; }
    bool observe_status_reply(const CanFrame &, uint32_t stamp_ms, Feedback &, int32_t &, int32_t &) override;

private:
    Capabilities capabilities_{};
    uint32_t requested_ms_{}, received_ms_{}, valid_fields_{};
    uint16_t can_id_{}, period_ms_{}, timeout_ms_{};
    bool requested_{}, received_{};
};

}
#endif
