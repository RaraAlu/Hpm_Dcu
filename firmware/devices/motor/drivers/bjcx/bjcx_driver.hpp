#ifndef DCU_BJCX_DRIVER_HPP
#define DCU_BJCX_DRIVER_HPP

#include "motor_session_driver.hpp"

namespace dcu::motor {

class BjcxMotorDriver final : public MotorSessionDriver {
public:
    static bool validate(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms, bool neutral_mit) noexcept;
    bool configure(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms, bool neutral_mit) noexcept;
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
    bool prepare_status_poll(uint32_t now_ms, StatusPoll &) const override;
    void status_poll_submitted(uint32_t now_ms) override;
    bool neutral_mit_only() const override { return neutral_mit_; }
    bool encode_neutral_mit(CanFrame &, uint8_t counter) const override;
    uint8_t max_rx_bytes() const override;
    bool observe_status_reply(const CanFrame &, uint32_t stamp_ms, Feedback &, int32_t &, int32_t &) override;

private:
    Capabilities capabilities_{};
    uint32_t requested_ms_{}, received_ms_{}, valid_fields_{};
    uint16_t can_id_{}, period_ms_{}, timeout_ms_{};
    bool requested_{}, received_{}, neutral_mit_{};
};

}
#endif
