#ifndef DCU_ENCOS_DRIVER_HPP
#define DCU_ENCOS_DRIVER_HPP

#include "motor_session_driver.hpp"
#include "detail/encos_session_state.hpp"
#include "encos_motor.h"

namespace dcu::motor {

class EncosMotorDriver final : public MotorSessionDriver {
public:
    static bool validate(const Routing &, const encos_settings_t &,
                         uint8_t frame_format) noexcept;
    bool configure(const Routing &, const encos_settings_t &,
                   uint8_t frame_format, bool zero_mit_loopback = false) noexcept;
    bool zero_mit_loopback() const override { return zero_mit_loopback_; }
    const Capabilities &capabilities() const noexcept override { return capabilities_; }
    Result encode(const Command &, CanFrame &) const noexcept override;
    Result decode(const CanFrame &, Feedback &) const noexcept override;
    void bind_session(unsigned device, unsigned bus, uint8_t format, uint16_t power_period_ms) noexcept;
    bool query_ready(uint32_t now_ms) const override;
    bool prepare_start(const Command &, uint32_t now_ms, Command &first) override;
    bool start_query(uint32_t now_ms, bool active) override;
    ObservationResult observe(const CanFrame &, uint32_t stamp_ms, uint32_t now_ms,
                              bool active, Feedback &, int32_t &position, int32_t &velocity) override;
    void observe_cyclic(uint32_t stamp_ms, bool first) override;
    QuerySample query_sample(uint32_t now_ms) const override;
    void acknowledge_queries() override;

private:
    Routing route_{};
    encos_profile_t profile_{};
    Capabilities capabilities_{};
    detail::EncosSessionState queries_{};
    bool configured_{};
    bool zero_mit_loopback_{};
};

}
#endif
