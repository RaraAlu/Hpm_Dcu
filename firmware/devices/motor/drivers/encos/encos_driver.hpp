#ifndef DCU_ENCOS_DRIVER_HPP
#define DCU_ENCOS_DRIVER_HPP

#include "motor_driver.hpp"
#include "encos_motor.h"

namespace dcu::motor {

class EncosMotorDriver final : public MotorDriver {
public:
    static bool validate(const Routing &, const encos_settings_t &,
                         uint8_t frame_format) noexcept;
    bool configure(const Routing &, const encos_settings_t &,
                   uint8_t frame_format) noexcept;
    const Capabilities &capabilities() const noexcept override { return capabilities_; }
    Result encode(const Command &, CanFrame &) const noexcept override;
    Result decode(const CanFrame &, Feedback &) const noexcept override;

private:
    Routing route_{};
    encos_profile_t profile_{};
    Capabilities capabilities_{};
    bool configured_{};
};

}
#endif
