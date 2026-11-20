#ifndef DCU_PRODUCTION_STARTUP_HPP
#define DCU_PRODUCTION_STARTUP_HPP

#include "gateway_readiness.h"
#include "device_config.h"

namespace dcu::app::gateway_application::detail {

class ProductionStartup {
public:
    bool boot(const dcu_gateway_profile_t &, uint64_t boot_id, uint32_t now_ms);
    bool enforced() const;
    void step(uint32_t now_ms, unsigned expected_devices, bool buses_healthy);
    void observe(unsigned device, uint32_t stamp_ms, uint32_t now_ms);
    void invalidate(dcu_gateway_reason_t reason, uint32_t now_ms);
    bool resume() const { return dcu_gateway_resume_static(time_us_); }
    bool apply_policy(uint32_t profile, uint32_t base, uint32_t revision, uint32_t ttl_ms, uint32_t now_ms);
    uint32_t readiness_ttl_ms() const { return ttl_ms_; }
    dcu_gateway_readiness_t status() const;
    bool may_probe() const;
    bool running() const;

private:
    uint64_t time_us(uint32_t now_ms);
    uint64_t time_us_{}, preparation_us_{};
    uint32_t clock_ms_{}, ttl_ms_{};
    uint32_t verified_{};
    uint32_t stamps_ms_[DCU_MOTOR_DEVICE_CAPACITY]{};
    bool boot_called_{};
};

}
#endif
