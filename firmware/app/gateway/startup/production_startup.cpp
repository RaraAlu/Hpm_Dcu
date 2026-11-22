#include "detail/production_startup.hpp"

namespace dcu::app::gateway_application::detail {

uint64_t ProductionStartup::time_us(uint32_t now)
{
    time_us_ += uint64_t{now - clock_ms_} * 1000U;
    clock_ms_ = now;
    return time_us_;
}

bool ProductionStartup::boot(const dcu_gateway_profile_t &profile, uint64_t boot_id, uint32_t now)
{
    if (boot_called_) return false;
    boot_called_ = true;
    clock_ms_ = now;
    ttl_ms_ = profile.readiness_ttl_ms;
    return dcu_gateway_boot_static(&profile, boot_id, time_us_);
}

bool ProductionStartup::enforced() const
{
#if defined(DCU_HOST_ENGINE_TEST)
    // Native engine fixtures can bypass the outer protocol; firmware cannot.
    return boot_called_;
#else
    return true;
#endif
}

bool ProductionStartup::apply_policy(uint32_t profile, uint32_t base, uint32_t revision,
                                     uint32_t ttl_ms, uint32_t now)
{
    if (!boot_called_) return false;
    const auto stamp = time_us(now);
    if (!dcu_gateway_apply_policy(profile, base, revision, ttl_ms, stamp)) return false;
    ttl_ms_ = ttl_ms;
    verified_ = 0;
    preparation_us_ = stamp;
    return true;
}

dcu_gateway_readiness_t ProductionStartup::status() const { return dcu_gateway_readiness(time_us_); }

bool ProductionStartup::running() const
{
    return !enforced() || status().state == DCU_GATEWAY_RUNNING;
}

bool ProductionStartup::may_probe() const
{
    const auto state = status().state;
    return enforced() && (state == DCU_GATEWAY_PREPARING || state == DCU_GATEWAY_READY);
}

void ProductionStartup::invalidate(dcu_gateway_reason_t reason, uint32_t now)
{
    if (!boot_called_) return;
    dcu_gateway_link_lost(reason, time_us(now));
    verified_ = 0;
}

void ProductionStartup::observe(unsigned device, uint32_t stamp, uint32_t now)
{
    if (!boot_called_ || device >= DCU_MOTOR_DEVICE_CAPACITY) return;
    if (!may_probe() && !running()) return;
    const auto t = time_us(now);
    const uint64_t age_us = uint64_t{now - stamp} * 1000U;
    if (age_us > t || t - age_us < preparation_us_) return;
    stamps_ms_[device] = stamp;
    verified_ |= 1U << device;
}

void ProductionStartup::step(uint32_t now, uint32_t expected_mask, bool healthy)
{
    if (!boot_called_) return;
    const auto t = time_us(now);
    auto state = status().state;
    if (!healthy) {
        invalidate(DCU_GATEWAY_REASON_CAN_OFFLINE, now);
        return;
    }
    if (state == DCU_GATEWAY_HOLD) {
        verified_ = 0;
        preparation_us_ = t;
        if (!dcu_gateway_retry_preparation(t)) return;
        state = DCU_GATEWAY_PREPARING;
    }
    if (state != DCU_GATEWAY_PREPARING && state != DCU_GATEWAY_READY && state != DCU_GATEWAY_RUNNING)
        return;
    uint32_t oldest = 0;
    for (unsigned i = 0; expected_mask; ++i, expected_mask >>= 1) {
        if (!(expected_mask & 1U)) continue;
        const uint32_t age = now - stamps_ms_[i];
        if (!(verified_ & (1U << i)) || age >= ttl_ms_) return;
        if (age > oldest) oldest = age;
    }
    // With an empty device list this only certifies the local CAN configuration.
    (void)dcu_gateway_probe_verified(t - uint64_t{oldest} * 1000U, t);
}

}
