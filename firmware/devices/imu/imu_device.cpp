#include "imu_device.hpp"

#include <cmath>

namespace dcu::imu {

bool ImuDevice::valid(const Sample &sample, uint32_t supported) noexcept
{
    constexpr uint32_t known = Acceleration | AngularVelocity | Orientation | Temperature;
    if (!sample.valid_fields || (sample.valid_fields & ~(supported & known)))
        return false;
    if (static_cast<uint8_t>(sample.timestamp_domain) > static_cast<uint8_t>(TimestampDomain::EthercatDc))
        return false;
    for (unsigned i = 0; i < 3; ++i) {
        if ((sample.valid_fields & Acceleration) && !std::isfinite(sample.acceleration_m_s2[i]))
            return false;
        if ((sample.valid_fields & AngularVelocity) && !std::isfinite(sample.angular_velocity_rad_s[i]))
            return false;
    }
    if ((sample.valid_fields & Temperature) && !std::isfinite(sample.temperature_C))
        return false;
    if (sample.valid_fields & Orientation) {
        float norm = 0;
        for (float value : sample.orientation_wxyz) {
            if (!std::isfinite(value))
                return false;
            norm += value * value;
        }
        if (!std::isfinite(norm) || std::fabs(norm - 1.0F) > 0.01F)
            return false;
    }
    return true;
}

Result ImuDevice::poll(uint64_t now_ns) noexcept
{
    if (!maximum_age_ns_ || (polled_ && now_ns < last_poll_ns_)) {
        snapshot_.status = maximum_age_ns_ ? Status::ClockError : Status::InvalidData;
        snapshot_.sample.valid_fields = 0;
        return Result::InvalidData;
    }
    last_poll_ns_ = now_ns;
    polled_ = true;
    if (!initialized_) {
        const auto result = driver_.initialize();
        if (result != Result::Ok) {
            snapshot_.sample.valid_fields = 0;
            snapshot_.status = result == Result::Unavailable ? Status::Unavailable :
                result == Result::NotReady ? Status::Starting : Status::Error;
            return result;
        }
        initialized_ = true;
        snapshot_.status = Status::Starting;
    }
    Sample candidate{};
    const auto result = driver_.read(candidate);
    if (result == Result::NoData || result == Result::NotReady)
        return result;
    if (result != Result::Ok || !valid(candidate, driver_.supported_fields())) {
        if (result == Result::Unavailable || result == Result::IoError)
            initialized_ = false;
        snapshot_.sample.valid_fields = 0;
        snapshot_.status = result == Result::Unavailable ? Status::Unavailable :
            result == Result::Ok || result == Result::InvalidData ? Status::InvalidData : Status::Error;
        return result == Result::Ok ? Result::InvalidData : result;
    }
    snapshot_.sample = candidate;
    snapshot_.status = Status::Ready;
    snapshot_.received_monotonic_ns = now_ns;
    ++snapshot_.sample_sequence;
    return Result::Ok;
}

Snapshot ImuDevice::snapshot(uint64_t now_ns) const noexcept
{
    Snapshot result = snapshot_;
    if (result.status == Status::Ready) {
        if (now_ns < last_poll_ns_ || now_ns < result.received_monotonic_ns) {
            result.status = Status::ClockError;
            result.sample.valid_fields = 0;
        } else if (now_ns - result.received_monotonic_ns >= maximum_age_ns_) {
            result.status = Status::Stale;
            result.sample.valid_fields = 0;
        }
    }
    return result;
}

}
