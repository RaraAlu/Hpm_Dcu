#ifndef DCU_IMU_DEVICE_HPP
#define DCU_IMU_DEVICE_HPP

#include <cstdint>

namespace dcu::imu {

enum Field : uint32_t {
    Acceleration = 1U << 0, AngularVelocity = 1U << 1,
    Orientation = 1U << 2, Temperature = 1U << 3
};
enum class Result : uint8_t { Ok, NoData, NotReady, Unavailable, IoError, InvalidData };
enum class Status : uint8_t { Unavailable, Starting, Ready, Stale, Error, InvalidData, ClockError };
enum class TimestampDomain : uint8_t { Unknown, Sensor, LocalMonotonic, EthercatDc };

struct Sample {
    uint32_t valid_fields{};
    uint64_t capture_timestamp_ns{};
    TimestampDomain timestamp_domain{};
    float acceleration_m_s2[3]{};
    float angular_velocity_rad_s[3]{};
    float orientation_wxyz[4]{};
    float temperature_C{};
};

struct Snapshot {
    Status status{Status::Unavailable};
    uint32_t sample_sequence{};
    uint64_t received_monotonic_ns{};
    Sample sample{};
};

class ImuDriver {
public:
    virtual uint32_t supported_fields() const noexcept = 0;
    // Both operations must be bounded and nonblocking. NotReady permits retry.
    virtual Result initialize() noexcept = 0;
    virtual Result read(Sample &) noexcept = 0;

protected:
    ~ImuDriver() = default;
};

class UnavailableImuDriver final : public ImuDriver {
public:
    uint32_t supported_fields() const noexcept override { return 0; }
    Result initialize() noexcept override { return Result::Unavailable; }
    Result read(Sample &) noexcept override { return Result::Unavailable; }
};

class ImuDevice {
public:
    // The driver must outlive the device. Time is local monotonic ns, not sensor time.
    ImuDevice(ImuDriver &driver, uint64_t maximum_age_ns) noexcept
        : driver_(driver), maximum_age_ns_(maximum_age_ns) {}
    Result poll(uint64_t now_ns) noexcept;
    Snapshot snapshot(uint64_t now_ns) const noexcept;

private:
    static bool valid(const Sample &, uint32_t supported) noexcept;
    ImuDriver &driver_;
    uint64_t maximum_age_ns_;
    Snapshot snapshot_{};
    uint64_t last_poll_ns_{};
    bool polled_{};
    bool initialized_{};
};

}
#endif
