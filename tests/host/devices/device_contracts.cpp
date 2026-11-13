#include "motor_driver.hpp"
#include "imu_device.hpp"

#include <cstdio>
#include <cstring>
#include <limits>

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); return false; \
} } while (false)

namespace motor = dcu::motor;
namespace imu = dcu::imu;

class TestMotor final : public motor::MotorDriver {
public:
    motor::Capabilities caps{motor::mode_bit(motor::ControlMode::Torque), motor::Position, 0, false};
    mutable unsigned encode_calls{};
    motor::Result encode_result{motor::Result::Ok};
    uint8_t dlc{2};
    uint8_t flags{};
    uint32_t id{0x456};
    uint32_t feedback_fields{motor::Position};
    const motor::Capabilities &capabilities() const noexcept override { return caps; }
    motor::Result encode(const motor::Command &, motor::CanFrame &frame) const noexcept override
    {
        ++encode_calls;
        frame.id = id;
        frame.dlc = dlc;
        frame.flags = flags;
        frame.data[0] = 0xAA;
        frame.data[1] = 0x55;
        return encode_result;
    }
    motor::Result decode(const motor::CanFrame &, motor::Feedback &feedback) const noexcept override
    {
        feedback.valid_fields = feedback_fields;
        feedback.position_urad = 12345;
        return motor::Result::Ok;
    }
};

class TestImu final : public imu::ImuDriver {
public:
    uint32_t fields{imu::Acceleration | imu::AngularVelocity | imu::Orientation | imu::Temperature};
    imu::Result init_result{imu::Result::Ok};
    imu::Result read_result{imu::Result::Ok};
    imu::Sample value{};
    unsigned init_calls{}, read_calls{};
    TestImu()
    {
        value.valid_fields = imu::Acceleration;
        value.acceleration_m_s2[2] = 9.81F;
        value.timestamp_domain = imu::TimestampDomain::Sensor;
        value.capture_timestamp_ns = UINT64_C(9000000000);
    }
    uint32_t supported_fields() const noexcept override { return fields; }
    imu::Result initialize() noexcept override { ++init_calls; return init_result; }
    imu::Result read(imu::Sample &sample) noexcept override
    {
        ++read_calls;
        sample = value;
        return read_result;
    }
};

static bool motor_unbound_and_capabilities()
{
    motor::MotorDevice device;
    motor::Command command{};
    command.mode = motor::ControlMode::Position;
    motor::CanFrame frame{};
    frame.id = 999;
    motor::Feedback feedback{};
    feedback.position_urad = 999;
    CHECK(device.encode(command, frame) == motor::Result::NotConfigured && frame.id == 999);
    CHECK(device.decode(frame, feedback) == motor::Result::NotConfigured && feedback.position_urad == 999);
    CHECK(!device.bound() && device.capabilities().control_modes == 0);
    TestMotor driver;
    device.bind(driver);
    CHECK(device.encode(command, frame) == motor::Result::Unsupported && driver.encode_calls == 0);
    command.mode = static_cast<motor::ControlMode>(255);
    CHECK(device.encode(command, frame) == motor::Result::Unsupported && driver.encode_calls == 0);
    device.reset();
    CHECK(!device.bound() && !device.capabilities().active_stop_command);
    return true;
}

static bool motor_substitution()
{
    TestMotor driver;
    motor::MotorDevice device;
    device.bind(driver);
    motor::Command command{};
    command.mode = motor::ControlMode::Torque;
    motor::CanFrame frame{};
    CHECK(device.encode(command, frame) == motor::Result::Ok);
    CHECK(frame.id == 0x456 && frame.dlc == 2 && frame.data[0] == 0xAA);
    motor::Feedback feedback{};
    CHECK(device.decode(frame, feedback) == motor::Result::Ok);
    CHECK(feedback.position_urad == 12345 && feedback.valid_fields == motor::Position);
    return true;
}

static bool motor_failed_outputs_are_not_committed()
{
    TestMotor driver;
    motor::MotorDevice device;
    device.bind(driver);
    motor::Command command{};
    command.mode = motor::ControlMode::Torque;
    motor::CanFrame frame{};
    frame.id = 999;
    driver.encode_result = motor::Result::InvalidCommand;
    CHECK(device.encode(command, frame) == motor::Result::InvalidCommand && frame.id == 999);
    driver.encode_result = motor::Result::Ok;
    driver.dlc = 9;
    CHECK(device.encode(command, frame) == motor::Result::InvalidFrame && frame.id == 999);
    driver.dlc = 2;
    driver.id = 0x800;
    CHECK(device.encode(command, frame) == motor::Result::InvalidFrame && frame.id == 999);
    driver.id = 1;
    driver.flags = DCU_CAN_FLAG_BRS;
    CHECK(device.encode(command, frame) == motor::Result::InvalidFrame && frame.id == 999);
    motor::Feedback feedback{};
    feedback.position_urad = 999;
    driver.feedback_fields = motor::Torque;
    CHECK(device.decode(frame, feedback) == motor::Result::InvalidFrame && feedback.position_urad == 999);
    driver.feedback_fields = 64;
    driver.caps.feedback_fields = 64;
    CHECK(device.decode(frame, feedback) == motor::Result::InvalidFrame && feedback.position_urad == 999);
    return true;
}

static bool imu_unavailable()
{
    imu::UnavailableImuDriver driver;
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(0) == imu::Result::Unavailable);
    const auto snapshot = device.snapshot(1000);
    CHECK(snapshot.status == imu::Status::Unavailable && snapshot.sample.valid_fields == 0);
    CHECK(snapshot.sample_sequence == 0 && snapshot.received_monotonic_ns == 0);
    CHECK(snapshot.sample.orientation_wxyz[0] == 0 && driver.supported_fields() == 0);
    return true;
}

static bool imu_startup()
{
    TestImu driver;
    driver.init_result = imu::Result::NotReady;
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(0) == imu::Result::NotReady && driver.read_calls == 0);
    CHECK(device.snapshot(0).status == imu::Status::Starting);
    driver.init_result = imu::Result::Ok;
    driver.read_result = imu::Result::NoData;
    CHECK(device.poll(1) == imu::Result::NoData && driver.init_calls == 2);
    CHECK(device.snapshot(1).status == imu::Status::Starting);
    driver.read_result = imu::Result::Ok;
    CHECK(device.poll(2) == imu::Result::Ok && driver.init_calls == 2);
    CHECK(device.snapshot(2).status == imu::Status::Ready && device.snapshot(2).sample_sequence == 1);
    return true;
}

static bool imu_freshness()
{
    TestImu driver;
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(1000) == imu::Result::Ok);
    driver.read_result = imu::Result::NoData;
    CHECK(device.poll(1099) == imu::Result::NoData);
    CHECK(device.snapshot(1099).status == imu::Status::Ready);
    const auto stale = device.snapshot(1100);
    CHECK(stale.status == imu::Status::Stale && stale.sample.valid_fields == 0);
    CHECK(stale.sample_sequence == 1 && stale.received_monotonic_ns == 1000);
    CHECK(stale.sample.capture_timestamp_ns == UINT64_C(9000000000));
    return true;
}

static bool imu_invalid_samples()
{
    TestImu driver;
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(0) == imu::Result::Ok);
    driver.value.acceleration_m_s2[0] = std::numeric_limits<float>::quiet_NaN();
    CHECK(device.poll(1) == imu::Result::InvalidData && device.snapshot(1).sample.valid_fields == 0);
    CHECK(device.snapshot(1).sample_sequence == 1);
    driver.value.acceleration_m_s2[0] = 0;
    driver.value.valid_fields = imu::Orientation;
    CHECK(device.poll(2) == imu::Result::InvalidData);
    driver.value.orientation_wxyz[0] = 1;
    CHECK(device.poll(3) == imu::Result::Ok);
    driver.fields = imu::Acceleration;
    CHECK(device.poll(4) == imu::Result::InvalidData);
    driver.value.valid_fields = 0;
    CHECK(device.poll(5) == imu::Result::InvalidData);
    driver.fields = imu::Acceleration;
    driver.value.valid_fields = imu::Acceleration;
    driver.value.timestamp_domain = static_cast<imu::TimestampDomain>(255);
    CHECK(device.poll(6) == imu::Result::InvalidData);
    return true;
}

static bool imu_errors()
{
    TestImu driver;
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(10) == imu::Result::Ok);
    driver.read_result = imu::Result::IoError;
    CHECK(device.poll(11) == imu::Result::IoError);
    CHECK(device.snapshot(11).status == imu::Status::Error && device.snapshot(11).sample.valid_fields == 0);
    driver.read_result = imu::Result::Unavailable;
    CHECK(device.poll(12) == imu::Result::Unavailable);
    CHECK(device.snapshot(12).status == imu::Status::Unavailable);
    driver.read_result = imu::Result::Ok;
    CHECK(device.poll(13) == imu::Result::Ok && device.snapshot(13).sample_sequence == 2);
    return true;
}

static bool imu_clock_contract()
{
    TestImu driver;
    imu::ImuDevice invalid(driver, 0);
    CHECK(invalid.poll(0) == imu::Result::InvalidData && driver.init_calls == 0);
    imu::ImuDevice device(driver, 100);
    CHECK(device.poll(1000) == imu::Result::Ok);
    CHECK(device.snapshot(999).status == imu::Status::ClockError);
    CHECK(device.poll(999) == imu::Result::InvalidData && driver.read_calls == 1);
    CHECK(device.snapshot(1001).sample.valid_fields == 0);
    CHECK(device.poll(1002) == imu::Result::Ok && device.snapshot(1002).sample_sequence == 2);
    return true;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    struct Case { const char *name; bool (*run)(); };
    const Case cases[] = {
        {"motor_unbound", motor_unbound_and_capabilities},
        {"motor_substitution", motor_substitution},
        {"motor_transactional", motor_failed_outputs_are_not_committed},
        {"imu_unavailable", imu_unavailable}, {"imu_startup", imu_startup},
        {"imu_freshness", imu_freshness}, {"imu_invalid", imu_invalid_samples},
        {"imu_errors", imu_errors}, {"imu_clock", imu_clock_contract},
    };
    for (const auto &entry : cases)
        if (std::strcmp(argv[1], entry.name) == 0) return entry.run() ? 0 : 1;
    return 2;
}
