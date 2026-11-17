#include "motor_driver_binding.hpp"
#include "encos_motor.h"
#include "../motor/encos_motion_reference.hpp"
#include "device_config.h"

#include <cstdio>
#include <cstring>
#include <climits>

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); return false; \
} } while (false)

namespace motor = dcu::motor;
namespace app = dcu::app;

struct Fixture {
    mc_range_t range{0, 500, 0, 5, -1000, 1000, -2000, 2000, -100, 100, -100, 100};
    mc_basic_t basic{};
    mc_routing_t route{};
    encos_profile_t profile{};
    Fixture()
    {
        basic.TorqueConstantRaw = 50;
        basic.CanTimeoutMs = 100;
        route.CanId = 1;
        route.ModelProfileId = 0x8000;
        route.FeedbackPolicy = 1;
        route.PositionLimitMin = -10000000;
        route.PositionLimitMax = 10000000;
        route.VelocityLimit = 20000000;
        route.CurrentLimit = 100;
        route.TorqueLimit = 10000;
        encos_profile(&range, &basic, &profile);
    }
    app::MotorBinding bind(unsigned slot = 0) const
    {
        return app::configure_motor_driver(slot, route, basic, range, 0);
    }
};

class TestFeedbackDriver final : public motor::MotorDriver {
public:
    motor::Feedback value{};
    const motor::Capabilities &capabilities() const noexcept override
    {
        static constexpr motor::Capabilities caps{0, motor::Position | motor::Velocity | motor::Current, 0, false};
        return caps;
    }
    motor::Result encode(const motor::Command &, motor::CanFrame &) const noexcept override
    {
        return motor::Result::Unsupported;
    }
    motor::Result decode(const motor::CanFrame &, motor::Feedback &feedback) const noexcept override
    {
        feedback = value;
        return motor::Result::Ok;
    }
};

static bool registry()
{
    Fixture f;
    CHECK(app::motor_parameter_dialect(0x8000) == app::ParameterDialect::EncosLegacy);
    CHECK(app::motor_parameter_dialect(0xFFFF) == app::ParameterDialect::None);
    CHECK(f.bind().driver != nullptr && f.bind().parameter_dialect == app::ParameterDialect::EncosLegacy);
    CHECK(f.bind(ENCOS_AXIS_COUNT).driver == nullptr);
    f.route.ModelProfileId = 0xFFFF;
    CHECK(!app::validate_motor_configuration(f.route, f.basic, f.range, 0));
    CHECK(f.bind().driver == nullptr);
    return true;
}

static bool validation()
{
    Fixture original;
    CHECK(app::validate_motor_configuration(original.route, original.basic, original.range, 0));
    for (unsigned which = 0; which < 14; ++which) {
        Fixture f;
        switch (which) {
        case 0: f.route.CanId = 0; break;
        case 1: f.route.CanId = 9; break;
        case 2: f.route.StopPolicy = 1; break;
        case 3: f.route.FeedbackPolicy = 0; break;
        case 4: f.basic.CanTimeoutMs = 0; break;
        case 5: f.basic.CanTimeoutMs = 501; break;
        case 6: f.route.PositionLimitMax = f.route.PositionLimitMin; break;
        case 7: f.route.PositionLimitMin = -10000001; break;
        case 8: f.route.VelocityLimit = 20000001; break;
        case 9: f.route.CurrentLimit = 101; break;
        case 10: f.route.TorqueLimit = 10001; break;
        case 11: f.route.PowerQueryPeriodMs = 19; break;
        case 12: f.basic.CommunicationMode = 1; break;
        case 13: f.range.KpMax = f.range.KpMin; break;
        }
        CHECK(!app::validate_motor_configuration(f.route, f.basic, f.range, 0));
    }
    return true;
}

static bool command_bytes()
{
    Fixture f;
    const auto binding = f.bind();
    CHECK(binding.driver != nullptr);
    motor::MotorDevice device;
    device.bind(*binding.driver);
    uint32_t random = 17;
    for (unsigned i = 0; i < 1000; ++i) {
        random = random * 1664525U + 1013904223U;
        mc_axis_rx_t command{};
        command.Mode = static_cast<uint8_t>(i % 5 + 1);
        command.ControlWord = 9;
        command.TargetPosition = command.Mode == 1 || command.Mode == 4 ?
            static_cast<int32_t>(random % 20000001U) - 10000000 : 0;
        command.TargetVelocity = command.Mode == 1 || command.Mode == 2 || command.Mode == 4 ?
            static_cast<int32_t>(random % 20000001U) : 0;
        command.TargetCurrent = command.Mode == 3 ? static_cast<int32_t>(random % 2001U) - 1000 : 0;
        command.TargetTorque = command.Mode == 4 || command.Mode == 5 ?
            static_cast<int32_t>(random % 20001U) - 10000 : 0;
        command.CurrentLimit = command.Mode <= 2 ? 100 : 0;
        command.Kp = command.Mode == 4 ? 1234 : 0;
        command.Kd = command.Mode == 4 ? 2345 : 0;
        motor::CanFrame expected{}, actual{};
        CHECK(legacy_reference::encode(&command, &f.route, &f.profile, &expected));
        CHECK(app::encode_motor_command(device, command, actual));
        CHECK(actual.id == expected.id && actual.dlc == expected.dlc && actual.flags == expected.flags);
        CHECK(std::memcmp(actual.data, expected.data, 8) == 0);
    }
    return true;
}

static bool rejected_commands()
{
    Fixture f;
    motor::MotorDevice device;
    device.bind(*f.bind().driver);
    for (unsigned which = 0; which < 7; ++which) {
        mc_axis_rx_t command{};
        command.Mode = 3;
        switch (which) {
        case 0: command.Mode = 0; break;
        case 1: command.Mode = 6; break;
        case 2: command.ControlWord = 16; break;
        case 3: command.Flags = 1; break;
        case 4: command.TargetPosition = 1; break;
        case 5: command.TargetCurrent = 1001; break;
        case 6: command.CurrentLimit = 1; break;
        }
        motor::CanFrame frame{};
        frame.id = 999;
        CHECK(!app::encode_motor_command(device, command, frame) && frame.id == 999);
    }
    motor::Command wide{};
    wide.mode = motor::ControlMode::Current;
    motor::CanFrame frame{};
    frame.id = 999;
    const int32_t unrepresentable[] = {65536, -65536};
    for (int32_t current : unrepresentable) {
        wide.current_cA = current;
        CHECK(device.encode(wide, frame) == motor::Result::InvalidCommand && frame.id == 999);
    }
    return true;
}

static bool feedback_bytes()
{
    Fixture f;
    motor::MotorDevice device;
    device.bind(*f.bind().driver);
    uint32_t random = 19;
    for (unsigned i = 0; i < 256; ++i) {
        motor::CanFrame frame{};
        frame.id = 1;
        frame.dlc = 8;
        frame.data[0] = static_cast<uint8_t>(0x20 | (i & 7));
        for (unsigned j = 1; j < 8; ++j) {
            random = random * 1664525U + 1013904223U;
            frame.data[j] = static_cast<uint8_t>(random >> 24);
        }
        mc_axis_tx_t expected, actual;
        std::memset(&expected, 0x5A, sizeof(expected));
        expected.ValidFlags = 64;
        actual = expected;
        CHECK(legacy_reference::feedback(&frame, &f.profile, &expected));
        CHECK(app::decode_motor_feedback(device, frame, actual));
        CHECK(std::memcmp(&actual, &expected, sizeof(actual)) == 0);
    }
    return true;
}

static bool feedback_rejection()
{
    Fixture f;
    motor::MotorDevice device;
    device.bind(*f.bind().driver);
    for (unsigned which = 0; which < 5; ++which) {
        motor::CanFrame frame{};
        frame.id = 1;
        frame.dlc = 8;
        frame.data[0] = 0x20;
        switch (which) {
        case 0: frame.id = 2; break;
        case 1: frame.dlc = 9; break;
        case 2: frame.dlc = 7; break;
        case 3: frame.data[0] = 0x40; break;
        case 4: frame.flags = 2; break;
        }
        mc_axis_tx_t before{};
        before.ActualPosition = 123;
        before.ValidFlags = 64;
        auto after = before;
        CHECK(!app::decode_motor_feedback(device, frame, after));
        CHECK(std::memcmp(&before, &after, sizeof(after)) == 0);
    }
    Fixture wide_current;
    wide_current.range.CurMinRaw = -4000;
    wide_current.range.CurMaxRaw = 4000;
    wide_current.route.CurrentLimit = 4000;
    const auto binding = wide_current.bind();
    CHECK(binding.driver != nullptr);
    device.bind(*binding.driver);
    motor::CanFrame overflow{};
    overflow.id = 1;
    overflow.dlc = 8;
    overflow.data[0] = 0x20;
    overflow.data[4] = 0x0F;
    overflow.data[5] = 0xFF;
    mc_axis_tx_t previous{};
    previous.ActualPosition = 123;
    auto after = previous;
    CHECK(!app::decode_motor_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    TestFeedbackDriver partial;
    partial.value.valid_fields = motor::Current;
    device.bind(partial);
    motor::Feedback logical{};
    CHECK(device.decode(overflow, logical) == motor::Result::Ok);
    CHECK(!app::decode_motor_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    partial.value.valid_fields = motor::Position | motor::Velocity | motor::Current;
    partial.value.motor_fault = 256;
    CHECK(!app::decode_motor_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    return true;
}

static Fixture boundary_fixture(unsigned variant)
{
    Fixture f;
    if (variant == 1) {
        f.range = {0, 17, 0, 3, -1234, 876, -3123, 2134, -456, 234, -473, 357};
        f.basic.TorqueConstantRaw = 0;
        f.route.CanId = 8;
        f.route.PositionLimitMin = -12340000;
        f.route.PositionLimitMax = 8760000;
        f.route.VelocityLimit = 21340000;
        f.route.CurrentLimit = 357;
        f.route.TorqueLimit = 23400;
    } else if (variant == 2) {
        f.range.CurMinRaw = f.range.TorMinRaw = -5000;
        f.range.CurMaxRaw = f.range.TorMaxRaw = 5000;
        f.basic.TorqueConstantRaw = UINT16_MAX;
        f.route.CurrentLimit = 5000;
        f.route.TorqueLimit = 500000;
    }
    encos_profile(&f.range, &f.basic, &f.profile);
    return f;
}

static bool command_boundaries()
{
    for (unsigned variant = 0; variant < 3; ++variant) {
        const auto f = boundary_fixture(variant);
        const auto binding = f.bind();
        CHECK(binding.driver != nullptr);
        motor::MotorDevice device;
        device.bind(*binding.driver);
        for (uint8_t mode = 1; mode <= 5; ++mode) {
            for (unsigned edge = 0; edge < 42; ++edge) {
                mc_axis_rx_t command{};
                command.Mode = mode;
                command.CurrentLimit = mode <= 2 ? 1 : 0;
                switch (edge) {
                case 0: break;
                case 1: command.TargetPosition = f.route.PositionLimitMin; break;
                case 2: command.TargetPosition = f.route.PositionLimitMax; break;
                case 3: command.TargetPosition = f.route.PositionLimitMin - 1; break;
                case 4: command.TargetPosition = f.route.PositionLimitMax + 1; break;
                case 5: command.TargetVelocity = -1; break;
                case 6: command.TargetVelocity = f.route.VelocityLimit; break;
                case 7: command.TargetVelocity = f.route.VelocityLimit + 1; break;
                case 8: command.TargetVelocity = -static_cast<int32_t>(f.route.VelocityLimit); break;
                case 9: command.TargetVelocity = -static_cast<int32_t>(f.route.VelocityLimit) - 1; break;
                case 10: command.TargetCurrent = 1000; break;
                case 11: command.TargetCurrent = 1001; break;
                case 12: command.TargetCurrent = -1000; break;
                case 13: command.TargetCurrent = -1001; break;
                case 14: command.TargetCurrent = INT16_MIN; break;
                case 15: command.TargetCurrent = INT16_MAX; break;
                case 16: command.TargetTorque = f.route.TorqueLimit; break;
                case 17: command.TargetTorque = f.route.TorqueLimit + 1; break;
                case 18: command.TargetTorque = -static_cast<int32_t>(f.route.TorqueLimit); break;
                case 19: command.TargetTorque = -static_cast<int32_t>(f.route.TorqueLimit) - 1; break;
                case 20: command.TargetTorque = 9; break;
                case 21: command.TargetTorque = -9; break;
                case 22: command.TargetTorque = 11; break;
                case 23: command.TargetTorque = -11; break;
                case 24: command.TargetTorque = INT16_MAX * 10 + 9; break;
                case 25: command.TargetTorque = INT16_MAX * 10 + 10; break;
                case 26: command.TargetTorque = INT16_MIN * 10 - 9; break;
                case 27: command.TargetTorque = INT16_MIN * 10 - 10; break;
                case 28: command.CurrentLimit = 0; break;
                case 29: command.CurrentLimit = f.route.CurrentLimit; break;
                case 30: command.CurrentLimit = f.route.CurrentLimit + 1; break;
                case 31: command.CurrentLimit = 4095; break;
                case 32: command.CurrentLimit = 4096; break;
                case 33: command.Kp = 1700; break;
                case 34: command.Kp = 1701; break;
                case 35: command.Kp = 50000; break;
                case 36: command.Kd = 5000; break;
                case 37: command.Kd = 5001; break;
                case 38: command.ControlWord = 15; break;
                case 39: command.ControlWord = 16; break;
                case 40: command.Flags = 1; break;
                case 41: command.TargetVelocity = 1; break;
                }
                motor::CanFrame expected{}, actual{};
                std::memset(&actual, 0x5A, sizeof(actual));
                const auto before = actual;
                const bool accepted = legacy_reference::encode(&command, &f.route, &f.profile, &expected);
                CHECK(app::encode_motor_command(device, command, actual) == accepted);
                if (accepted) {
                    CHECK(actual.id == expected.id && actual.dlc == expected.dlc && actual.flags == expected.flags);
                    CHECK(std::memcmp(actual.data, expected.data, sizeof(actual.data)) == 0);
                } else {
                    CHECK(std::memcmp(&actual, &before, sizeof(actual)) == 0);
                }
            }
        }
    }
    return true;
}

static bool feedback_boundaries()
{
    const uint16_t positions[] = {0, 1, 32767, 32768, 65534, 65535};
    for (unsigned variant = 0; variant < 3; ++variant) {
        const auto f = boundary_fixture(variant);
        const auto binding = f.bind();
        CHECK(binding.driver != nullptr);
        motor::MotorDevice device;
        device.bind(*binding.driver);
        for (uint16_t position : positions) {
            for (unsigned raw = 0; raw < 4096; ++raw) {
                motor::CanFrame frame{};
                frame.id = f.route.CanId;
                frame.flags = raw & 1 ? 24 : 0;
                frame.dlc = 8;
                frame.data[0] = 0x20 | (raw & 7);
                frame.data[1] = position >> 8;
                frame.data[2] = position;
                frame.data[3] = raw >> 4;
                frame.data[4] = (raw & 15) << 4 | raw >> 8;
                frame.data[5] = raw;
                frame.data[6] = raw;
                frame.data[7] = 255 - frame.data[6];
                mc_axis_tx_t expected;
                std::memset(&expected, 0x5A, sizeof(expected));
                expected.ValidFlags = raw & 1 ? 0xFF : 0;
                const auto before = expected;
                auto actual = before;
                const bool accepted = legacy_reference::feedback(&frame, &f.profile, &expected);
                CHECK(app::decode_motor_feedback(device, frame, actual) == accepted);
                CHECK(std::memcmp(&actual, accepted ? &expected : &before, sizeof(actual)) == 0);
            }
        }
    }
    return true;
}

static bool axis_isolation()
{
    Fixture first, second;
    second.route.CanId = 7;
    second.basic.TorqueConstantRaw = 0;
    motor::MotorDevice a, b;
    a.bind(*first.bind(0).driver);
    b.bind(*second.bind(1).driver);
    CHECK(a.capabilities().torque_constant_Nm_A == 0.5 && b.capabilities().torque_constant_Nm_A == 0);
    CHECK((a.capabilities().feedback_fields & motor::Torque) && !(b.capabilities().feedback_fields & motor::Torque));
    CHECK(!a.capabilities().active_stop_command && !b.capabilities().active_stop_command);
    mc_axis_rx_t command{};
    command.Mode = 3;
    motor::CanFrame x{}, y{};
    CHECK(app::encode_motor_command(a, command, x) && app::encode_motor_command(b, command, y));
    CHECK(x.id == 1 && y.id == 7);
    second.route.ModelProfileId = 0xFFFF;
    CHECK(second.bind(1).driver == nullptr);
    CHECK(app::encode_motor_command(a, command, x) && x.id == 1);
    return true;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    struct Case { const char *name; bool (*run)(); };
    const Case cases[] = {
        {"registry", registry}, {"validation", validation}, {"commands", command_bytes},
        {"reject_commands", rejected_commands}, {"feedback", feedback_bytes},
        {"reject_feedback", feedback_rejection}, {"axis_isolation", axis_isolation},
        {"command_boundaries", command_boundaries}, {"feedback_boundaries", feedback_boundaries},
    };
    for (const auto &entry : cases)
        if (std::strcmp(argv[1], entry.name) == 0) return entry.run() ? 0 : 1;
    return 2;
}
