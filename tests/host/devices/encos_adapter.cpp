#include "motor_driver_binding.hpp"
#include "can_transport.h"
#include "encos_motor.h"
#include "../motor/encos_motion_reference.hpp"
#include "device_config.h"
#include "motor_axis_codec.hpp"

#include <cstdio>
#include <cstring>
#include <climits>

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); return false; \
} } while (false)

namespace motor = dcu::motor;

bool dcu_can_cancel_tx(unsigned, dcu_can_tx_id_t) { return false; }
namespace codec = dcu::ethercat::motor_axis_codec;

static bool encode_wire_command(const motor::MotorDevice &device, const mc_axis_rx_t &wire, motor::CanFrame &frame)
{
    return device.encode(codec::command(wire), frame) == motor::Result::Ok;
}

static bool decode_wire_feedback(const motor::MotorDevice &device, const motor::CanFrame &frame, mc_axis_tx_t &wire)
{
    motor::Feedback value{};
    constexpr uint32_t required = motor::Position | motor::Velocity | motor::Current;
    if (device.decode(frame, value) != motor::Result::Ok || (value.valid_fields & required) != required ||
        value.current_cA < INT16_MIN || value.current_cA > INT16_MAX || value.motor_fault > UINT8_MAX)
        return false;
    codec::sample(value, wire);
    return true;
}

struct Fixture {
    dcu::motor::MotorConfiguration config{
        {1, -10000000, 10000000, 20000000, 10000, 100},
        {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0},
        0x8000, 0,
    };

    motor::MotorSessionDriver *bind(unsigned slot = 0) const
    {
        return motor::configure_motor_driver(slot, config, 0);
    }

    bool reference_encode(const mc_axis_rx_t &command, encos_frame_t &frame) const
    {
        const auto &route = config.route;
        mc_routing_t wire{};
        wire.CanId = route.can_id;
        wire.PositionLimitMin = route.position_min_urad;
        wire.PositionLimitMax = route.position_max_urad;
        wire.VelocityLimit = route.velocity_limit_urad_s;
        wire.CurrentLimit = route.current_limit_dA;
        wire.TorqueLimit = route.torque_limit_mNm;
        return legacy_reference::encode(&command, &wire, &config.encos.profile, &frame);
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
    CHECK(f.bind() != nullptr);
    CHECK(f.bind(ENCOS_AXIS_COUNT) == nullptr);
    f.config.model_profile_id = 0xFFFF;
    CHECK(!motor::validate_motor_configuration(f.config, 0));
    CHECK(f.bind() == nullptr);
    return true;
}

static bool validation()
{
    Fixture original;
    CHECK(motor::validate_motor_configuration(original.config, 0));
    auto boundary = original.config;
    boundary.route.position_max_urad = 1001000;
    boundary.encos.profile.pos[1] = 1001 / 1000.0;
    CHECK(motor::validate_motor_configuration(boundary, 0));
    ++boundary.route.position_max_urad;
    CHECK(!motor::validate_motor_configuration(boundary, 0));
    for (unsigned which = 0; which < 12; ++which) {
        auto config = original.config;
        switch (which) {
        case 0: config.route.can_id = 0; break;
        case 1: config.route.can_id = 9; break;
        case 2: config.encos.timeout_ms = 0; break;
        case 3: config.encos.timeout_ms = 501; break;
        case 4: config.route.position_max_urad = config.route.position_min_urad; break;
        case 5: config.route.position_min_urad = -10000001; break;
        case 6: config.route.velocity_limit_urad_s = 20000001; break;
        case 7: config.route.current_limit_dA = 101; break;
        case 8: config.route.torque_limit_mNm = 10001; break;
        case 9: config.power_query_period_ms = 19; break;
        case 10: config.encos.frame_format = 1; break;
        case 11: config.encos.profile.kp[1] = config.encos.profile.kp[0]; break;
        }
        CHECK(!motor::validate_motor_configuration(config, 0));
    }
    return true;
}

static bool command_bytes()
{
    Fixture f;
    const auto binding = f.bind();
    CHECK(binding != nullptr);
    motor::MotorDevice device;
    device.bind(*binding);
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
        CHECK(f.reference_encode(command, expected));
        CHECK(encode_wire_command(device, command, actual));
        CHECK(actual.id == expected.id && actual.dlc == expected.dlc && actual.flags == expected.flags);
        CHECK(std::memcmp(actual.data, expected.data, 8) == 0);
    }
    return true;
}

static bool rejected_commands()
{
    Fixture f;
    motor::MotorDevice device;
    device.bind(*f.bind());
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
        CHECK(!encode_wire_command(device, command, frame) && frame.id == 999);
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
    device.bind(*f.bind());
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
        CHECK(legacy_reference::feedback(&frame, &f.config.encos.profile, &expected));
        CHECK(decode_wire_feedback(device, frame, actual));
        CHECK(std::memcmp(&actual, &expected, sizeof(actual)) == 0);
    }
    return true;
}

static bool feedback_rejection()
{
    Fixture f;
    motor::MotorDevice device;
    device.bind(*f.bind());
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
        CHECK(!decode_wire_feedback(device, frame, after));
        CHECK(std::memcmp(&before, &after, sizeof(after)) == 0);
    }
    Fixture wide_current;
    wide_current.config.encos.profile.current[0] = -400;
    wide_current.config.encos.profile.current[1] = 400;
    wide_current.config.route.current_limit_dA = 4000;
    const auto binding = wide_current.bind();
    CHECK(binding != nullptr);
    device.bind(*binding);
    motor::CanFrame overflow{};
    overflow.id = 1;
    overflow.dlc = 8;
    overflow.data[0] = 0x20;
    overflow.data[4] = 0x0F;
    overflow.data[5] = 0xFF;
    mc_axis_tx_t previous{};
    previous.ActualPosition = 123;
    auto after = previous;
    CHECK(!decode_wire_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    TestFeedbackDriver partial;
    partial.value.valid_fields = motor::Current;
    device.bind(partial);
    motor::Feedback logical{};
    CHECK(device.decode(overflow, logical) == motor::Result::Ok);
    CHECK(!decode_wire_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    partial.value.valid_fields = motor::Position | motor::Velocity | motor::Current;
    partial.value.motor_fault = 256;
    CHECK(!decode_wire_feedback(device, overflow, after));
    CHECK(std::memcmp(&previous, &after, sizeof(after)) == 0);
    return true;
}

static Fixture boundary_fixture(unsigned variant)
{
    Fixture f;
    auto &profile = f.config.encos.profile;
    if (variant == 1) {
        profile = {{0, 17}, {0, 3}, {-12.34, 8.76}, {-31.23, 21.34}, {-45.6, 23.4}, {-47.3, 35.7}, 0};
        f.config.route = {8, -12340000, 8760000, 21340000, 23400, 357};
    } else if (variant == 2) {
        profile.current[0] = profile.torque[0] = -500;
        profile.current[1] = profile.torque[1] = 500;
        profile.kt = UINT16_MAX / 100.;
        f.config.route.current_limit_dA = 5000;
        f.config.route.torque_limit_mNm = 500000;
    }
    return f;
}

static bool command_boundaries()
{
    for (unsigned variant = 0; variant < 3; ++variant) {
        const auto f = boundary_fixture(variant);
        const auto binding = f.bind();
        CHECK(binding != nullptr);
        motor::MotorDevice device;
        device.bind(*binding);
        for (uint8_t mode = 1; mode <= 5; ++mode) {
            for (unsigned edge = 0; edge < 42; ++edge) {
                mc_axis_rx_t command{};
                command.Mode = mode;
                command.CurrentLimit = mode <= 2 ? 1 : 0;
                switch (edge) {
                case 0: break;
                case 1: command.TargetPosition = f.config.route.position_min_urad; break;
                case 2: command.TargetPosition = f.config.route.position_max_urad; break;
                case 3: command.TargetPosition = f.config.route.position_min_urad - 1; break;
                case 4: command.TargetPosition = f.config.route.position_max_urad + 1; break;
                case 5: command.TargetVelocity = -1; break;
                case 6: command.TargetVelocity = f.config.route.velocity_limit_urad_s; break;
                case 7: command.TargetVelocity = f.config.route.velocity_limit_urad_s + 1; break;
                case 8: command.TargetVelocity = -static_cast<int32_t>(f.config.route.velocity_limit_urad_s); break;
                case 9: command.TargetVelocity = -static_cast<int32_t>(f.config.route.velocity_limit_urad_s) - 1; break;
                case 10: command.TargetCurrent = 1000; break;
                case 11: command.TargetCurrent = 1001; break;
                case 12: command.TargetCurrent = -1000; break;
                case 13: command.TargetCurrent = -1001; break;
                case 14: command.TargetCurrent = INT16_MIN; break;
                case 15: command.TargetCurrent = INT16_MAX; break;
                case 16: command.TargetTorque = f.config.route.torque_limit_mNm; break;
                case 17: command.TargetTorque = f.config.route.torque_limit_mNm + 1; break;
                case 18: command.TargetTorque = -static_cast<int32_t>(f.config.route.torque_limit_mNm); break;
                case 19: command.TargetTorque = -static_cast<int32_t>(f.config.route.torque_limit_mNm) - 1; break;
                case 20: command.TargetTorque = 9; break;
                case 21: command.TargetTorque = -9; break;
                case 22: command.TargetTorque = 11; break;
                case 23: command.TargetTorque = -11; break;
                case 24: command.TargetTorque = INT16_MAX * 10 + 9; break;
                case 25: command.TargetTorque = INT16_MAX * 10 + 10; break;
                case 26: command.TargetTorque = INT16_MIN * 10 - 9; break;
                case 27: command.TargetTorque = INT16_MIN * 10 - 10; break;
                case 28: command.CurrentLimit = 0; break;
                case 29: command.CurrentLimit = f.config.route.current_limit_dA; break;
                case 30: command.CurrentLimit = f.config.route.current_limit_dA + 1; break;
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
                const bool accepted = f.reference_encode(command, expected);
                CHECK(encode_wire_command(device, command, actual) == accepted);
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
        CHECK(binding != nullptr);
        motor::MotorDevice device;
        device.bind(*binding);
        for (uint16_t position : positions) {
            for (unsigned raw = 0; raw < 4096; ++raw) {
                motor::CanFrame frame{};
                frame.id = f.config.route.can_id;
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
                const bool accepted = legacy_reference::feedback(&frame, &f.config.encos.profile, &expected);
                CHECK(decode_wire_feedback(device, frame, actual) == accepted);
                CHECK(std::memcmp(&actual, accepted ? &expected : &before, sizeof(actual)) == 0);
            }
        }
    }
    return true;
}

static bool axis_isolation()
{
    Fixture first, second;
    second.config.route.can_id = 7;
    second.config.encos.profile.kt = 0;
    motor::MotorDevice a, b;
    a.bind(*first.bind(0));
    b.bind(*second.bind(1));
    CHECK(a.capabilities().torque_constant_Nm_A == 0.5 && b.capabilities().torque_constant_Nm_A == 0);
    CHECK((a.capabilities().feedback_fields & motor::Torque) && !(b.capabilities().feedback_fields & motor::Torque));
    CHECK(!a.capabilities().active_stop_command && !b.capabilities().active_stop_command);
    mc_axis_rx_t command{};
    command.Mode = 3;
    motor::CanFrame x{}, y{};
    CHECK(encode_wire_command(a, command, x) && encode_wire_command(b, command, y));
    CHECK(x.id == 1 && y.id == 7);
    second.config.model_profile_id = 0xFFFF;
    CHECK(second.bind(1) == nullptr);
    CHECK(encode_wire_command(a, command, x) && x.id == 1);
    return true;
}

static bool field_mapping()
{
    const mc_axis_rx_t wire{15, 4, 7, INT32_MIN, INT32_MAX, -12345, INT16_MIN, 123, 456, UINT16_MAX};
    const auto command = codec::command(wire);
    const auto copy = codec::command(command);
    CHECK(std::memcmp(&wire, &copy, sizeof(wire)) == 0);
    mc_sdo_command_t sdo{};
    sdo.ControlWord = wire.ControlWord; sdo.Mode = wire.Mode; sdo.Flags = wire.Flags;
    sdo.TargetPosition = wire.TargetPosition; sdo.TargetVelocity = wire.TargetVelocity;
    sdo.TargetTorque = wire.TargetTorque; sdo.TargetCurrent = wire.TargetCurrent;
    sdo.Kp = wire.Kp; sdo.Kd = wire.Kd; sdo.CurrentLimit = wire.CurrentLimit;
    const auto sdo_copy = codec::command(codec::command(sdo));
    CHECK(std::memcmp(&wire, &sdo_copy, sizeof(wire)) == 0);

    dcu::motor::MotorFeedback feedback{};
    feedback.sample = {63, -123, 456, -789, 2345, -1234, 3276, 7};
    feedback.mode = motor::ControlMode::Impedance;
    feedback.status_word = 0x1234;
    feedback.bridge_fault = 9;
    feedback.power_mW = -567;
    feedback.feedback_age_ms = 123;
    feedback.power_age_ms = 456;
    feedback.power_valid = true;
    const auto tx = codec::feedback(feedback);
    CHECK(tx.StatusWord == 0x1234 && tx.ModeDisplay == 4 && tx.ValidFlags == 127);
    CHECK(tx.ActualPosition == -123 && tx.ActualVelocity == 456 && tx.ActualCurrent == -789 && tx.EstimatedTorque == 2345);
    CHECK(tx.MotorTemperature == -1234 && tx.MosTemperature == 3276 && tx.MotorFaultRaw == 7 && tx.BridgeFaultCode == 9);
    CHECK(tx.ActualPower == -567 && tx.FeedbackAgeMs == 123 && tx.PowerAgeMs == 456);
    const uint32_t fields[] = {motor::Position, motor::Velocity, motor::Current, motor::Torque,
                              motor::MotorTemperature, motor::MosTemperature};
    for (unsigned i = 0; i < 6; ++i) {
        feedback.sample.valid_fields = fields[i];
        feedback.power_valid = false;
        CHECK(codec::feedback(feedback).ValidFlags == (1U << i));
    }
    dcu::motor::MotorDiagnostics diagnostic{1, 2, 3, 4, 5, 6, -7, -8, 9, 10};
    mc_diag_t mirror{};
    mirror.LastServiceError = 123;
    codec::diagnostics(diagnostic, mirror);
    CHECK(mirror.TxEnqueueCount == 1 && mirror.TxDropCount == 2 && mirror.RxValidCount == 3);
    CHECK(mirror.ProtocolErrorCount == 4 && mirror.ActiveFaultBits == 5 && mirror.TxCompleteCount == 6);
    CHECK(mirror.QueriedPosition == -7 && mirror.QueriedVelocity == -8);
    CHECK(mirror.PositionQueryAgeUs == 9 && mirror.VelocityQueryAgeUs == 10 && mirror.LastServiceError == 123);

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
        {"field_mapping", field_mapping},
    };
    for (const auto &entry : cases)
        if (std::strcmp(argv[1], entry.name) == 0) return entry.run() ? 0 : 1;
    return 2;
}
