#include "encos_motor.hpp"
#include "bjcx_status.hpp"
#include "encos_interface.h"

#include <cstdio>
#include <cmath>
#include <cstring>

namespace motor = dcu::motor;
namespace encos = dcu::encos;

extern "C" size_t motor_c_layout(unsigned index);
extern "C" bool motor_c_query(dcu_can_frame_t *frame);

#define CHECK(expression) do { \
    if (!(expression)) { \
        std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); \
        return false; \
    } \
} while (false)

struct MotorFixture {
    motor::Routing route{1, -10000000, 10000000, 20000000, 10000, 100};
    encos_profile_t profile{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5};
};

static mc_object_t *object(uint16_t index)
{
    for (unsigned i = 0; i < mc_object_count; ++i)
        if (mc_objects[i].index == index)
            return &mc_objects[i];
    return nullptr;
}

static bool c_abi()
{
    dcu_can_frame_t frame{};
    frame.id = 0x123;
    frame.flags = 24;
    CHECK(motor_c_query(&frame));
    CHECK(frame.id == 0x123 && frame.flags == 24);
    CHECK(frame.dlc == 2 && frame.data[0] == 0xE0 && frame.data[1] == 5);
    return true;
}

static bool layout()
{
    const size_t expected[] = {
        sizeof(dcu_can_frame_t), alignof(dcu_can_frame_t),
        sizeof(mc_axis_rx_t), alignof(mc_axis_rx_t),
        sizeof(mc_axis_tx_t), alignof(mc_axis_tx_t),
        sizeof(mc_imu_t), alignof(mc_imu_t),
        offsetof(mc_imu_t, TimestampNs), sizeof(mc_field_t), sizeof(mc_object_t)
    };
    for (unsigned i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i)
        CHECK(motor_c_layout(i) == expected[i]);

    unsigned fields = 0;
    for (unsigned i = 0; i < mc_object_count; ++i) {
        const auto &entry = mc_objects[i];
        CHECK(entry.data != nullptr && entry.size != 0);
        CHECK(i == 0 || mc_objects[i - 1].index < entry.index);
        fields += entry.count;
    }
    CHECK(mc_object_count != 0 && fields == mc_field_count);
    for (unsigned i = 0; i < mc_field_count; ++i) {
        const auto &field = mc_fields[i];
        const auto *entry = object(field.index);
        CHECK(entry != nullptr);
        CHECK(field.sub >= 1 && field.sub <= entry->count);
        CHECK(field.bytes != 0 && field.offset + field.bytes <= entry->size);
        CHECK(i == 0 || mc_fields[i - 1].index < field.index ||
              (mc_fields[i - 1].index == field.index && mc_fields[i - 1].sub < field.sub));
    }
    const auto *imu = object(0x6300);
    const uint8_t empty_imu[64]{};
    CHECK(imu != nullptr && imu->size == sizeof(empty_imu));
    CHECK(std::memcmp(imu->data, empty_imu, sizeof(empty_imu)) == 0);
    return true;
}

static bool profile()
{
    MotorFixture motor;
    CHECK(encos_profile_valid(&motor.profile));
    CHECK(motor.profile.kp[1] == 500 && motor.profile.kd[1] == 5);
    CHECK(motor.profile.pos[0] == -10 && motor.profile.vel[1] == 20);
    CHECK(motor.profile.current[0] == -10 && motor.profile.kt == 0.5);
    motor.profile.kp[1] = motor.profile.kp[0];
    CHECK(!encos_profile_valid(&motor.profile));
    return true;
}

static bool commands()
{
    MotorFixture motor;
    const uint8_t sizes[] = {8, 7, 3, 8, 3};
    const uint8_t golden[][8] = {
        {0x20, 0, 0, 0, 0, 0, 1, 0x91},
        {0x41, 0, 0, 0, 0, 0, 100, 0},
        {0x61, 0xFF, 0x83, 0, 0, 0, 0, 0},
        {0, 0, 0, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF},
        {0x65, 0xFF, 0x83, 0, 0, 0, 0, 0}
    };
    for (uint8_t mode = 1; mode <= 5; ++mode) {
        motor::Command command{};
        command.mode = static_cast<motor::ControlMode>(mode);
        command.current_limit_dA = mode <= 2 ? 100 : 0;
        command.current_cA = mode == 3 ? -125 : 0;
        command.torque_mNm = mode == 5 ? -1250 : 0;
        dcu_can_frame_t frame{};
        CHECK(encos::encode_motion(command, motor.route, motor.profile, frame));
        CHECK(frame.id == 1 && frame.flags == 0 && frame.dlc == sizes[mode - 1]);
        CHECK(std::memcmp(frame.data, golden[mode - 1], sizeof(golden[0])) == 0);
    }
    return true;
}

static bool rejected_commands()
{
    MotorFixture motor;
    motor::Command command{};
    command.mode = motor::ControlMode::Current;
    dcu_can_frame_t frame{};
    CHECK(encos::encode_motion(command, motor.route, motor.profile, frame));
    command.reserved_flags = 1;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    command.reserved_flags = 0;
    command.control_bits = 16;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    command.control_bits = 0;
    command.current_cA = 1001;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    command.current_cA = 0;
    command.position_urad = 1;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    command.position_urad = 0;
    motor.route.can_id = 0;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    motor.route.can_id = 9;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    motor.route.can_id = 1;
    command.mode = motor::ControlMode::Impedance;
    command.position_urad = motor.route.position_max_urad + 1;
    CHECK(!encos::encode_motion(command, motor.route, motor.profile, frame));
    return true;
}

static bool feedback()
{
    MotorFixture motor;
    dcu_can_frame_t frame{};
    frame.dlc = 8;
    frame.data[0] = 0x20;
    frame.data[6] = 50;
    frame.data[7] = 70;
    motor::Feedback state{};
    CHECK(encos::decode_feedback(frame, motor.profile, state));
    CHECK(state.position_urad == -10000000 && state.velocity_urad_s == -20000000);
    CHECK(state.current_cA == -1000 && state.torque_mNm == -5000);
    CHECK(state.motor_temperature_deciC == 0 && state.mos_temperature_deciC == 100);
    CHECK(state.valid_fields == 0x3F && state.motor_fault == 0);
    frame.dlc = 7;
    CHECK(!encos::decode_feedback(frame, motor.profile, state));
    frame.dlc = 8;
    frame.data[0] = 0x40;
    CHECK(!encos::decode_feedback(frame, motor.profile, state));
    frame.data[0] = 0x28;
    CHECK(!encos::decode_feedback(frame, motor.profile, state));
    return true;
}

static bool transactional_outputs()
{
    MotorFixture fixture;
    motor::CanFrame frame{};
    std::memset(&frame, 0x5A, sizeof(frame));
    const auto before = frame;
    for (unsigned which = 0; which < 4; ++which) {
        motor::Command command{};
        command.mode = motor::ControlMode::Position;
        command.current_limit_dA = 1;
        switch (which) {
        case 0: command.velocity_urad_s = -1; break;
        case 1:
            command.mode = motor::ControlMode::Impedance;
            command.current_limit_dA = 0;
            command.kd_milli = 5001;
            break;
        case 2: command.current_cA = 65536; break;
        case 3: command.mode = static_cast<motor::ControlMode>(255); break;
        }
        CHECK(!encos::encode_motion(command, fixture.route, fixture.profile, frame));
        CHECK(std::memcmp(&frame, &before, sizeof(frame)) == 0);
    }
    frame = {};
    frame.dlc = 8;
    frame.data[0] = 0x20;
    fixture.profile.current[0] = -500;
    fixture.profile.current[1] = 500;
    motor::Feedback previous{};
    previous.position_urad = 123;
    previous.valid_fields = motor::Position;
    auto state = previous;
    CHECK(!encos::decode_feedback(frame, fixture.profile, state));
    CHECK(std::memcmp(&state, &previous, sizeof(state)) == 0);
    for (unsigned which = 0; which < 3; ++which) {
        auto profile = MotorFixture{}.profile;
        if (which == 0) profile.pos[0] = -3000;
        if (which == 1) profile.vel[0] = -3000;
        if (which == 2) profile.current[0] = NAN;
        CHECK(!encos::decode_feedback(frame, profile, state));
        CHECK(std::memcmp(&state, &previous, sizeof(state)) == 0);
    }
    return true;
}

static bool parameters()
{
    dcu_can_frame_t frame{};
    CHECK(encos_query(36, 10, &frame));
    CHECK(frame.dlc == 2 && frame.data[0] == 0xEA && frame.data[1] == 36);
    CHECK(!encos_query(36, 0, &frame) && !encos_query(36, 11, &frame));
    CHECK(!encos_query(6, 1, &frame));
    CHECK(encos_parameter_query(1) == 5 && encos_parameter_query(2) == 0);
    CHECK(encos_parameter_query(0x101) == 1 && encos_parameter_query(0x127) == 39);
    CHECK(encos_parameter_query(0x128) == 0);
    encos_parameter_values_t values{{50, 0, 0, 0}, 0};
    CHECK(encos_parameter(3, &values, &frame));
    CHECK(frame.dlc == 4 && frame.data[0] == 0xC0 && frame.data[1] == 4);
    CHECK(frame.data[2] == 0 && frame.data[3] == 50);
    CHECK(!encos_parameter(1, &values, &frame));
    return true;
}

static bool floats()
{
    const uint8_t one[] = {0x3F, 0x80, 0, 0};
    const uint8_t infinity[] = {0x7F, 0x80, 0, 0};
    const uint8_t nan[] = {0x7F, 0xC0, 0, 0};
    int32_t value = 0;
    CHECK(encos_be16(one) == 0x3F80 && encos_be32(one) == 0x3F800000);
    CHECK(encos_float(one, 1000000, &value) && value == 1000000);
    CHECK(!encos_float(infinity, 1, &value) && !encos_float(nan, 1, &value));
    CHECK(!encos_float(one, 2147483648.0, &value));
    return true;
}

static bool bjcx_status()
{
    constexpr uint16_t id = 0x123;
    const auto request = dcu::bjcx::status_request(id);
    CHECK(request.id == id && request.flags == (DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS));
    CHECK(request.dlc == 1 && request.data[0] == 0x17);

    motor::CanFrame reply{id, 10, request.flags,
        {0x80, 0x53, 0x45, 0x42, 0xb4, 0, 0, 0xc1, 0xf0, 0, 0, 0xff, 0x9c, 45, 51, 0}};
    motor::Feedback out{};
    CHECK(dcu::bjcx::decode_status_reply(id, reply, out));
    CHECK(out.position_urad == 1570796 && out.velocity_urad_s == -3141593);
    CHECK(out.current_cA == -100 && out.motor_temperature_deciC == -50 && out.mos_temperature_deciC == 10);
    CHECK(out.motor_fault == 0x345 && out.valid_fields == (motor::Position | motor::Velocity |
          motor::Current | motor::MotorTemperature | motor::MosTemperature));

    reply.data[3] = 0x7f; reply.data[4] = 0xc0; // NaN affects only position.
    CHECK(dcu::bjcx::decode_status_reply(id, reply, out));
    CHECK(!(out.valid_fields & motor::Position) && (out.valid_fields & motor::Velocity) && out.motor_fault == 0x345);
    reply.data[1] = 0x63;
    CHECK(dcu::bjcx::decode_status_reply(id, reply, out));
    CHECK(out.valid_fields == (motor::Current | motor::MotorTemperature | motor::MosTemperature));
    for (unsigned error = 0; error < 4; ++error) {
        auto bad = reply;
        if (error == 0) ++bad.id;
        if (error == 1) bad.flags = DCU_CAN_FLAG_FD;
        if (error == 2) bad.dlc = 9;
        if (error == 3) bad.data[0] = 0x17;
        out.motor_fault = 0xabc;
        CHECK(!dcu::bjcx::decode_status_reply(id, bad, out) && out.motor_fault == 0xabc);
    }
    return true;
}

int main(int argc, char **argv)
{
    const struct { const char *name; bool (*run)(); } cases[] = {
        {"abi", c_abi}, {"layout", layout}, {"profile", profile},
        {"commands", commands}, {"reject", rejected_commands},
        {"feedback", feedback}, {"parameters", parameters}, {"float", floats},
        {"transactional", transactional_outputs}, {"bjcx_status", bjcx_status}
    };
    if (argc != 2)
        return 2;
    for (const auto &test : cases)
        if (std::strcmp(argv[1], test.name) == 0)
            return test.run() ? 0 : 1;
    return 2;
}
