#include "encos_motor.h"
#include "motor_control.h"

#include <cstdio>
#include <cstring>

extern "C" size_t motor_c_layout(unsigned index);
extern "C" bool motor_c_query(encos_frame_t *frame);

#define CHECK(expression) do { \
    if (!(expression)) { \
        std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); \
        return false; \
    } \
} while (false)

struct MotorFixture {
    mc_range_t range{0, 500, 0, 5, -1000, 1000, -2000, 2000, -100, 100, -100, 100};
    mc_basic_t basic{};
    mc_routing_t route{};
    encos_profile_t profile{};

    MotorFixture()
    {
        basic.TorqueConstantRaw = 50;
        basic.CanTimeoutMs = 100;
        route.CanId = 1;
        route.PositionLimitMin = -10000000;
        route.PositionLimitMax = 10000000;
        route.VelocityLimit = 20000000;
        route.CurrentLimit = 100;
        route.TorqueLimit = 10000;
        encos_profile(&range, &basic, &profile);
    }
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
    encos_frame_t frame{};
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
        sizeof(encos_frame_t), alignof(encos_frame_t),
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
    CHECK(encos_profile(&motor.range, &motor.basic, &motor.profile));
    CHECK(motor.profile.kp[1] == 500 && motor.profile.kd[1] == 5);
    CHECK(motor.profile.pos[0] == -10 && motor.profile.vel[1] == 20);
    CHECK(motor.profile.current[0] == -10 && motor.profile.kt == 0.5);
    motor.basic.CanTimeoutMs = 0;
    CHECK(!encos_profile(&motor.range, &motor.basic, &motor.profile));
    motor.basic.CanTimeoutMs = 100;
    motor.range.KpMax = motor.range.KpMin;
    CHECK(!encos_profile(&motor.range, &motor.basic, &motor.profile));
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
        mc_axis_rx_t command{};
        command.Mode = mode;
        command.CurrentLimit = mode <= 2 ? 100 : 0;
        command.TargetCurrent = mode == 3 ? -125 : 0;
        command.TargetTorque = mode == 5 ? -1250 : 0;
        encos_frame_t frame{};
        CHECK(encos_encode(&command, &motor.route, &motor.profile, &frame));
        CHECK(frame.id == 1 && frame.flags == 0 && frame.dlc == sizes[mode - 1]);
        CHECK(std::memcmp(frame.data, golden[mode - 1], sizeof(frame.data)) == 0);
    }
    return true;
}

static bool rejected_commands()
{
    MotorFixture motor;
    mc_axis_rx_t command{};
    command.Mode = 3;
    encos_frame_t frame{};
    CHECK(encos_encode(&command, &motor.route, &motor.profile, &frame));
    command.Flags = 1;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    command.Flags = 0;
    command.ControlWord = 16;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    command.ControlWord = 0;
    command.TargetCurrent = 1001;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    command.TargetCurrent = 0;
    command.TargetPosition = 1;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    command.TargetPosition = 0;
    motor.route.CanId = 0;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    motor.route.CanId = 9;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    motor.route.CanId = 1;
    command.Mode = 4;
    command.TargetPosition = motor.route.PositionLimitMax + 1;
    CHECK(!encos_encode(&command, &motor.route, &motor.profile, &frame));
    return true;
}

static bool feedback()
{
    MotorFixture motor;
    encos_frame_t frame{};
    frame.dlc = 8;
    frame.data[0] = 0x20;
    frame.data[6] = 50;
    frame.data[7] = 70;
    mc_axis_tx_t state{};
    state.ValidFlags = 64;
    CHECK(encos_feedback(&frame, &motor.profile, &state));
    CHECK(state.ActualPosition == -10000000 && state.ActualVelocity == -20000000);
    CHECK(state.ActualCurrent == -1000 && state.EstimatedTorque == -5000);
    CHECK(state.MotorTemperature == 0 && state.MosTemperature == 100);
    CHECK(state.ValidFlags == 0x7F && state.MotorFaultRaw == 0);
    frame.dlc = 7;
    CHECK(!encos_feedback(&frame, &motor.profile, &state));
    frame.dlc = 8;
    frame.data[0] = 0x40;
    CHECK(!encos_feedback(&frame, &motor.profile, &state));
    frame.data[0] = 0x28;
    CHECK(!encos_feedback(&frame, &motor.profile, &state));
    return true;
}

static bool parameters()
{
    MotorFixture motor;
    encos_frame_t frame{};
    CHECK(encos_query(36, 10, &frame));
    CHECK(frame.dlc == 2 && frame.data[0] == 0xEA && frame.data[1] == 36);
    CHECK(!encos_query(36, 0, &frame) && !encos_query(36, 11, &frame));
    CHECK(!encos_query(6, 1, &frame));
    CHECK(encos_parameter_query(1) == 5 && encos_parameter_query(2) == 0);
    CHECK(encos_parameter_query(0x101) == 1 && encos_parameter_query(0x127) == 39);
    CHECK(encos_parameter_query(0x128) == 0);
    mc_gain_t gain{};
    mc_kt_t kt{};
    CHECK(encos_parameter(3, &motor.basic, &motor.range, &gain, &kt, &frame));
    CHECK(frame.dlc == 4 && frame.data[0] == 0xC0 && frame.data[1] == 4);
    CHECK(frame.data[2] == 0 && frame.data[3] == 50);
    CHECK(!encos_parameter(1, &motor.basic, &motor.range, &gain, &kt, &frame));
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

int main(int argc, char **argv)
{
    const struct { const char *name; bool (*run)(); } cases[] = {
        {"abi", c_abi}, {"layout", layout}, {"profile", profile},
        {"commands", commands}, {"reject", rejected_commands},
        {"feedback", feedback}, {"parameters", parameters}, {"float", floats}
    };
    if (argc != 2)
        return 2;
    for (const auto &test : cases)
        if (std::strcmp(argv[1], test.name) == 0)
            return test.run() ? 0 : 1;
    return 2;
}
