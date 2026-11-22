#include "motor_instance.hpp"
#include "motor_driver_binding.hpp"
#include "encos_parameter_transactions.hpp"
#include "can_transport.h"
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

namespace motor = dcu::motor;
namespace parameters = motor::encos_parameters;

static unsigned canceled;
uint16_t dcu_can_bus_state(uint32_t) { return DCU_CAN_BUS_ERROR_ACTIVE; }
bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id) { CHECK(bus == 0 && id); ++canceled; return true; }

static dcu::motor::MotorConfiguration configuration()
{
    dcu::motor::MotorConfiguration result{};
    result.route = {1, -10000000, 10000000, 20000000, 10000, 100};
    result.encos = {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0};
    result.model_profile_id = 0x8000;
    result.power_query_period_ms = 100;
    return result;
}

static void validate()
{
    const auto valid = configuration();
    CHECK(motor::validate_motor_configuration(valid, 0));
    for (unsigned i = 0; i < 10; ++i) {
        auto invalid = valid;
        switch (i) {
        case 0: invalid.encos.profile.kp[1] = INFINITY; break;
        case 1: invalid.encos.profile.kd[0] = NAN; break;
        case 2: invalid.encos.profile.pos[0] = -3000; break;
        case 3: invalid.encos.profile.vel[1] = 3000; break;
        case 4: invalid.encos.profile.kt = NAN; break;
        case 5: invalid.encos.profile.kt = -0.1; break;
        case 6: invalid.route.current_limit_dA = 65536 + 100; break;
        case 7: invalid.encos.timeout_ms = 0; break;
        case 8: invalid.encos.frame_format = 1; break;
        case 9: invalid.model_profile_id = 0xFFFF; break;
        }
        CHECK(!motor::validate_motor_configuration(invalid, 0));
        CHECK(motor::configure_motor_driver(0, invalid, 0) == nullptr);
    }
    CHECK(motor::configure_motor_driver(0, valid, 0) != nullptr);
}

static void lifecycle()
{
    parameters::initialize({});
    dcu::motor::MotorInstance axis;
    auto config = configuration();
    CHECK(axis.bind(0, config, 0));
    config.route.can_id = 7;
    CHECK(axis.matches(1) && !axis.matches(7));
    motor::Command command{};
    command.mode = motor::ControlMode::Position;
    command.current_limit_dA = 100;
    CHECK(axis.accept(command, true, 1));
    command.control_bits = motor::Arm;
    CHECK(axis.accept(command, true, 1));
    for (uint8_t code = 1; code <= 4; ++code) {
        CHECK(parameters::start_query(0, 0, code, 1, 1, motor::ParameterDialect::EncosLegacy, 1, 0));
        parameters::mark_submitted(0, 1, code);
        parameters::finish_tx(0, {code, DCU_CAN_TX_COMPLETED, {}});
        dcu_can_frame_t reply{};
        reply.id = 1;
        reply.dlc = 6;
        reply.data[0] = 0xA0;
        reply.data[1] = code;
        if (code == 4) { reply.data[2] = 0x3F; reply.data[3] = 0xA0; }
        axis.consume_feedback(reply, 1, 1);
        CHECK(!parameters::inspect(0).busy);
    }
    CHECK(axis.query_ready(1));
    command.control_bits |= motor::StartMotion;
    CHECK(axis.accept(command, true, 1) && axis.active());
    dcu_can_frame_t control{};
    CHECK(axis.encode_control(control) && control.id == 1 && control.dlc == 8);
    axis.control_submitted(1, 5);
    CHECK(axis.finish_tx({5, DCU_CAN_TX_COMPLETED, {}}));
    dcu_can_frame_t reply{};
    reply.id = 1;
    reply.dlc = 8;
    reply.data[0] = 0x20;
    reply.data[1] = 0x80;
    reply.data[3] = 0x80;
    reply.data[4] = 8;
    reply.data[6] = 50;
    reply.data[7] = 70;
    axis.consume_feedback(reply, 1, 1);
    dcu::motor::MotorFeedback feedback{};
    dcu::motor::MotorDiagnostics diagnostics{};
    CHECK(axis.read_feedback(1, 20, feedback, diagnostics).online);
    CHECK(feedback.status_word & 8U);
    CHECK(feedback.sample.valid_fields == 63 && feedback.power_valid);
    CHECK(feedback.power_mW == 1250);
    CHECK(feedback.sample.motor_temperature_deciC == 0 && feedback.sample.mos_temperature_deciC == 100);
    CHECK(diagnostics.tx_enqueue_count == 1 && diagnostics.rx_valid_count == 5 && diagnostics.tx_complete_count == 1);
    axis.record_tx_drop();
    feedback.sample.motor_fault = 7;
    diagnostics.tx_drop_count = UINT32_MAX;
    axis.read_feedback(1, 20, feedback, diagnostics);
    CHECK(!feedback.sample.motor_fault && diagnostics.tx_drop_count == 1);
    CHECK((feedback.status_word & (16U | 1024U)) == (16U | 1024U));
    axis.acknowledge_publication();
    axis.read_feedback(1, 20, feedback, diagnostics);
    CHECK(!(feedback.status_word & (16U | 1024U)));
    axis.control_submitted(2, 6);
    axis.check_timeouts(22, false, 20);
    CHECK(!axis.active() && canceled == 1);
    CHECK(axis.finish_tx({6, DCU_CAN_TX_CANCELLED, {}}));
    CHECK(axis.read_feedback(22, 20, feedback, diagnostics).fault && feedback.bridge_fault == 4);
    CHECK(diagnostics.tx_complete_count == 1);
    CHECK(axis.bind(0, configuration(), 0));
    axis.read_feedback(21, 20, feedback, diagnostics);
    CHECK(diagnostics.tx_drop_count == 1 && diagnostics.rx_valid_count == 5);
    CHECK(!feedback.sample.valid_fields && !feedback.power_valid && !feedback.bridge_fault);
}

static void parameters_write()
{
    const uint8_t expected[][8] = {
        {0xC1, 2, 1}, {0xC0, 4, 0, 50}, {0xC0, 5, 0x12, 0x34, 0xFE, 0xDC},
        {0xC0, 6, 0x12, 0x34, 0xFE, 0xDC}, {0xC0, 7, 0x12, 0x34, 0xFE, 0xDC},
        {0xC0, 8, 0x12, 0x34, 0xFE, 0xDC}, {0xC0, 9, 0x12, 0x34, 0xFE, 0xDC},
        {0xC0, 10, 0x12, 0x34, 0xFE, 0xDC}, {0xC0, 11, 0, 100},
        {0xC0, 12, 0x12, 0x34, 0xFE, 0xDC}, {0xC0, 13, 0x12, 0x34, 0xFE, 0xDC},
        {0xC0, 14, 0x12, 0x34, 0xFE, 0xDC}, {0xC0, 15, 2},
        {0xCA, 16, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC},
    };
    for (uint16_t id = 2; id <= 15; ++id) {
        encos_parameter_values_t value{{0x1234, 0xFEDC, 0, 0}, 10};
        if (id == 2) value.words[0] = 1;
        if (id == 3) value.words[0] = 50;
        if (id == 10) value.words[0] = 100;
        if (id == 14) value.words[0] = 2;
        if (id == 15) value = {{0x123, 0x456, 0x789, 0xABC}, 10};
        dcu_can_frame_t frame{};
        frame.id = 8;
        frame.flags = 24;
        CHECK(encos_parameter(id, &value, &frame));
        CHECK(frame.id == 8 && frame.flags == 24);
        const unsigned size = id == 2 || id == 14 ? 3 : id == 3 || id == 10 ? 4 : id == 15 ? 8 : 6;
        CHECK(frame.dlc == size && std::memcmp(frame.data, expected[id - 2], sizeof(expected[0])) == 0);
    }
    for (unsigned i = 0; i < 9; ++i) {
        uint16_t id = 15;
        encos_parameter_values_t value{{1, 2, 3, 4}, 1};
        switch (i) {
        case 0: id = 1; break;
        case 1: id = 16; break;
        case 2: id = 2; value.words[0] = 2; break;
        case 3: id = 3; value.words[0] = 0; break;
        case 4: id = 10; value.words[0] = 0; break;
        case 5: id = 14; value.words[0] = 3; break;
        case 6: value.group = 0; break;
        case 7: value.group = 11; break;
        case 8: value.words[3] = 4096; break;
        }
        dcu_can_frame_t previous{};
        previous.id = 7;
        previous.data[3] = 123;
        auto frame = previous;
        CHECK(!encos_parameter(id, &value, &frame));
        CHECK(std::memcmp(&previous, &frame, sizeof(frame)) == 0);
    }
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "config")) validate();
    else if (!std::strcmp(argv[1], "lifecycle")) lifecycle();
    else if (!std::strcmp(argv[1], "parameters")) parameters_write();
    else return 2;
    return 0;
}
