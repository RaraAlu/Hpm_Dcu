#include "gateway_application.hpp"
#include "encos_parameter_transactions.hpp"
#include "can_transport.h"
#include "realtime_port.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <tuple>
#include <vector>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

namespace app = dcu::app::gateway_application;
namespace parameters = dcu::motor::encos_parameters;
namespace motor = dcu::motor;
static uint32_t now_ms, now_us;
static unsigned canceled;
static std::deque<dcu_can_frame_t> received[ENCOS_CHANNEL_COUNT];
static std::vector<std::pair<unsigned, dcu_can_frame_t>> sent;
static std::deque<dcu_can_tx_event_t> tx_events[ENCOS_CHANNEL_COUNT];
static dcu_can_tx_id_t tx_serial;

uint32_t dcu_time_now_ms(void) { return now_ms; }
uint32_t dcu_time_now_us(void) { return now_us; }
uint16_t dcu_can_bus_state(uint32_t) { return DCU_CAN_BUS_ERROR_ACTIVE; }
uint32_t dcu_can_rx_drops(unsigned) { return 0; }
bool dcu_can_pending(unsigned) { return false; }
unsigned dcu_can_tx_free(unsigned) { return 8; }
uint32_t dcu_can_tx_progress(unsigned) { return 0; }
void dcu_can_cancel(unsigned) { ++canceled; }
bool dcu_can_cancel_tx(unsigned, dcu_can_tx_id_t id) { CHECK(id); ++canceled; return true; }
dcu_can_tx_id_t dcu_can_submit(unsigned bus, const dcu_can_frame_t *frame)
{
    sent.emplace_back(bus, *frame);
    tx_events[bus].push_back({++tx_serial, DCU_CAN_TX_COMPLETED});
    return tx_serial;
}
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    if (tx_events[bus].empty()) return false;
    *event = tx_events[bus].front();
    tx_events[bus].pop_front();
    return true;
}
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, uint32_t *stamp)
{
    if (received[bus].empty()) return false;
    *frame = received[bus].front();
    received[bus].pop_front();
    *stamp = now_ms;
    return true;
}

static app::Configuration configuration(bool motor_axis = true)
{
    app::Configuration config{};
    config.board = {10, 20};
    config.device_count = motor_axis ? 1 : 0;
    for (auto &channel : config.channels)
        channel = {0, 0, 2, true};
    if (motor_axis) {
        config.channels[0].role = 1;
        auto &axis = config.devices[0];
        axis.route = {1, -10000000, 10000000, 20000000, 10000, 100};
        axis.encos = {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0};
        axis.model_profile_id = 0x8000;
    }
    return config;
}

static app::CycleCommand cycle_command()
{
    app::CycleCommand command{};
    command.board = {0, 1, 1, 10, 1, true};
    for (auto &channel : command.channels) channel.period_us = 2000;
    command.channels[0] = {1, 1, true, 2000};
    command.active_devices = 1;
    command.devices[0].mode = motor::ControlMode::Position;
    command.devices[0].current_limit_dA = 50;
    return command;
}

static app::ProcessResult tick(app::CycleCommand *command = nullptr, uint32_t delta_ms = 1)
{
    now_ms += delta_ms;
    now_us = now_ms * 1000;
    if (command) ++command->board.sequence;
    return app::process(command);
}

static void initialize_core(const app::Configuration &config, app::OperatingMode mode = app::OperatingMode::Motor)
{
    parameters::initialize({});
    CHECK(app::initialize(config));
    CHECK(app::select_mode(mode));
}

static void begin_session(app::CycleCommand &command, app::Owner owner)
{
    CHECK(app::set_owner(owner));
    app::operational(true);
    command.board.control_bits = 0;
    command.board.enabled_devices = 0;
    CHECK(tick(&command).accepted);
    command.board.control_bits = 1;
    command.board.enabled_devices = 1;
    CHECK(tick(&command).accepted);
}

static void query_replies(size_t &next)
{
    while (next < sent.size()) {
        const auto [bus, request] = sent[next++];
        if (request.dlc != 2 || request.data[1] > 4) continue;
        dcu_can_frame_t response{};
        response.id = request.id;
        response.dlc = 6;
        response.data[0] = 0xA0;
        response.data[1] = request.data[1];
        received[bus].push_back(response);
    }
}

static void start_axis(app::CycleCommand &command)
{
    initialize_core(configuration());
    begin_session(command, app::OWNER_PDO);
    command.devices[0].control_bits = motor::Arm;
    CHECK(tick(&command).accepted);
    size_t next = 0;
    dcu::motor::MotorFeedback feedback{};
    dcu::motor::MotorDiagnostics diagnostic{};
    for (unsigned i = 0; i < 20; ++i) {
        query_replies(next);
        tick(&command);
        CHECK(app::read_device(0, feedback, diagnostic));
        if (feedback.status_word & 4U) break;
    }
    CHECK(feedback.status_word & 4U);
    command.devices[0].control_bits |= motor::StartMotion;
    const auto result = tick(&command);
    CHECK(result.accepted && result.mirrored_devices == 1);
    dcu_can_frame_t response{};
    response.id = 1;
    response.dlc = 8;
    response.data[0] = 0x20;
    response.data[1] = 0x80;
    response.data[3] = 0x80;
    response.data[4] = 8;
    received[0].push_back(response);
    tick(&command);
    CHECK(app::read_device(0, feedback, diagnostic));
    CHECK(feedback.status_word & 8U);
}

static void configuration_values()
{
    const auto valid = configuration();
    for (unsigned i = 0; i < 10; ++i) {
        auto invalid = valid;
        switch (i) {
        case 0: invalid.device_count = DCU_MOTOR_DEVICE_CAPACITY + 1; break;
        case 1: invalid.board.max_command_lease_ms = 1; break;
        case 2: invalid.board.feedback_timeout_ms = 21; break;
        case 3: invalid.channels[0].role = 4; break;
        case 4: invalid.channels[0].frame_format = 2; break;
        case 5: invalid.channels[0].enabled = false; break;
        case 6: invalid.channels[0].raw_rate_limit = 9; break;
        case 7: invalid.devices[0].bus = DCU_DEVICE_CAN_BUS_COUNT; break;
        case 8: invalid.devices[0].route.current_limit_dA = 65536 + 100; break;
        case 9:
            invalid.device_count = 2;
            invalid.devices[1] = invalid.devices[0];
            break;
        }
        CHECK(!app::initialize(invalid));
    }
    initialize_core(valid);
    CHECK(app::preop_access_allowed());
    CHECK(!app::select_mode(app::OperatingMode::Raw));
    CHECK(!app::select_mode(static_cast<app::OperatingMode>(3)));
    CHECK(!app::set_owner(static_cast<app::Owner>(3)));
    auto command = cycle_command();
    begin_session(command, app::OWNER_PDO);
    CHECK(!app::preop_access_allowed() && !app::initialize(valid));
    CHECK(!app::select_mode(app::OperatingMode::MotorWithRaw));
    CHECK(app::select_mode(app::OperatingMode::Motor));
}

static void stop_and_snapshots()
{
    auto command = cycle_command();
    start_axis(command);
    app::Publication snapshot{};
    app::prepare_publication(snapshot);
    CHECK(snapshot.online_devices == 1 && snapshot.online_channels == 1 && !snapshot.fault_devices);
    const auto sequence = snapshot.sequence;
    snapshot.sequence = UINT32_MAX;
    snapshot.board_status = UINT16_MAX;
    snapshot.devices[0].feedback.sample.current_cA = INT32_MAX;
    snapshot.devices[0].diagnostics.tx_drop_count = UINT32_MAX;
    app::prepare_publication(snapshot);
    CHECK(snapshot.sequence == sequence + 1 && snapshot.board_status == 3);
    CHECK(snapshot.devices[0].feedback.sample.current_cA != INT32_MAX && !snapshot.devices[0].diagnostics.tx_drop_count);
    CHECK(!app::owner_change_allowed() && !app::set_owner(app::OWNER_SDO));
    const auto accepted = snapshot.accepted_cycle_sequence;
    command.board.control_bits = 2;
    command.board.layout_valid = false;
    now_us += 100;
    const auto result = app::process(&command);
    CHECK(result.consumed && !result.accepted && !result.mirrored_devices);
    app::prepare_publication(snapshot);
    CHECK(snapshot.accepted_cycle_sequence == accepted && (snapshot.board_status & 64));
    CHECK(!(snapshot.devices[0].feedback.status_word & 8U) && canceled);
}

static void sdo_sequence()
{
    initialize_core(configuration());
    auto cycle = cycle_command();
    begin_session(cycle, app::OWNER_SDO);
    app::SdoCommand request{cycle.devices[0], 1, 10};
    auto result = app::commit_sdo_command(0, request, now_ms);
    CHECK(result.changed && result.result == motor::RESULT_OK && result.accepted_sequence == 1 && !result.mirror_command);
    request.sequence = 2;
    request.command.control_bits = motor::Arm;
    result = app::commit_sdo_command(0, request, now_ms);
    CHECK(result.result == motor::RESULT_OK && result.accepted_sequence == 2);
    request.sequence = 1;
    result = app::commit_sdo_command(0, request, now_ms);
    CHECK(result.changed && result.result == motor::RESULT_INVALID && result.accepted_sequence == 2);
    request.command.control_bits = motor::ResetFault;
    request.sequence = 3;
    CHECK(app::commit_sdo_command(0, request, now_ms).result == motor::RESULT_OK);
    request.command.control_bits = motor::Arm;
    request.sequence = 4;
    CHECK(app::commit_sdo_command(0, request, now_ms).result == motor::RESULT_OK);
    cycle.devices[0].control_bits = motor::Arm;
    tick(&cycle, 5);
    request.command.control_bits = motor::Stop;
    result = app::commit_sdo_command(0, request, now_ms);
    CHECK(!result.changed && result.accepted_sequence == 4);
    dcu::motor::MotorFeedback feedback{};
    dcu::motor::MotorDiagnostics diagnostic{};
    app::read_device(0, feedback, diagnostic);
    CHECK(feedback.status_word & motor::ArmedQueryOnly);
    tick(&cycle, 5);
    app::read_device(0, feedback, diagnostic);
    CHECK(feedback.bridge_fault == 3 && !(feedback.status_word & motor::ArmedQueryOnly));
}

static void ownership()
{
    initialize_core(configuration(false));
    CHECK(app::set_owner(app::OWNER_PDO));
    app::operational(true);
    app::operational(false);
    CHECK(app::owner() == app::OWNER_NONE);
    app::operational(true);
    CHECK(app::owner() == app::OWNER_PDO);
    app::operational(false);
    CHECK(app::set_owner(app::OWNER_SDO));
    app::operational(true);
    CHECK(app::owner() == app::OWNER_SDO);
    app::cycle_fault();
    CHECK(app::owner() == app::OWNER_NONE);
    app::operational(true);
    CHECK(app::owner() == app::OWNER_NONE);
}

static void full_raw()
{
    auto config = configuration(false);
    for (auto &channel : config.channels) channel.role = 2;
    initialize_core(config, app::OperatingMode::Raw);
    auto command = cycle_command();
    command.board.enabled_channels = 15;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &batch = command.raw[c];
        batch.kind = dcu::app::RawCanBatch::Kind::FullBus;
        batch.bus = c;
        batch.count = 1;
        batch.valid = true;
        batch.sequence = 1;
        batch.slots[0].bus = c;
        batch.slots[0].frame.id = 0x100 + c;
        batch.slots[0].frame.dlc = 1;
        batch.slots[0].frame.data[0] = 42 + c;
    }
    app::operational(true);
    command.board.enabled_devices = 0;
    command.board.control_bits = 0;
    CHECK(!tick(&command).raw_mirrored && sent.empty());
    command.board.control_bits = 1;
    CHECK(tick(&command).raw_mirrored && sent.size() == 4);
    for (unsigned c = 0; c < 4; ++c) {
        CHECK(std::get<0>(sent[c]) == c && std::get<1>(sent[c]).id == 0x100 + c);
        config.channels[c].role = 0;
    }
    CHECK(tick(&command).accepted && sent.size() == 4);
    for (auto &batch : command.raw) ++batch.sequence;
    CHECK(tick(&command, 2).raw_mirrored && sent.size() == 8);
    app::operational(false);
    CHECK(tick(&command).consumed && sent.size() == 8);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "configuration")) configuration_values();
    else if (!std::strcmp(argv[1], "stop")) stop_and_snapshots();
    else if (!std::strcmp(argv[1], "sdo")) sdo_sequence();
    else if (!std::strcmp(argv[1], "owner")) ownership();
    else if (!std::strcmp(argv[1], "raw")) full_raw();
    else return 2;
    return 0;
}
