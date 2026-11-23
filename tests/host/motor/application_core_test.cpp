#include "gateway_application.hpp"
#include "detail/gateway_configuration.hpp"
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
static uint16_t bus_states[DCU_DEVICE_CAN_BUS_COUNT];
static unsigned canceled;
static std::deque<dcu_can_frame_t> received[ENCOS_CHANNEL_COUNT];
static std::deque<dcu_can_eof_t> receive_eofs;
static std::vector<std::pair<unsigned, dcu_can_frame_t>> sent;
static std::deque<dcu_can_tx_event_t> tx_events[ENCOS_CHANNEL_COUNT];
static uint32_t last_tx_eof_us[ENCOS_CHANNEL_COUNT];
static bool tx_seen[ENCOS_CHANNEL_COUNT];
static dcu_can_tx_id_t tx_serial;
static std::vector<unsigned> sent_slots;
static dcu_can_channel_config_t applied_channels[] = {
    {2, 1000000, 5000000, true, false}, {0, 1000000, 0, false, false},
};
static const dcu_motor_device_profile_t classic_motor{
    "test_motor", 0, 1, 0x8000, 0, -10000000, 10000000, 20000000, 10000, 100,
    {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0}, 0, 0,
    DCU_MOTOR_PROTOCOL_ENCOS, nullptr, false, 0,
};

const dcu_can_channel_config_t *dcu_can_channel_config(unsigned bus)
{
    for (const auto &channel : applied_channels)
        if (channel.bus == bus) return &channel;
    return nullptr;
}
uint32_t dcu_time_now_ms(void) { return now_ms; }
uint32_t dcu_time_now_us(void) { return now_us; }
bool dcu_tx_inhibited(void) { return false; }
uint16_t dcu_can_bus_state(uint32_t bus) { return bus_states[bus]; }
uint32_t dcu_can_rx_drops(unsigned) { return 0; }
bool dcu_can_pending(unsigned, unsigned) { return false; }
bool dcu_can_drain_raw(unsigned) { return true; }
unsigned dcu_can_tx_free(unsigned) { return 8; }
bool dcu_can_tx_serial_ready(unsigned bus)
{
    return tx_events[bus].empty() && (!tx_seen[bus] || now_us - last_tx_eof_us[bus] >= DCU_CAN_TX_EOF_GAP_US);
}
uint32_t dcu_can_tx_progress(unsigned) { return 0; }
void dcu_can_cancel(unsigned) { ++canceled; }
bool dcu_can_cancel_tx(unsigned, dcu_can_tx_id_t id) { CHECK(id); ++canceled; return true; }
dcu_can_tx_id_t dcu_can_submit(unsigned bus, unsigned slot, const dcu_can_frame_t *frame)
{
    sent.emplace_back(bus, *frame);
    sent_slots.push_back(slot);
    last_tx_eof_us[bus] = now_us;
    tx_seen[bus] = true;
    dcu_can_tx_event_t event{++tx_serial, DCU_CAN_TX_COMPLETED, {}};
    if (bus == 0 && applied_channels[1].timestamp_eof) {
        event.eof.tick = now_us;
        event.eof.valid = true;
    }
    tx_events[bus].push_back(event);
    return tx_serial;
}
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    if (tx_events[bus].empty()) return false;
    *event = tx_events[bus].front();
    tx_events[bus].pop_front();
    return true;
}
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, dcu_can_rx_stamp_t *stamp)
{
    if (received[bus].empty()) return false;
    *frame = received[bus].front();
    received[bus].pop_front();
    *stamp = {now_ms, {}};
    if (!receive_eofs.empty()) {
        stamp->eof = receive_eofs.front();
        receive_eofs.pop_front();
    }
    return true;
}

static app::detail::Configuration configuration(bool motor_axis = true)
{
    app::detail::Configuration config{};
    config.board = {10, 20};
    config.device_count = motor_axis ? 1 : 0;
    for (auto &channel : config.channels)
        channel = {0, 0, 2, true};
    if (motor_axis) config.channels[0].role = 1;
    config.devices = motor_axis ? &classic_motor : nullptr;
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

static void initialize_core(const app::detail::Configuration &config, app::OperatingMode mode = app::OperatingMode::Motor)
{
    parameters::initialize({});
    CHECK(app::detail::initialize_fixture(config));
    CHECK(app::select_mode(mode));
    CHECK(app::map_layout(mode));
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

static void cyclic_mit_sweep(app::OperatingMode mode)
{
    dcu_motor_device_profile_t devices[] = {classic_motor, classic_motor, classic_motor, classic_motor};
    for (unsigned a = 0; a < 4; ++a) devices[a].can_id = a + 1;
    auto config = configuration();
    config.devices = devices;
    config.device_count = 4;
    applied_channels[1].timestamp_eof = true;
    initialize_core(config, mode);
    auto command = cycle_command();
    begin_session(command, app::OWNER_PDO);
    command.active_devices = command.board.enabled_devices = 0xf;
    CHECK(tick(&command).accepted); // Prepare all enabled axes before the ARM edge.
    for (auto &axis : command.devices) {
        axis.control_bits = motor::Arm;
        axis.mode = motor::ControlMode::Impedance;
        axis.current_limit_dA = 0;
    }
    CHECK(tick(&command).accepted);
    size_t next = 0;
    for (unsigned i = 0; i < 40; ++i) {
        query_replies(next);
        CHECK(tick(&command).accepted);
    }
    for (unsigned a = 0; a < 4; ++a) {
        motor::MotorFeedback feedback{};
        motor::MotorDiagnostics diagnostics{};
        CHECK(app::read_device(a, feedback, diagnostics) && (feedback.status_word & 4U));
        command.devices[a].control_bits |= motor::StartMotion;
    }
    const auto first = sent.size();
    const uint32_t start = now_us + 1000;
    const uint32_t deadline = start + 1600;
    app::set_cyclic_feedback_window(2000, deadline);
    const auto accepted = tick(&command);
    CHECK(accepted.accepted && app::cyclic_feedback_missing() == 0xf);
    CHECK(accepted.mirrored_devices == 0xf);
    CHECK(sent.size() == first + 1);
    now_us = start + 99;
    app::process(nullptr);
    CHECK(sent.size() == first + 1);
    for (unsigned j = 1; j < 4; ++j) {
        now_us = start + 100 * j;
        app::process(nullptr);
        CHECK(sent.size() == first + j + 1 && app::cyclic_feedback_missing() == 0xf);
    }
    const auto queue_reply = [&](unsigned sent_index, uint32_t eof, bool valid) {
        dcu_can_frame_t frame{};
        frame.id = sent[first + sent_index].second.id;
        frame.dlc = 8;
        frame.data[0] = 0x20;
        frame.data[1] = 0x80;
        frame.data[3] = 0x80;
        frame.data[4] = 8;
        received[0].push_back(frame);
        dcu_can_eof_t stamp{};
        stamp.tick = eof;
        stamp.valid = valid;
        receive_eofs.push_back(stamp);
    };
    queue_reply(0, start - 1, true);
    queue_reply(1, start + 101, false);
    now_us = start + 350;
    app::process(nullptr);
    CHECK(app::cyclic_feedback_missing() == 0xf);
    for (unsigned j = 4; j-- > 0;) queue_reply(j, start + 100 * j + 1, true);
    now_us = start + 450;
    app::process(nullptr);
    CHECK(app::cyclic_feedback_missing() == 0 && sent.size() == first + 4);
    app::Publication snapshot{};
    app::prepare_publication(snapshot);
    for (unsigned a = 0; a < 4; ++a)
        CHECK((snapshot.devices[a].feedback.status_word & 16U) && snapshot.devices[a].cyclic_feedback_count == 1);
    now_us = start + 1000;
    app::process(nullptr);
    CHECK(sent.size() == first + 4);
}

static void configuration_values()
{
    const auto valid = configuration();
    for (unsigned i = 0; i < 10; ++i) {
        auto invalid = valid;
        dcu_motor_device_profile_t devices[]{classic_motor, classic_motor};
        invalid.devices = devices;
        switch (i) {
        case 0: invalid.device_count = DCU_MOTOR_DEVICE_CAPACITY + 1; break;
        case 1: invalid.board.max_command_lease_ms = 1; break;
        case 2: invalid.board.feedback_timeout_ms = 21; break;
        case 3: invalid.channels[0].role = 4; break;
        case 4: invalid.channels[0].frame_format = 2; break;
        case 5: invalid.channels[0].enabled = false; break;
        case 6: invalid.channels[0].raw_rate_limit = 9; break;
        case 7: devices[0].bus = DCU_DEVICE_CAN_BUS_COUNT; break;
        case 8: devices[0].current_limit_dA = 65536 + 100; break;
        case 9: invalid.device_count = 2; break;
        }
        CHECK(!app::detail::initialize_fixture(invalid));
    }
    initialize_core(valid);
    CHECK(app::preop_access_allowed());
    CHECK(!app::select_mode(app::OperatingMode::Raw));
    CHECK(!app::select_mode(static_cast<app::OperatingMode>(3)));
    CHECK(!app::set_owner(static_cast<app::Owner>(3)));
    auto command = cycle_command();
    begin_session(command, app::OWNER_PDO);
    CHECK(!app::preop_access_allowed() && !app::detail::initialize_fixture(valid));
    CHECK(!app::select_mode(app::OperatingMode::MotorWithRaw));
    CHECK(app::select_mode(app::OperatingMode::Motor));
}

static dcu_motor_device_profile_t bjcx_on_bus2()
{
    dcu_motor_device_profile_t status_motor{};
    status_motor.bus = 2;
    status_motor.can_id = classic_motor.can_id;
    status_motor.protocol = DCU_MOTOR_PROTOCOL_BJCX;
    status_motor.type = "HRA40H-101";
    status_motor.status_poll_period_ms = 100;
    status_motor.status_reply_timeout_ms = 10;
    return status_motor;
}

static void production_profile()
{
    const auto status_motor = bjcx_on_bus2();
    dcu_motor_device_profile_t devices[]{status_motor, classic_motor};
    const dcu_production_profile_t valid{applied_channels, 2, devices, 2, 8, 15, {}, {}, nullptr, 0};
    for (unsigned i = 0; i < 18; ++i) {
        auto invalid = valid;
        dcu_can_channel_config_t channels[]{applied_channels[0], applied_channels[1]};
        invalid.channels = channels;
        auto bad_device = status_motor;
        invalid.devices = &bad_device;
        invalid.device_count = 1;
        switch (i) {
        case 0: invalid.channels = nullptr; break;
        case 1: invalid.channel_count = 0; break;
        case 2: invalid.channel_count = DCU_DEVICE_CAN_BUS_COUNT + 1; break;
        case 3: channels[0].bus = DCU_DEVICE_CAN_BUS_COUNT; break;
        case 4: channels[1] = channels[0]; break;
        case 5: channels[0].bus = 1; break; // Not applied to CAN.
        case 6: channels[0].fd = false; break;
        case 7: ++channels[0].data_bitrate_bps; break;
        case 8: invalid.devices = nullptr; break;
        case 9: bad_device.bus = 1; break;
        case 10: bad_device.status_reply_timeout_ms = 0; break;
        case 11: bad_device.protocol = DCU_MOTOR_PROTOCOL_UNSPECIFIED; break;
        case 12: bad_device.type = nullptr; break;
        case 13: bad_device.model_profile_id = 0x8000; break;
        case 14: channels[0].timestamp_eof = true; break; // Applied controller has no EOF timestamps.
        case 15: bad_device.type = "HRA99H-999"; break;
        case 16: bad_device.type = "EC-A7216-P2-22H"; break;
        case 17: bad_device.type = ""; break;
        }
        CHECK(!app::initialize_profile(invalid));
        CHECK(!app::channel_configuration(2).enabled && sent.empty());
    }
    for (unsigned i = 0; i < 7; ++i) {
        const dcu_passthrough_device_profile_t route{2, 1, DCU_MOTOR_PROTOCOL_ENCOS, "EC-A7216-P2-22H"};
        dcu_passthrough_device_profile_t passthrough[]{route, route};
        dcu_production_profile_t invalid{applied_channels, 2, nullptr, 0, 8, 15, {}, {}, passthrough, 1};
        switch (i) {
        case 0: invalid.passthrough_devices = nullptr; break;
        case 1: invalid.devices = &status_motor; invalid.device_count = 1; break;
        case 2: passthrough[0].type = "HRA55H-100"; break;
        case 3: invalid.passthrough_device_count = 2; break;
        case 4: passthrough[0].bus = 0; break; // Applied as classic CAN.
        case 5: passthrough[0].protocol = DCU_MOTOR_PROTOCOL_BJCX; break;
        case 6: passthrough[0].type = "EC-A0000-P2-22H"; break;
        }
        CHECK(!app::initialize_profile(invalid));
        CHECK(!app::channel_configuration(2).enabled && sent.empty());
    }
    parameters::initialize({});
    CHECK(app::initialize_profile(valid) && !app::initialize_profile(valid));
    CHECK(app::profile_policy().max_command_lease_ms == 8 && app::profile_policy().feedback_timeout_ms == 15);
    CHECK(app::channel_configuration(2).role == 1 && app::channel_configuration(2).frame_format == 1);
    CHECK(app::channel_configuration(0).role == 1 && app::channel_configuration(0).frame_format == 0);
    CHECK(!app::channel_configuration(1).enabled && !app::channel_configuration(3).enabled && sent.empty());
    CHECK(app::set_owner(app::OWNER_SDO));
    CHECK(!app::read_parameter(0, {1, 0, 0x101}, now_ms, true).accepted);
    CHECK(app::read_parameter(1, {1, 0, 0x101}, now_ms, true).accepted);
    tick();
    CHECK(sent.size() == 2);
    CHECK(sent[0].first == 0 && sent[0].second.id == classic_motor.can_id && sent[0].second.dlc == 2);
    CHECK(sent[1].first == 2 && sent[1].second.id == status_motor.can_id && sent[1].second.data[0] == 0x17);
}

static void interleaved_slots()
{
    // Profile IDs 1..7 are bus 2 slots 0..6. Slot 6 uses the legacy queue.
    dcu_motor_device_profile_t devices[8]{classic_motor};
    for (unsigned i = 1; i < 7; ++i) {
        devices[i] = bjcx_on_bus2();
        devices[i].can_id = i;
    }
    devices[7] = classic_motor;
    devices[7].bus = 2;
    devices[7].encos.frame_format = 1;
    devices[7].can_id = 7;
    parameters::initialize({});
    CHECK(app::initialize_profile({applied_channels, 2, devices, 8, 8, 15, {}, {}, nullptr, 0}));
    CHECK(app::set_owner(app::OWNER_SDO));
    tick(); // BJCX slot 0 has no reply.
    CHECK(sent.size() == 1 && sent[0].second.id == 1 && sent_slots[0] == 0);
    CHECK(app::read_parameter(7, {1, 0, 0x101}, now_ms, true).accepted);
    tick();
    CHECK(sent.size() == 2 && sent[1].second.id == 7 && sent_slots[1] == 6);
    for (unsigned slot = 1; slot < 6; ++slot) {
        tick(); // No replies to any of the previous queries on this bus.
        CHECK(sent.size() == slot + 2 && sent.back().second.id == slot + 1 && sent_slots.back() == slot);
    }
    CHECK(!app::owner_change_allowed() && parameters::inspect(2).device == 7);
}

static void query_before_sideband()
{
    const auto motor = bjcx_on_bus2();
    auto config = configuration(false);
    config.devices = &motor;
    config.device_count = 1;
    config.channels[0].role = 2;
    config.channels[2] = {1, 1, 0, true};
    initialize_core(config, app::OperatingMode::MotorWithRaw);
    CHECK(app::set_owner(app::OWNER_PDO));
    app::operational(true);
    auto command = cycle_command();
    command.board.enabled_channels = command.board.enabled_devices = command.board.control_bits = 0;
    command.channels[0].expected_role = 2;
    command.channels[2].expected_role = 1;
    bus_states[1] = bus_states[2] = DCU_CAN_BUS_OFF;
    CHECK(tick(&command).accepted && sent.empty());

    // Bus 1 stays off. Bus 2's newly eligible query must precede bus 0's Raw frame in this cycle.
    bus_states[2] = DCU_CAN_BUS_ERROR_ACTIVE;
    command.board.enabled_channels = command.board.control_bits = 1;
    auto &raw = command.raw[0];
    raw.sequence = raw.count = 1;
    raw.valid = true;
    raw.slots[0].frame.id = 0x321;
    raw.slots[0].frame.dlc = 1;
    const auto result = tick(&command);
    CHECK(result.accepted && result.raw_mirrored && sent.size() == 2);
    CHECK(sent[0].first == 2 && sent[0].second.id == motor.can_id && sent[0].second.data[0] == 0x17);
    CHECK(sent[1].first == 0 && sent[1].second.id == 0x321);
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
    else if (!std::strcmp(argv[1], "production_profile")) production_profile();
    else if (!std::strcmp(argv[1], "interleaved_slots")) interleaved_slots();
    else if (!std::strcmp(argv[1], "query_before_sideband")) query_before_sideband();
    else if (!std::strcmp(argv[1], "stop")) stop_and_snapshots();
    else if (!std::strcmp(argv[1], "cyclic_mit")) cyclic_mit_sweep(app::OperatingMode::Motor);
    else if (!std::strcmp(argv[1], "cyclic_mit_with_raw")) cyclic_mit_sweep(app::OperatingMode::MotorWithRaw);
    else if (!std::strcmp(argv[1], "sdo")) sdo_sequence();
    else if (!std::strcmp(argv[1], "owner")) ownership();
    else if (!std::strcmp(argv[1], "raw")) full_raw();
    else return 2;
    return 0;
}
