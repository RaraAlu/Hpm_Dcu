#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <tuple>
#include <vector>
#include "gateway_application.hpp"
#include "detail/gateway_configuration.hpp"
#include "motor_instance.hpp"
#include "motor_driver_binding.hpp"
#include "can/raw_can_gateway.hpp"
#include "can/detail/raw_can_submit.hpp"
#include "can/cfg/gateway_schedule_policy.hpp"
#include "cfg/encos_runtime.hpp"
#include "encos_parameter_transactions.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_axis_codec.hpp"
#include "motor_adapter.hpp"
#include "motor_ethercat.h"
#include "dcu_loopback_evidence.hpp"
#include "production_init.h"
#include "can_transport.h"
#include "realtime_port.h"
extern "C" {
#include "digital_io.h"
#include "ecatslv.h"
#include "sdoserv.h"
}

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

UINT8 nAlStatus = STATE_PREOP;
extern "C" uint16_t dcu_ssc_station_address(void) { return 0x1234; }
TMC_ASSIGN gRxAssign{}, gTxAssign{};
alignas(4) static uint8_t ssc_txpdo[1072];

extern "C" void *dcu_ssc_txpdo_buffer(uint16_t *capacity)
{
    *capacity = sizeof(ssc_txpdo);
    return ssc_txpdo;
}

namespace parameters = dcu::motor::encos_parameters;
namespace motor_cfg = dcu::app::config::gateway_schedule;
namespace raw_cfg = dcu::app::config::raw_can;
using dcu::app::gateway_application::detail::raw_submit;
namespace motor_pdo = dcu::ethercat::motor_pdo;
namespace axis_codec = dcu::ethercat::motor_axis_codec;
namespace adapter = dcu::ethercat::motor_adapter;
using namespace dcu::app::gateway_application;
using namespace dcu::motor;
using namespace dcu::ethercat::motor_dictionary;

static uint32_t test_ms, test_us, cancellations;
static uint16_t test_input_size;
static std::deque<std::pair<dcu_can_frame_t, dcu_can_rx_stamp_t>> incoming[ENCOS_CHANNEL_COUNT];
static std::vector<std::pair<unsigned, dcu_can_frame_t>> sent;
static bool allow_send = true, allow_tx_events = true;
static uint16_t bus_states[ENCOS_CHANNEL_COUNT];
static unsigned service_cancellations[ENCOS_CHANNEL_COUNT];

static void select_owner(Owner owner)
{
    uint16_t value = owner;
    CHECK(MC_OD_Write(0x2003, 2, 1, &value, 0) == 0);
}

uint32_t dcu_time_now_ms(void) { return test_ms; }
uint32_t dcu_time_now_us(void) { return test_us; }
bool dcu_tx_inhibited(void) { return false; }
uint32_t dcu_can_rx_drops(unsigned) { return 0; }
uint16_t dcu_can_bus_state(uint32_t bus) { return bus_states[bus]; }
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, dcu_can_rx_stamp_t *stamp)
{
    if (incoming[bus].empty()) return false;
    *frame = incoming[bus].front().first;
    *stamp = incoming[bus].front().second;
    incoming[bus].pop_front();
    return true;
}
unsigned dcu_can_tx_free(unsigned) { return 8; }
uint32_t dcu_can_tx_progress(unsigned) { return 0; }
uint32_t dcu_can_raw_tx_cancelled(unsigned) { return 0; }
void dcu_can_cancel(unsigned) { ++cancellations; }
static dcu_can_tx_id_t test_tx_serial;
static dcu_can_eof_t test_tx_eof;
static std::deque<dcu_can_tx_event_t> tx_events[ENCOS_CHANNEL_COUNT];
static std::vector<dcu_can_tx_id_t> raw_ids[ENCOS_CHANNEL_COUNT];
static uint32_t last_tx_eof_us[ENCOS_CHANNEL_COUNT];
static bool tx_seen[ENCOS_CHANNEL_COUNT];
bool dcu_can_tx_serial_ready(unsigned bus)
{
    return tx_events[bus].empty() && (!tx_seen[bus] || test_us - last_tx_eof_us[bus] >= DCU_CAN_TX_EOF_GAP_US);
}
bool dcu_can_pending(unsigned bus, unsigned owner)
{
    return owner == DCU_CAN_DEVICE_ANY ? !tx_events[bus].empty() :
        owner == DCU_CAN_DEVICE_RAW && !raw_ids[bus].empty();
}
bool dcu_can_drain_raw(unsigned bus)
{
    for (auto id : raw_ids[bus]) dcu_can_cancel_tx(bus, id);
    return raw_ids[bus].empty();
}
bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id)
{
    CHECK(bus < ENCOS_CHANNEL_COUNT && id);
    ++service_cancellations[bus];
    ++cancellations;
    for (auto &event : tx_events[bus])
        if (event.id == id) event.status = DCU_CAN_TX_CANCELLED;
    return true;
}
dcu_can_tx_id_t dcu_can_submit(unsigned bus, unsigned owner, const dcu_can_frame_t *frame)
{
    if (!allow_send) return 0;
    sent.emplace_back(bus, *frame);
    last_tx_eof_us[bus] = test_us;
    tx_seen[bus] = true;
    tx_events[bus].push_back({++test_tx_serial, DCU_CAN_TX_COMPLETED, test_tx_eof});
    if (owner == DCU_CAN_DEVICE_RAW) raw_ids[bus].push_back(test_tx_serial);
    return test_tx_serial;
}
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    if (!allow_tx_events || tx_events[bus].empty()) return false;
    *event = tx_events[bus].front();
    tx_events[bus].pop_front();
    auto &ids = raw_ids[bus];
    ids.erase(std::remove(ids.begin(), ids.end(), event->id), ids.end());
    return true;
}

static void simulate_service_transmitted(unsigned bus, uint32_t now)
{
    const auto id = ++test_tx_serial;
    parameters::mark_submitted(bus, now, id);
    parameters::finish_tx(bus, {id, DCU_CAN_TX_COMPLETED, {}});
}

static const dcu_production_profile_t *fixture_profile;
static const dcu_can_channel_config_t test_channels[] = {
    {0, 1000000, 0, false, false}, {1, 1000000, 0, false, false},
    {2, 1000000, 0, false, false}, {3, 1000000, 0, false, false},
};
static const dcu_motor_device_profile_t test_device{
    "test_encos", 0, 1, 0x8000, 0, -10000000, 10000000, 20000000, 10000, 100,
    {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0}, 0, 0,
    DCU_MOTOR_PROTOCOL_ENCOS, nullptr, false, 0, false,
};
static dcu_production_profile_t test_profile{test_channels, ENCOS_CHANNEL_COUNT, &test_device, 1, 10, 20, {}, {}, nullptr, 0};
static dcu_motor_device_profile_t two_bus_devices[2];

extern "C" const dcu_production_profile_t *host_fixture_profile(void);
extern "C" const dcu_production_profile_t *host_mixed_profile(void);
extern "C" const dcu_production_profile_t *host_hra55h100_profile(void);
extern "C" const dcu_production_profile_t *host_five_axis_profile(void);
extern "C" const dcu_production_profile_t *dcu_production_profile(void)
{
    return fixture_profile ? fixture_profile : &test_profile;
}

const dcu_can_channel_config_t *dcu_can_channel_config(unsigned bus)
{
    const auto &profile = *dcu_production_profile();
    for (size_t i = 0; i < profile.channel_count; ++i)
        if (profile.channels[i].bus == bus) return &profile.channels[i];
    return nullptr;
}

void run_cycle()
{
    test_us = std::max(test_us + DCU_WORKER_PERIOD_US, test_ms * 1000U);
    motor_control_process();
}
void deliver(unsigned bus, const dcu_can_frame_t &frame, dcu_can_eof_t eof = {})
{
    incoming[bus].emplace_back(frame, dcu_can_rx_stamp_t{test_ms, eof});
    run_cycle();
}

static void select_wire_profile(unsigned profile)
{
    for (const bool tx : {false, true}) {
        const uint16_t index = tx ? 0x1c13 : 0x1c12;
        uint16_t indices[dcu::ethercat::pdo_layout::assignment_capacity], zero = 0;
        uint16_t count = motor_pdo::mapping_indices(profile, tx, indices);
        CHECK(MC_AssignWrite(index, 0, 1, &zero, 0) == 0);
        for (unsigned i = 0; i < count; ++i)
            CHECK(MC_AssignWrite(index, i + 1, 2, &indices[i], 0) == 0);
        CHECK(MC_AssignWrite(index, 0, 1, &count, 0) == 0);
    }
    uint16_t in = 0, out = 0;
    CHECK(motor_control_mapping(&in, &out) == 0);
    if (profile == 3) CHECK(in == 88 && out == 88);
    test_input_size = in;
}

static void request_test_layout(unsigned profile)
{
    const auto active = readiness();
    CHECK(request_layout(static_cast<OperatingMode>(profile), active.profile_id, active.revision, active.digest));
    CHECK(reset_layout());
    select_wire_profile(profile);
}

static void copy_txpdo(void *data)
{
    const auto profile = static_cast<unsigned>(mode());
    std::memcpy(data, ssc_txpdo, motor_pdo::layout::active().bytes(profile, true));
}

static void check_txpdo_bytes()
{
    const auto profile = static_cast<unsigned>(mode());
    const auto &layout = motor_pdo::layout::active();
    const size_t size = layout.bytes(profile, true);
    const auto *wire = ssc_txpdo;
    size_t offset = 0;
    const auto check = [&](const auto *mirror) {
        CHECK(offset + sizeof(*mirror) <= size);
        CHECK(!std::memcmp(wire + offset, mirror, sizeof(*mirror)));
        offset += sizeof(*mirror);
    };
    if (profile != 3) check(object<mc_board_tx_t>(0x6000));
    if (profile == 3) {
        check(object<mc_raw_t>(0x6401));
    } else if (profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            if (layout.buses & (1U << c)) check(object<canfd_bus_pdo_t>(0x6500 + 16 * c));
    } else {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            if (layout.buses & (1U << c)) check(object<mc_channel_tx_t>(0x6100 + c));
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
            if (layout.feedback_axes & (1U << a)) check(axis_feedback(a));
        if (profile == 1) check(object<mc_sideband_t>(0x6400));
    }
    CHECK(offset == size);
    if (std::getenv("DCU_DUMP_TXPDO"))
        CHECK(std::fwrite(wire, 1, size, stdout) == size);
}

static dcu::motor::MotorConfiguration axis_configuration()
{
    return {{1, -10000000, 10000000, 20000000, 10000, 100}, test_device.encos,
            0x8000, 0, 0, 0, 0, DCU_MOTOR_PROTOCOL_ENCOS};
}

bool bind_test_device(dcu::motor::MotorInstance &motor, unsigned device, uint8_t format = 0)
{
    auto configuration = axis_configuration();
    configuration.bus = device == 1 ? 1 : 0;
    return motor.bind(device, configuration, format);
}
static bool same_feedback(const dcu::motor::MotorFeedback &a, const dcu::motor::MotorFeedback &b)
{
    const auto x = axis_codec::feedback(a), y = axis_codec::feedback(b);
    return std::memcmp(&x, &y, sizeof(x)) == 0;
}
template <typename T> T read_sdo(uint16_t index, uint8_t sub)
{
    uint16_t storage[(sizeof(T) + 1) / 2]{};
    CHECK(MC_OD_Read(index, sub, sizeof(T), storage, 0) == 0);
    T value;
    std::memcpy(&value, storage, sizeof(value));
    return value;
}
static uint8_t position_query(unsigned slot, uint8_t operation)
{
    const uint16_t index = axis_object_base(slot) + 7;
    uint16_t selector = 0x101, op = operation, sequence[2]{1, 0};
    CHECK(MC_OD_Write(index, 1, 2, &selector, 0) == 0);
    CHECK(MC_OD_Write(index, 2, 1, &op, 0) == 0);
    return MC_OD_Write(index, 3, 4, sequence, 0);
}
void query_observation(dcu::motor::MotorInstance &axis, uint8_t code, uint32_t bits, uint32_t now = 1)
{
    CHECK(parameters::start_query(0, 0, code, 1, now, dcu::motor::ParameterDialect::EncosLegacy, 1, 0));
    simulate_service_transmitted(0, now);
    dcu_can_frame_t response{};
    response.id = 1;
    response.dlc = 6;
    response.data[0] = 0xA0;
    response.data[1] = code;
    for (unsigned i = 0; i < 4; ++i) response.data[2 + i] = bits >> (24 - 8 * i);
    axis.consume_feedback(response, now, now);
    CHECK(!parameters::inspect(0).busy);
}
void command_axis(uint16_t control, bool enabled = true, uint8_t expected_role = 1)
{
    uint8_t data[784]{};
    mc_board_rx_t board{};
    board.InterfaceVersion = MC_INTERFACE_VERSION;
    board.CycleSeq = object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq + 1;
    board.BoardControl = enabled ? 1 : 0;
    board.ChannelEnableMask = enabled ? 1 : 0;
    board.AxisEnableMask = enabled ? 1 : 0;
    board.CommandLeaseMs = 10;
    std::memcpy(data, &board, sizeof(board));
    mc_channel_rx_t channel{};
    channel.ChannelControl = 1;
    channel.ActiveMask = 1;
    channel.ExpectedRole = expected_role;
    channel.Reserved = 2000;
    std::memcpy(data + 32, &channel, sizeof(channel));
    mc_axis_rx_t axis{};
    axis.ControlWord = control;
    axis.Mode = 1;
    axis.CurrentLimit = 50;
    std::memcpy(data + 64, &axis, sizeof(axis));
    motor_control_output(data);
    run_cycle();
}
dcu_can_frame_t type1_feedback()
{
    dcu_can_frame_t frame{};
    frame.id = 1;
    frame.dlc = 8;
    const uint8_t data[] = {0x20, 0x80, 0, 0x80, 0x08, 0, 50, 50};
    std::memcpy(frame.data, data, sizeof(data));
    return frame;
}
void start_axis()
{
    CHECK(adapter::configured_device(0));
    select_owner(OWNER_PDO);
    motor_control_operational(true);
    command_axis(0, false);
    command_axis(0);
    command_axis(MC_AXIS_ARM);
    size_t read = 0;
    for (unsigned n = 0; n < 80; ++n) {
        run_cycle();
        while (read < sent.size()) {
            const auto [bus, request] = sent[read++];
            if (request.dlc != 2 || request.data[1] > 3) continue;
            dcu_can_frame_t response{};
            response.id = request.id;
            response.dlc = 6;
            response.data[0] = 0xA0;
            response.data[1] = request.data[1];
            deliver(bus, response);
        }
        motor_control_prepare_input();
        if (axis_feedback(0)->StatusWord & 4U) break;
    }
    CHECK(axis_feedback(0)->StatusWord & 4U);
    command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
    for (unsigned n = 0; n < 8 && !read_sdo<uint32_t>(0x2400, 1); ++n) run_cycle();
    CHECK(read_sdo<uint32_t>(0x2400, 1) != 0);
    deliver(0, type1_feedback());
    motor_control_prepare_input();
    CHECK(axis_feedback(0)->StatusWord & 8U);
}

struct RawPdoFixture {
    unsigned profile;
    mc_sideband_t sideband{};
    canfd_bus_pdo_t full[ENCOS_CHANNEL_COUNT]{};
    mc_board_rx_t board{};

    explicit RawPdoFixture(unsigned selected) : profile(selected)
    {
        select_wire_profile(profile);
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            full[c].cycle_seq = 1;
            full[c].valid_mask = 1;
            full[c].slot[0].can_id = 0x100 + c;
            full[c].slot[0].flags = 1;
            full[c].slot[0].dlc = 1;
            full[c].slot[0].data[0] = 42 + c;
        }
        sideband.ValidMask = sideband.SlotCount = 1;
        sideband.CanId0 = 0x100;
        sideband.Dlc0 = 1;
    }
    void submit(bool enabled = true, uint8_t channels = 15, bool advance = true)
    {
        uint8_t data[784]{};
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        if (advance) board.CycleSeq = object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq + 1;
        board.BoardControl = enabled ? 1 : 0;
        board.ChannelEnableMask = channels;
        board.CommandLeaseMs = 10;
        std::memcpy(data, &board, sizeof(board));
        if (profile == 1) {
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
                mc_channel_rx_t channel{};
                channel.ChannelControl = 1;
                channel.ExpectedRole = object<mc_channel_config_t>(0x2010 + c)->Role;
                channel.Reserved = 2000;
                if (motor_pdo::layout::active().buses & (1U << c))
                    motor_pdo::store(data, motor_pdo::layout::active().channel(false, c), channel);
            }
            motor_pdo::store(data, motor_pdo::layout::active().sideband(false), sideband);
        } else {
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
                if (motor_pdo::layout::active().buses & (1U << c))
                    motor_pdo::store(data, motor_pdo::layout::active().raw_bus(false, c), full[c]);
        }
        motor_control_output(data);
        run_cycle();
        motor_control_prepare_input();
    }
    void start()
    {
        motor_control_operational(true);
        submit(false);
        sent.clear();
    }
};

static constexpr uint32_t startup_epoch = 7;

static uint8_t switch_request(unsigned target, bool bad_digest = false)
{
    const auto profile = readiness();
    uint16_t storage[21]{};
    auto *p = reinterpret_cast<uint8_t *>(storage);
    p[0] = target;
    std::memcpy(p + 1, &profile.profile_id, 4);
    std::memcpy(p + 5, &profile.revision, 4);
    std::memcpy(p + 9, profile.digest, sizeof(profile.digest));
    if (bad_digest) p[40] ^= 1;
    return MC_OD_Write(0x2101, 1, 41, storage, 0);
}

static uint8_t switch_action(uint16_t action)
{
    return MC_OD_Write(0x2101, 2, 1, &action, 0);
}

static uint8_t write_limits(uint32_t packed)
{
    uint16_t data[2];
    std::memcpy(data, &packed, sizeof(data));
    return MC_OD_Write(0x2101, 5, sizeof(data), data, 0);
}

static void check_raw_state(RawState state)
{
    CHECK(raw_state() == state && read_sdo<uint8_t>(0x2101, 3) == static_cast<uint8_t>(state));
}

static void test_raw_switch(bool timeout, bool overflow)
{
    CHECK(boot_startup({1, 7, {1, 2, 3}, 1000, 5}, 17));
    RawPdoFixture input(1);
    check_raw_state(RawState::Armed);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    input.submit(false, 0);
    check_raw_state(RawState::Active);
    uint16_t action = 0;
    CHECK(MC_OD_Write(0x2101, 3, 1, &action, 0) == ABORTIDX_READ_ONLY_ENTRY);
    CHECK(!select_mode(OperatingMode::Motor) && !map_layout(OperatingMode::Motor));
    CHECK(MC_AssignWrite(0x1c12, 0, 1, &action, 0) != 0);
    CHECK(switch_request(0, true) != 0);
    check_raw_state(RawState::Active);
    allow_tx_events = false;
    ++input.sideband.BatchSeq;
    input.submit();
    CHECK(sent.size() == 1 && raw_ids[0].size() == 1);
    const auto before = cancellations;
    CHECK(switch_request(0) == 0 && cancellations == before + 1);
    check_raw_state(RawState::Draining);
    CHECK(tx_events[0].front().status == DCU_CAN_TX_CANCELLED);
    CHECK(switch_action(1) != 0);
    ++input.sideband.BatchSeq;
    input.submit();
    input.submit();
    CHECK(read_sdo<uint32_t>(0x2101, 4) == 1 && sent.size() == 1);
    if (timeout || overflow) {
        if (timeout) {
            // 保持板命令租期有效；只有透传撤销超过期限。
            for (unsigned i = 0; i < 20; ++i) { ++test_ms; input.submit(); }
        } else {
            for (unsigned i = 0; i < raw_cfg::drop_limit - 1; ++i) {
                ++input.sideband.BatchSeq;
                input.submit();
            }
            check_raw_state(RawState::Draining);
            ++input.sideband.BatchSeq;
            input.submit();
        }
        check_raw_state(RawState::Faulted);
        CHECK(sent.size() == 1 && readiness().state == DCU_GATEWAY_RUNNING);
        return;
    }
    allow_tx_events = true;
    input.submit();
    check_raw_state(RawState::Requested);
    CHECK(mode() == OperatingMode::MotorWithRaw && sent.size() == 1);
    nAlStatus = STATE_SAFEOP;
    motor_control_operational(false);
    CHECK(switch_action(1) != 0);
    nAlStatus = STATE_PREOP;
    CHECK(!map_layout(OperatingMode::Motor));
    CHECK(switch_action(1) == 0);
    uint16_t in = 9, out = 9;
    CHECK(motor_control_mapping(&in, &out) != 0 && !in && !out); // 旧分配仍不匹配。
    select_wire_profile(0);
    check_raw_state(RawState::Disabled);
    CHECK(test_input_size == 64 && read_sdo<uint32_t>(0x2100, 5) == 7);
    CHECK(switch_request(2) == 0 && switch_action(1) == 0);
    select_wire_profile(2);
    RawPdoFixture full(2);
    check_raw_state(RawState::Armed);
    CHECK(test_input_size == 544);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    full.submit(false, 0);
    full.submit();
    check_raw_state(RawState::Active);
    dcu_can_frame_t rx{};
    rx.id = 42;
    incoming[0].emplace_back(rx, dcu_can_rx_stamp_t{});
    full.submit();
    CHECK(object<canfd_bus_pdo_t>(0x6500)->valid_mask == 1);
    CHECK(switch_action(2) == 0);
    check_raw_state(RawState::Faulted);
    CHECK(switch_request(0) != 0 && switch_action(1) != 0);
    CHECK(!motor_pdo::load<canfd_bus_pdo_t>(ssc_txpdo, sizeof(mc_board_tx_t)).valid_mask);
    motor_control_prepare_input();
    check_txpdo_bytes();
    CHECK(!object<canfd_bus_pdo_t>(0x6500)->valid_mask);
    motor_control_operational(false);
    nAlStatus = STATE_PREOP;
    select_wire_profile(2);
    motor_control_operational(true);
    check_raw_state(RawState::Faulted);
    motor_control_operational(false);
    CHECK(switch_action(1) == 0);
    check_raw_state(RawState::Faulted);
    CHECK(!switch_request(2));
    select_wire_profile(2);
    motor_control_operational(true);
    check_raw_state(RawState::Active);
}

static void startup_business(uint32_t seq, uint16_t control, bool enabled = true, uint32_t reserved = 0,
                             uint16_t lease = 10)
{
    uint8_t wire[784]{};
    mc_board_rx_t board{};
    board.InterfaceVersion = MC_INTERFACE_VERSION;
    board.CycleSeq = seq;
    board.BoardControl = enabled ? 1 : 0;
    board.ChannelEnableMask = enabled ? 1U << 2 : 0;
    board.AxisEnableMask = enabled ? 1U << 14 : 0;
    board.CommandLeaseMs = lease;
    board.Reserved2 = reserved;
    std::memcpy(wire, &board, sizeof(board));
    mc_channel_rx_t channel{};
    channel.ChannelControl = 1;
    channel.ActiveMask = 1;
    channel.ExpectedRole = 1;
    channel.Reserved = 2000;
    motor_pdo::store(wire, motor_pdo::layout::active().channel(false, 2), channel);
    mc_axis_rx_t axis{};
    axis.ControlWord = control;
    axis.Mode = 1;
    axis.CurrentLimit = 50;
    const auto offset = motor_pdo::layout::active().axis(false, 14);
    if (offset != motor_pdo::layout::absent) motor_pdo::store(wire, offset, axis);
    motor_control_output(wire);
    run_cycle();
    motor_control_prepare_input();
}

static dcu_gateway_state_t startup_state()
{
    return static_cast<dcu_gateway_state_t>(read_sdo<uint16_t>(0x2002, 3) & 0xff);
}

static void serve_startup_reads(size_t &read, bool answer_all)
{
    while (read < sent.size()) {
        const auto [bus, request] = sent[read++];
        dcu_can_frame_t reply{};
        reply.id = request.id;
        reply.flags = request.flags;
        CHECK(reply.flags == (DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS));
        if (bus == 2 && request.id == 3) {
            CHECK(request.dlc == 2 && request.data[1] >= 1 && request.data[1] <= 3);
            reply.dlc = 6;
            reply.data[0] = 0xA0;
            reply.data[1] = request.data[1];
        } else {
            CHECK((bus == 0 && request.id == 3) || (bus == 2 && request.id == 36));
            CHECK(request.dlc == 1 && request.data[0] == 0x17);
            if (!answer_all && bus == 2) continue;
            reply.dlc = 10;
            reply.data[0] = 0x80;
            reply.data[13] = reply.data[14] = 50;
        }
        deliver(bus, reply);
    }
}

static void test_production_startup(bool journal_ok, bool switch_while_running)
{
    fixture_profile = host_mixed_profile();
    CHECK(motor_control_init());
    select_wire_profile(1);
    auto *board = object<mc_board_tx_t>(0x6000);
    CHECK(motor_control_boot_startup(journal_ok ? uint64_t{startup_epoch} << 32 : 0) == journal_ok);
    if (!journal_ok) {
        CHECK(startup_state() == DCU_GATEWAY_FAULT);
        CHECK(readiness().reason == DCU_GATEWAY_REASON_CONFIG_INVALID);
        motor_control_operational(true);
        startup_business(1, 0, false);
        startup_business(2, MC_AXIS_ARM | MC_AXIS_START_MOTION);
        CHECK(!board->AcceptedCycleSeq);
        CHECK(sent.empty() && readiness().session_id == 0);
        return;
    }
    uint16_t requested_owner = OWNER_PDO;
    CHECK(MC_OD_Write(0x2003, 2, 1, &requested_owner, 0) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    CHECK(MC_OD_Write(0x2002, 3, 2, &requested_owner, 0) == ABORTIDX_READ_ONLY_ENTRY);
    CHECK(read_sdo<uint32_t>(0x2002, 1) == startup_epoch);
    CHECK(read_sdo<uint16_t>(axis_object_base(14), 1) == 3);
    CHECK(read_sdo<uint16_t>(axis_object_base(14), 2) == 0x8000);
    CHECK(read_sdo<uint16_t>(axis_object_base(0), 2) == 0);
    CHECK(read_sdo<uint16_t>(axis_object_base(14) + 2, 2) == 500);
    CHECK(startup_state() == DCU_GATEWAY_PREPARING);
    size_t read = 0;
    for (; test_ms < 25; ++test_ms) {
        run_cycle();
        serve_startup_reads(read, false);
    }
    CHECK(startup_state() == DCU_GATEWAY_PREPARING && read >= 3); // One routed device has not answered.
    for (; test_ms < 350 && startup_state() != DCU_GATEWAY_READY; ++test_ms) {
        run_cycle();
        serve_startup_reads(read, true);
    }
    CHECK(startup_state() == DCU_GATEWAY_READY);
    const auto ready = read_sdo<uint32_t>(0x2002, 2);
    CHECK(ready != 0 && read_sdo<uint32_t>(0x2002, 4) == 0);
    startup_business(1, 0, false); // PREOP cannot accept even a neutral frame.
    CHECK(!board->AcceptedCycleSeq);
    motor_control_operational(true);
    startup_business(1000, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!board->AcceptedCycleSeq && startup_state() == DCU_GATEWAY_RUNNING);
    startup_business(1, 0, false, 1); // Previously used token bytes are reserved again.
    CHECK(!board->AcceptedCycleSeq);
    startup_business(1, 0, false);
    CHECK(board->AcceptedCycleSeq == 1 && readiness().session_id == 0);

    // A valid PDO drives the real ENCOS encoder through the same scheduler used in firmware.
    startup_business(2, 0);
    CHECK(board->AcceptedCycleSeq == 2);
    startup_business(3, MC_AXIS_ARM);
    for (unsigned i = 0; i < 240 && !(axis_feedback(14)->StatusWord & 4U); ++i) {
        run_cycle();
        serve_startup_reads(read, true);
        motor_control_prepare_input();
    }
    CHECK(axis_feedback(14)->StatusWord & 4U);
    const auto first_business = sent.size();
    startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    for (unsigned i = 0; i < 16; ++i) run_cycle();
    CHECK(board->AcceptedCycleSeq == 4);
    CHECK(std::any_of(sent.begin() + first_business, sent.end(), [](const auto &tx) {
        return tx.first == 2 && tx.second.id == 3 && tx.second.dlc == 8;
    }));

    auto cyclic_feedback = type1_feedback();
    cyclic_feedback.id = 3;
    cyclic_feedback.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    deliver(2, cyclic_feedback);
    motor_control_prepare_input();
    CHECK((axis_feedback(14)->StatusWord & 8U) && (axis_feedback(14)->ValidFlags & 7U) == 7U);

    if (switch_while_running) {
        const auto cancels = cancellations;
        CHECK(switch_request(2) != 0); // OP 内不能把已配置的电机切离调度。
        CHECK(switch_request(0) == 0);
        check_raw_state(RawState::Requested);
        CHECK(cancellations == cancels && owner() == OWNER_PDO);
        for (unsigned seq : {5U, 6U}) {
            if (seq == 6) CHECK(switch_action(2) == 0);
            const auto first = sent.size();
            test_ms += 2;
            startup_business(seq, MC_AXIS_ARM | MC_AXIS_START_MOTION);
            CHECK(board->AcceptedCycleSeq == seq && (axis_feedback(14)->StatusWord & 8U));
            for (unsigned n = 0; n < 2000 / DCU_WORKER_PERIOD_US + 2; ++n) run_cycle();
            CHECK(std::any_of(sent.begin() + first, sent.end(), [](const auto &tx) {
                return tx.first == 2 && tx.second.id == 3 && tx.second.dlc == 8;
            }));
        }
        check_raw_state(RawState::Faulted);
        CHECK(!object<mc_sideband_t>(0x6400)->ValidMask && cancellations == cancels);
        return;
    }

    const auto before_drop = cancellations;
    check_raw_state(RawState::Active);
    motor_control_operational(false);
    check_raw_state(RawState::Faulted);
    CHECK(cancellations > before_drop);
    const size_t after_drop = read = sent.size();
    startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!board->AcceptedCycleSeq);
    motor_control_operational(true);
    for (unsigned i = 0; i < 350 && startup_state() != DCU_GATEWAY_RUNNING; ++i) {
        ++test_ms;
        startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
        serve_startup_reads(read, true);
        CHECK(!board->AcceptedCycleSeq);
    }
    CHECK(startup_state() == DCU_GATEWAY_RUNNING && read > after_drop);
    check_raw_state(RawState::Faulted);
    CHECK(read_sdo<uint32_t>(0x2002, 2) > ready && read_sdo<uint32_t>(0x2002, 4) == 0);
    startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!board->AcceptedCycleSeq);
    serve_startup_reads(read, true); // No control frame before the new neutral baseline.
    startup_business(5, 0, false);
    CHECK(board->AcceptedCycleSeq == 5 && readiness().session_id == 0);
    startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(board->AcceptedCycleSeq == 5);
    startup_business(6, 0);
    CHECK(board->AcceptedCycleSeq == 6);
    test_ms += 10;
    startup_business(7, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!board->AcceptedCycleSeq && readiness().session_id == 0);
    serve_startup_reads(read, true);
}

static void test_dynamic_policy(const char *encoded)
{
    using namespace dcu::app::profile_update;
    constexpr uint16_t object_index = 0x2100;
    uint16_t wire[36 / sizeof(uint16_t)]{};
    auto *file = std::fopen(encoded, "rb");
    CHECK(file && std::fread(wire, 1, sizeof(wire), file) == sizeof(wire) && std::fgetc(file) == EOF);
    std::fclose(file);
    test_profile.devices = nullptr;
    test_profile.device_count = 0;
    test_profile.readiness = {1, 1, {}, 1000, 5};
    CHECK(motor_control_init());
    select_wire_profile(1);
    CHECK(motor_control_boot_startup(uint64_t{startup_epoch} << 32));
    run_cycle();
    const auto result = [] { return static_cast<Status>(read_sdo<uint16_t>(object_index, 3)); };
    const auto offer = [&](unsigned bytes = sizeof(wire)) { return MC_OD_Write(object_index, 1, bytes, wire, 0); };
    const auto commit = [](uint16_t revision) {
        uint16_t words[2] = {revision, 0};
        return MC_OD_Write(object_index, 2, sizeof(words), words, 0);
    };
    const auto active = [](uint32_t revision, uint16_t lease, uint16_t feedback, uint32_t ttl) {
        CHECK(read_sdo<uint32_t>(object_index, 5) == revision);
        CHECK(read_sdo<uint16_t>(0x2001, 2) == lease && read_sdo<uint16_t>(0x2001, 3) == feedback);
        CHECK(read_sdo<uint32_t>(object_index, 7) == ttl);
    };
    CHECK(startup_state() == DCU_GATEWAY_READY && result() == Status::Idle);
    const auto generation = read_sdo<uint32_t>(0x2002, 2);
    CHECK(!offer() && result() == Status::Staged && read_sdo<uint32_t>(object_index, 6) == 2);
    active(1, 10, 20, 1000);
    CHECK(offer(sizeof(wire) - 2) == ABORTIDX_PARAM_LENGTH_ERROR && result() == Status::Aborted);
    CHECK(commit(2) == ABORTIDX_VALUE_EXCEEDED);
    CHECK(!offer() && result() == Status::Staged);
    wire[17] ^= 1;
    CHECK(offer() == ABORTIDX_VALUE_EXCEEDED && result() == Status::Invalid);
    CHECK(!read_sdo<uint32_t>(object_index, 6) && commit(2) == ABORTIDX_VALUE_EXCEEDED);
    wire[17] ^= 1;
    CHECK(!offer());
    test_ms += 5000;
    CHECK(result() == Status::Aborted && commit(2) == ABORTIDX_VALUE_EXCEEDED);
    active(1, 10, 20, 1000);
    CHECK(!offer());
    nAlStatus = STATE_SAFEOP;
    motor_control_discard_profile();
    CHECK(commit(2) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    nAlStatus = STATE_PREOP;
    CHECK(result() == Status::Aborted && commit(2) == ABORTIDX_VALUE_EXCEEDED);
    CHECK(!offer() && !commit(2) && result() == Status::Applied);
    active(2, 8, 15, 750);
    CHECK(!commit(2)); // A lost response can be retried without re-preparing.
    CHECK(startup_state() == DCU_GATEWAY_PREPARING);
    run_cycle();
    CHECK(startup_state() == DCU_GATEWAY_READY && read_sdo<uint32_t>(0x2002, 2) > generation);
    CHECK(offer() == ABORTIDX_VALUE_EXCEEDED && result() == Status::Stale);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    CHECK(offer() == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    CHECK(commit(2) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    startup_business(1, 0, false);
    CHECK(!object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq);
    startup_business(1, 0, false, 0, 8);
    CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1 && readiness().session_id == 0 && sent.empty());
    motor_control_operational(false);
    active(2, 8, 15, 750);
}

static void test_production_raw(bool authorize)
{
    fixture_profile = host_hra55h100_profile();
    CHECK(motor_control_init() && motor_control_boot_startup(17));
    select_wire_profile(1);
    CHECK(motor_control_verify_contract());
    auto wrong_profile = *fixture_profile;
    wrong_profile.readiness.digest[0] ^= 1;
    fixture_profile = &wrong_profile;
    CHECK(!motor_control_verify_contract());
    fixture_profile = host_hra55h100_profile();
    ++gTxAssign.aEntries[1];
    CHECK(!motor_control_verify_contract());
    --gTxAssign.aEntries[1];
    CHECK(adapter::configured_device(21)->can_id == 2);
    CHECK(read_sdo<uint32_t>(0x2101, 5) == 0);
    motor_control_operational(true);
    nAlStatus = STATE_OP;
    CHECK(write_limits(1U << 24) && switch_request(3));
    motor_control_operational(false);
    nAlStatus = STATE_SAFEOP;
    CHECK(write_limits(1U << 24) && switch_request(3));
    nAlStatus = STATE_PREOP;
    CHECK(!switch_action(1)); // 非协商退 OP 后先解锁，再请求。
    CHECK(write_limits(9U << 24) && write_limits((1U << 24) | 1));
    const uint32_t quota = authorize ? 1U << 24 : 0;
    if (authorize) CHECK(!write_limits(quota));
    CHECK(read_sdo<uint32_t>(0x2101, 5) == quota);
    CHECK(!switch_request(3) && !switch_action(1));
    select_wire_profile(3);
    CHECK(gRxAssign.u16SubIndex0 == 1 && gTxAssign.u16SubIndex0 == 1);
    CHECK(gRxAssign.aEntries[0] == 0x1641 && gTxAssign.aEntries[0] == 0x1A42);
    CHECK(test_input_size == 88);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    mc_raw_t wire{};
    const auto submit = [&] { ++wire.BatchSeq; motor_control_output(&wire); run_cycle(); };
    submit();
    wire.ValidMask = wire.SlotCount = 1;
    wire.Channel = 3;
    wire.CanId = 0x321;
    wire.FrameFlags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    wire.Dlc = 15;
    uint8_t payload[64];
    for (unsigned i = 0; i < sizeof(payload); ++i) payload[i] = 0xc7 ^ i;
    std::memcpy(reinterpret_cast<uint8_t *>(&wire) + 24, payload, 64);
    submit();
    submit(); // 同一 2 ms 窗口的第二帧。
    CHECK(sent.size() == unsigned(authorize) && raw_drops() == (authorize ? 1U : 2U));
    if (authorize) {
        CHECK(sent[0].first == 3 && sent[0].second.id == 0x321 && sent[0].second.dlc == 15);
        CHECK(!std::memcmp(sent[0].second.data, payload, 64));
        deliver(3, sent[0].second);
        motor_control_prepare_input();
        CHECK(ssc_txpdo[4] == 1 && ssc_txpdo[20] == 3 && ssc_txpdo[22] == 15 && ssc_txpdo[23] == 0x34);
        CHECK(!std::memcmp(ssc_txpdo + 24, payload, 64));
        test_ms += 2;
        wire.CanId = 2;
        submit();
        CHECK(sent.size() == 1 && raw_drops() == 2);
        wire.CanId = 0x321;
        allow_tx_events = false;
        submit();
        CHECK(sent.size() == 2);
    } else {
        deliver(3, {0x321, 15, wire.FrameFlags, {}});
        motor_control_prepare_input();
        CHECK(ssc_txpdo[4] == 0);
    }
    const auto count = sent.size();
    CHECK(!switch_request(1));
    motor_control_operational(false);
    nAlStatus = STATE_PREOP;
    if (authorize) {
        CHECK(write_limits(0) && read_sdo<uint32_t>(0x2101, 5) == quota);
        allow_tx_events = true;
        run_cycle();
    }
    CHECK(!write_limits(0) && !switch_action(1));
    select_wire_profile(1);
    CHECK(test_input_size == 128 && gRxAssign.u16SubIndex0 == 5 && gTxAssign.u16SubIndex0 == 5);
    CHECK(object<mc_board_config_t>(0x2001)->AxisConfiguredMask == (1U << 21));
    CHECK(object<mc_channel_config_t>(0x2013)->Role == 1);
    ++test_ms;
    run_cycle();
    CHECK(startup_state() == DCU_GATEWAY_READY);
    motor_control_operational(true);
    nAlStatus = STATE_OP;
    uint8_t motor_wire[784]{};
    const auto motor_cycle = [&](unsigned seq, unsigned control) {
        test_ms += 2;
        motor_control_set_neutral_window(2000000, test_ms * 1000U + 900U);
        mc_board_rx_t b{};
        b.InterfaceVersion = MC_INTERFACE_VERSION;
        b.CycleSeq = seq;
        b.CommandLeaseMs = 10;
        b.BoardControl = control ? 1 : 0;
        b.ChannelEnableMask = control ? 1U << 3 : 0;
        b.AxisEnableMask = control ? 1U << 21 : 0;
        mc_axis_rx_t axis{};
        axis.Mode = 4;
        axis.ControlWord = control;
        motor_pdo::store(motor_wire, 0, b);
        motor_pdo::store(motor_wire, 40, mc_channel_rx_t{1, 1, 1, 2000});
        motor_pdo::store(motor_wire, 48, axis);
        motor_control_output(motor_wire);
        run_cycle();
        motor_control_prepare_input();
    };
    motor_cycle(99, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq && sent.size() == count);
    motor_cycle(1, 0);
    motor_cycle(2, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 2 && sent.size() == count);
    motor_cycle(3, MC_AXIS_ARM);
    motor_cycle(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(sent.size() == count + 1 && sent.back().first == 3 && sent.back().second.id == 2);
    CHECK(sent.back().second.dlc == 10 && sent.back().second.data[0] == 0x11);
}

static void test_registered_passthrough()
{
    fixture_profile = host_hra55h100_profile();
    CHECK(motor_control_init() && motor_control_boot_startup(17));
    select_wire_profile(1);
    CHECK(!adapter::configured_device(14) && adapter::configured_device(21)->can_id == 2);
    CHECK(read_sdo<uint32_t>(0x2012, 3) == 1000000 && read_sdo<uint8_t>(0x2012, 7) == 0);
    CHECK(read_sdo<uint32_t>(0x2101, 5) == 0);
    ++test_ms;
    run_cycle();
    CHECK(sent.empty());
    CHECK(!write_limits(1U << 16) && !switch_request(3));
    uint16_t owner = 1U << 2;
    CHECK(MC_OD_Write(0x2101, 6, 1, &owner, 0) != 0); // Not a motion bus to hand over.
    CHECK(!switch_action(1));
    select_wire_profile(3);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    mc_raw_t wire{};
    const auto submit = [&] { test_ms += 2; ++wire.BatchSeq; motor_control_output(&wire); run_cycle(); };
    submit();
    wire.ValidMask = wire.SlotCount = 1;
    wire.Channel = 2;
    wire.CanId = 0x7ff;
    wire.FrameFlags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    wire.Dlc = 6;
    wire.Data_1 = 1;
    wire.Data_3 = 4;
    wire.Data_5 = 4;
    submit();
    const uint8_t change_id[]{0, 1, 0, 4, 0, 4};
    CHECK(sent.size() == 1 && sent[0].first == 2 && sent[0].second.id == 0x7ff &&
          sent[0].second.flags == wire.FrameFlags && sent[0].second.dlc == wire.Dlc &&
          !std::memcmp(sent[0].second.data, change_id, sizeof(change_id)));
    const uint8_t ack[]{0, 4, 1, 4};
    deliver(2, {0x7ff, 4, wire.FrameFlags, {0, 4, 1, 4}});
    motor_control_prepare_input();
    CHECK(ssc_txpdo[4] == 1 && ssc_txpdo[16] == 0xff && ssc_txpdo[17] == 7 &&
          ssc_txpdo[20] == 2 && ssc_txpdo[21] == wire.FrameFlags && ssc_txpdo[22] == sizeof(ack) &&
          !std::memcmp(ssc_txpdo + 24, ack, sizeof(ack)));

    wire.CanId = 4;
    wire.Dlc = 2;
    wire.Data_0 = 0xe0;
    submit();
    CHECK(sent.size() == 2 && sent[1].first == 2 && sent[1].second.id == 4 &&
          sent[1].second.data[0] == 0xe0 && sent[1].second.data[1] == 1);
    deliver(2, {4, 6, wire.FrameFlags, {0xa0, 1, 0xc3, 0x0b, 0x6c, 0x26}});
    motor_control_prepare_input();
    CHECK(ssc_txpdo[4] == 1 && ssc_txpdo[16] == 4 && ssc_txpdo[24] == 0xa0);

    wire.CanId = 0x1abcdeff;
    wire.FrameFlags |= DCU_CAN_FLAG_EXTENDED;
    submit();
    CHECK(sent.size() == 3 && sent[2].first == 2 && sent[2].second.id == wire.CanId &&
          sent[2].second.flags == wire.FrameFlags);
    deliver(2, {wire.CanId, 2, wire.FrameFlags, {0x12, 0x34}});
    motor_control_prepare_input();
    const uint8_t extended_id_le[]{0xff, 0xde, 0xbc, 0x1a};
    CHECK(ssc_txpdo[4] == 1 && !std::memcmp(ssc_txpdo + 16, extended_id_le, sizeof(extended_id_le)) &&
          ssc_txpdo[21] == wire.FrameFlags && ssc_txpdo[24] == 0x12 && ssc_txpdo[25] == 0x34);
    CHECK(sent.size() == 3 && !read_sdo<uint8_t>(0x2101, 6) && !raw_drops());
}

static void test_hot_topology(bool timeout)
{
    constexpr uint32_t full = (15U << 14) | (1U << 21), subset = full ^ (1U << 15);
    const auto rx = gRxAssign, tx = gTxAssign;
    const auto state = readiness();
    const auto stage = [&](uint32_t axes, uint32_t generation, bool valid = true, uint32_t buses = 12) {
        uint16_t words[44]{};
        auto *p = reinterpret_cast<uint8_t *>(words);
        std::memcpy(p, &state.boot_id, 8);
        std::memcpy(p + 8, &state.revision, 4);
        std::memcpy(p + 12, &generation, 4);
        std::memcpy(p + 16, &buses, 4);
        std::memcpy(p + 20, &axes, 4);
        std::memcpy(p + 24, object<mc_layout_identity_t>(0x2102)->Sha256, 32);
        std::memcpy(p + 56, state.digest, 32);
        if (!valid) p[25] ^= 1;
        return MC_OD_Write(0x2103, 1, sizeof(words), words, 0);
    };
    const auto commit = [](uint32_t gen) {
        uint16_t words[2];
        std::memcpy(words, &gen, 4);
        return MC_OD_Write(0x2103, 6, 4, words, 0);
    };
    const auto assign = [](unsigned idx, const TMC_ASSIGN &catalogue, uint16_t omit, uint16_t omit2 = 0) {
        uint16_t n = 0;
        CHECK(!MC_AssignWrite(idx, 0, 1, &n, 0));
        for (unsigned i = 0; i < catalogue.u16SubIndex0; ++i) {
            uint16_t block = catalogue.aEntries[i];
            if (block != omit && block != omit2) CHECK(!MC_AssignWrite(idx, ++n, 2, &block, 0));
        }
        CHECK(!MC_AssignWrite(idx, 0, 1, &n, 0));
    };
    CHECK(read_sdo<uint32_t>(0x2103, 4) == full);
    CHECK(stage(subset, 1) && stage(subset, 0, false) && stage(subset | 1, 0) && stage(full, 0, true, 4));
    nAlStatus = STATE_OP;
    CHECK(stage(subset, 0));
    nAlStatus = STATE_PREOP;
    tx_events[2].push_back({++test_tx_serial, DCU_CAN_TX_CANCELLED, {}});
    allow_tx_events = false;
    CHECK(!stage(subset, 0) && raw_state() == RawState::Draining);
    CHECK(switch_action(1) && commit(1) && switch_request(0));
    if (timeout) {
        test_ms += 21;
        run_cycle();
        CHECK(raw_state() == RawState::Faulted && topology_pending() && commit(1));
        CHECK(!motor_control_commit_topology(1) && read_sdo<uint32_t>(0x2103, 2) == 0);
        CHECK(sent.empty());
    }
    allow_tx_events = true;
    if (timeout) { CHECK(!switch_action(1)); CHECK(!stage(subset, 0)); }
    run_cycle();
    CHECK(!switch_action(1) && commit(2) && commit(1));
    CHECK(read_sdo<uint32_t>(0x2103, 4) == full);
    assign(0x1c12, rx, 0x162f);
    assign(0x1c13, tx, 0x1a2e); // Same width, wrong physical axis.
    CHECK(commit(1));
    uint16_t in = 0, out = 0;
    assign(0x1c13, tx, 0x1a2f);
    CHECK(motor_control_mapping(&in, &out) != 0 && read_sdo<uint32_t>(0x2103, 2) == 0);
    uint8_t old_image[216]{};
    mc_board_rx_t stale{};
    stale.CycleSeq = 99; stale.BoardControl = 1; stale.CommandLeaseMs = 10;
    stale.InterfaceVersion = MC_INTERFACE_VERSION; stale.AxisEnableMask = full;
    motor_pdo::store(old_image, 0, stale);
    motor_control_output(old_image);
    CHECK(!commit(1) && commit(1) && !motor_control_mapping(&in, &out) && in == 224 && out == 192);
    motor_control_process();
    CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 0 && !cyclic_feedback_required());
    CHECK(read_sdo<uint32_t>(0x2103, 4) == subset && !adapter::configured_device(15) &&
          adapter::configured_device(16)->can_id == 3 && read_sdo<uint32_t>(0x2103, 2) == 1);
    CHECK(!topology_pending() && owner() == OWNER_NONE);
    for (unsigned i = 0; i < 50; ++i) { ++test_ms; run_cycle(); }
    CHECK(!sent.empty());
    for (const auto &submit : sent) CHECK(submit.first == 2 && submit.second.id != 2 && submit.second.dlc == 2);
    CHECK(stage(full, 0) && !stage(full, 1) && !switch_action(1));
    assign(0x1c12, rx, 0);
    assign(0x1c13, tx, 0);
    CHECK(!commit(2) && !motor_control_mapping(&in, &out) && in == 256 && out == 216);
    CHECK(adapter::configured_device(15)->can_id == 2 && readiness().boot_id == state.boot_id &&
          read_sdo<uint32_t>(0x2103, 2) == 2 && read_sdo<uint32_t>(0x2103, 4) == full);
    CHECK(!stage(15U << 14, 2, true, 4) && !switch_action(1));
    assign(0x1c12, rx, 0x1604, 0x1635);
    assign(0x1c13, tx, 0x1a04, 0x1a35);
    CHECK(!commit(3) && !motor_control_mapping(&in,&out) && in == 216 && out == 184);
    CHECK(!channel_configuration(3).enabled && !adapter::configured_device(21));
}

static void test_encos_four_axis_production_pdo(const char *scenario)
{
    auto profile = *host_five_axis_profile();
    const auto is = [&](const char *s) { return !std::strcmp(scenario, s); };
    const bool endurance = is("encos_four_axis_production_pdo"), regular = is("regular_limits");
    dcu_motor_device_profile_t devices[5];
    CHECK(profile.device_count == 5);
    std::copy_n(profile.devices, 5, devices);
    for (unsigned n = 1; n <= 4; ++n) {
        CHECK(devices[n].zero_mit_loopback && devices[n].bus == 2 && devices[n].can_id == n &&
              devices[n].current_limit_dA == 1);
        if (regular) devices[n].zero_mit_loopback = false;
    }
    profile.devices = devices;
    fixture_profile = &profile;
    CHECK(motor_control_init() && motor_control_boot_startup(uint64_t{startup_epoch} << 32));
    select_wire_profile(1);
    CHECK(object<mc_board_config_t>(0x2001)->AxisConfiguredMask == ((15U << 14) | (1U << 21)));
    if (is("hot_select") || is("hot_timeout")) { test_hot_topology(is("hot_timeout")); return; }
    CHECK(test_input_size == 256 && motor_pdo::layout::active().bytes(1, false) == 216);
    CHECK(gRxAssign.aEntries[3] == 0x162E && gTxAssign.aEntries[7] == 0x1A35);
    const uint16_t saved = gRxAssign.aEntries[3];
    gRxAssign.aEntries[3] = gRxAssign.aEntries[4];
    gRxAssign.aEntries[4] = saved;
    uint16_t in = 1, out = 1;
    CHECK(motor_control_mapping(&in, &out) != 0 && !in && !out);
    select_wire_profile(1);
    constexpr uint8_t mit_zero[]{0, 0, 0, 0x7f, 0xff, 0x7f, 0xf7, 0xff};
    unsigned transmitted[4]{}, sequence = 0, beat = 0, last_tx_us = 0;
    size_t read = 0;
    bool command_expected = false;
    std::deque<std::pair<dcu_can_frame_t, dcu_can_rx_stamp_t>> held;
    const auto worker = [&] {
        test_ms = (test_us + DCU_WORKER_PERIOD_US) / 1000U;
        test_tx_eof = {test_us * 100U + 100U, true};
        run_cycle();
        while (read < sent.size()) {
            const auto [bus, tx] = sent[read++];
            CHECK(bus == 2 && tx.id >= 1 && tx.id <= 4 && tx.flags == 0x18);
            dcu_can_frame_t rx{tx.id, 6, tx.flags, {0xa0, tx.data[1]}};
            if (tx.dlc == 2) {
                CHECK(tx.data[0] == 0xe0 && tx.data[1] >= 1 && tx.data[1] <= 3);
                if (tx.data[1] == 1) { rx.data[2] = 0x42; rx.data[3] = 0xb4; }
                else if (!regular) { rx.data[2] = 0x3f; rx.data[3] = 0x80; } // 1 rpm or 1 A
            } else {
                CHECK(command_expected && tx.dlc == sizeof(mit_zero) && !std::memcmp(tx.data, mit_zero, sizeof(mit_zero)));
                CHECK(!last_tx_us || test_us - last_tx_us >= 100);
                last_tx_us = test_us;
                ++transmitted[tx.id - 1];
                rx = {tx.id, 8, tx.flags, {0x20, 0x80, static_cast<uint8_t>(tx.id), 0x80, 8, 0, 50, 50}};
                if (!regular || (beat == 2 && tx.id == 3)) {
                    rx.data[4] = beat % 2 ? 7 : 8;
                    rx.data[5] = beat % 2 ? 0xfd : 2; // -0.12 A / +0.12 A
                }
                if (tx.id == 3 && beat == 2) {
                    if (is("silent")) continue;
                    if (is("cancel_tx")) tx_events[2].back().status = DCU_CAN_TX_CANCELLED;
                    if (is("bad_tx_eof")) tx_events[2].back().eof.valid = false;
                    if (is("motor_fault")) rx.data[0] |= 1;
                }
            }
            const bool good_eof = !(command_expected && beat == 2 && tx.id == 3 && is("bad_eof"));
            dcu_can_rx_stamp_t stamp{test_ms, {test_tx_eof.tick + 10U, good_eof}};
            if (command_expected && beat == 2 && tx.id == 3 && is("old_rx")) stamp.eof.tick = test_tx_eof.tick - 1U;
            if (endurance && tx.dlc == 8) {
                held.emplace_back(rx, stamp);
                if (held.size() == 4) {
                    incoming[2].insert(incoming[2].end(), held.rbegin(), held.rend());
                    held.clear();
                }
            } else incoming[2].emplace_back(rx, stamp);
        }
    };
    while (test_ms < 350 && startup_state() != DCU_GATEWAY_READY) worker();
    CHECK(startup_state() == DCU_GATEWAY_READY);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    const auto cycle = [&](bool enabled, uint16_t control) {
        uint8_t wire[784]{};
        mc_board_rx_t board{};
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        board.CycleSeq = ++sequence;
        board.CommandLeaseMs = 10;
        board.BoardControl = enabled ? 1 : 0;
        board.ChannelEnableMask = enabled ? 1U << 2 : 0;
        board.AxisEnableMask = enabled ? 15U << 14 : 0;
        motor_pdo::store(wire, 0, board);
        motor_pdo::store(wire, 32, mc_channel_rx_t{1, 15, 1, 2000});
        mc_axis_rx_t axis{};
        axis.ControlWord = control;
        axis.Mode = 4;
        for (unsigned n = 0; n < 4; ++n) {
            axis.Kp = n == 2 && beat == 2 && is("nonzero") ? 1 : 0;
            motor_pdo::store(wire, 48 + n * 24, axis);
        }
        motor_control_set_cyclic_feedback_window(2000, test_us + 1600);
        motor_control_output(wire);
        for (unsigned us = 0; us < 2000; us += DCU_WORKER_PERIOD_US) worker();
        if (motor_control_cyclic_feedback_missing()) return; // No new SM3 on a missed deadline.
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == sequence);
    };
    cycle(false, 0);
    cycle(true, 0);
    for (unsigned i = 0; i < 40; ++i) cycle(true, MC_AXIS_ARM);
    for (unsigned n = 0; n < 4; ++n) CHECK(axis_feedback(14 + n)->StatusWord & 4U);
    command_expected = true;
    const uint32_t all = 15U << 14;
    const auto evidence = [&] {
        return dcu::loopback_evidence::read(ssc_txpdo + motor_pdo::layout::active().sideband(true));
    };
    for (beat = 1; beat <= (endurance ? 30000U : 2U); ++beat) {
        const bool failed = !endurance && beat == 2;
        uint8_t previous[256];
        if (failed) std::memcpy(previous, ssc_txpdo, sizeof(previous));
        cycle(true, MC_AXIS_ARM | MC_AXIS_START_MOTION);
        CHECK(motor_control_cyclic_feedback_required() == 0x1e);
        CHECK(motor_control_cyclic_feedback_missing() == (failed ? 1U << 3 : 0));
        CHECK(motor_pdo::load<mc_board_tx_t>(ssc_txpdo, 0).AcceptedCycleSeq == sequence - failed);
        if (failed) CHECK(!std::memcmp(previous, ssc_txpdo, sizeof(previous)));
        const auto masks = evidence();
        const bool no_tx = failed && (is("cancel_tx") || is("nonzero"));
        CHECK(masks.requested == (regular ? 0 : all));
        CHECK(masks.tx_complete == masks.requested && masks.rx_matched == masks.requested);
        for (unsigned n = 0; n < 4; ++n) {
            CHECK(transmitted[n] == beat - (n == 2 && failed && is("nonzero")));
            if (failed) {
                const auto slot = 14 + n;
                CHECK(read_sdo<uint32_t>(0x2400 + 16 * slot, 2) == beat - (n == 2 && no_tx));
                MotorFeedback f{};
                MotorDiagnostics d{};
                CHECK(read_device(1 + n, f, d));
                if (n == 2 && regular) CHECK(f.bridge_fault == 2 && d.active_fault_bits == 4);
                if (n == 2 && is("motor_fault")) CHECK(f.sample.motor_fault != 0);
            } else {
                const auto *feedback = axis_feedback(14 + n);
                CHECK(axis_diagnostics(14 + n)->TxCompleteCount == beat);
                CHECK(feedback->ModeDisplay == 4 && (feedback->StatusWord & (8U | 16U)) == (8U | 16U));
                CHECK(!feedback->BridgeFaultCode && !feedback->MotorFaultRaw && !axis_diagnostics(14 + n)->ActiveFaultBits);
                if (!regular) CHECK(feedback->ActualCurrent == (beat % 2 ? -12 : 12));
            }
        }
    }
    command_expected = false;
    cycle(false, 0);
    CHECK(!evidence().requested && !axis_diagnostics(21)->TxCompleteCount);
}

static void test_raw_motor_handoff()
{
    fixture_profile = host_hra55h100_profile();
    CHECK(motor_control_init() && motor_control_boot_startup(17));
    select_wire_profile(1);
    uint16_t owner = 1U << 3;
    const auto grant = [&] { return MC_OD_Write(0x2101, 6, 1, &owner, 0); };
    CHECK(grant() && !write_limits(1U << 24) && !switch_request(3));
    CHECK(!grant() && !read_sdo<uint8_t>(0x2101, 6) && !switch_action(1));
    select_wire_profile(3);
    CHECK(read_sdo<uint8_t>(0x2101, 6) == owner);
    nAlStatus = STATE_OP;
    motor_control_operational(true);
    CHECK(grant());
    mc_raw_t wire{};
    wire.BatchSeq = 1;
    motor_control_output(&wire);
    run_cycle();
    wire.BatchSeq++;
    wire.ValidMask = wire.SlotCount = 1;
    wire.Channel = 3;
    wire.CanId = 0x02;
    wire.FrameFlags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    wire.Dlc = 10;
    wire.Data_0 = 0x11;
    motor_control_output(&wire);
    run_cycle();
    CHECK(sent.size() == 1 && sent[0].first == 3 && sent[0].second.id == 0x02);
    CHECK(sent[0].second.dlc == 10 && sent[0].second.data[0] == 0x11 && !raw_drops());
    deliver(3, {0x02, 10, wire.FrameFlags, {0x80}});
    run_cycle();
    motor_control_prepare_input();
    CHECK(ssc_txpdo[4] == 1 && ssc_txpdo[16] == 0x02 && ssc_txpdo[24] == 0x80);
    test_ms += 2;
    ++wire.BatchSeq;
    wire.CanId = 0x7ff;
    motor_control_output(&wire);
    run_cycle();
    CHECK(sent.size() == 1 && raw_drops() == 1);
    motor_control_operational(false);
    CHECK(!read_sdo<uint8_t>(0x2101, 6));
}

static void test_raw_without_motor_probes(unsigned target)
{
    fixture_profile = host_mixed_profile();
    CHECK(motor_control_init() && motor_control_boot_startup(17));
    select_wire_profile(1);
    test_ms = 1;
    run_cycle();
    CHECK(!sent.empty() && !owner_change_allowed());
    CHECK(!switch_request(target) && !switch_action(1));
    const auto cancels = service_cancellations[2] + service_cancellations[0];
    select_wire_profile(target);
    CHECK(service_cancellations[2] + service_cancellations[0] > cancels && owner_change_allowed());
    const auto count = sent.size();
    for (unsigned i = 0; i < 25; ++i) { ++test_ms; run_cycle(); }
    CHECK(sent.size() == count && startup_state() == DCU_GATEWAY_READY);
    CHECK(!switch_request(1) && !switch_action(1));
    select_wire_profile(1);
    for (unsigned i = 0; i < 25; ++i) { ++test_ms; run_cycle(); }
    CHECK(sent.size() > count && startup_state() == DCU_GATEWAY_PREPARING);
}

int main(int argc, char **argv)
{
    if (argc == 3 && !std::strcmp(argv[1], "dynamic_policy")) {
        test_dynamic_policy(argv[2]);
        return 0;
    }
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "hot_select") || !std::strcmp(argv[1], "hot_timeout")) {
        test_encos_four_axis_production_pdo(argv[1]);
        return 0;
    }
    if (!std::strcmp(argv[1], "encos_four_axis_production_pdo") || !std::strncmp(argv[1], "encos_loopback_", 15)) {
        test_encos_four_axis_production_pdo(!std::strncmp(argv[1], "encos_loopback_", 15) ? argv[1] + 15 : argv[1]);
        return 0;
    }
    if (!std::strcmp(argv[1], "registered_passthrough")) {
        test_registered_passthrough();
        return 0;
    }
    if (!std::strcmp(argv[1], "mixed_raw_no_probe") || !std::strcmp(argv[1], "mixed_fd_no_probe")) {
        test_raw_without_motor_probes(!std::strcmp(argv[1], "mixed_raw_no_probe") ? 2 : 3);
        return 0;
    }
    if (!std::strcmp(argv[1], "production_raw") || !std::strcmp(argv[1], "production_no_quota")) {
        test_production_raw(!std::strcmp(argv[1], "production_raw"));
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_motor_handoff")) {
        test_raw_motor_handoff();
        return 0;
    }
    if (!std::strcmp(argv[1], "production_startup") || !std::strcmp(argv[1], "production_no_journal") ||
        !std::strcmp(argv[1], "switch_with_motor")) {
        test_production_startup(std::strcmp(argv[1], "production_no_journal"), !std::strcmp(argv[1], "switch_with_motor"));
        return 0;
    }
    if (!std::strcmp(argv[1], "mixed_motor_profile")) {
        fixture_profile = host_mixed_profile();
        CHECK(motor_control_init());
        CHECK(adapter::configured_device(14)->model_profile_id == 0x8000);
        CHECK(adapter::configured_device(15)->protocol == DCU_MOTOR_PROTOCOL_BJCX);
        CHECK(adapter::configured_device(0)->protocol == DCU_MOTOR_PROTOCOL_BJCX);
        CHECK(!std::strcmp(adapter::configured_device(15)->type, "HRA40H-101"));
        CHECK(!std::strcmp(adapter::configured_device(0)->type, "HRA55H-100"));
        CHECK(adapter::configured_device(0)->can_id == adapter::configured_device(14)->can_id);
        CHECK(sent.empty());
        select_owner(OWNER_SDO);
        CHECK(position_query(15, 1) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
        CHECK(position_query(14, 1) == 0);

        test_ms = 1;
        allow_tx_events = false;
        run_cycle();
        CHECK(sent.size() == 2);
        const auto &[slow_bus, status_request] = sent[0];
        const auto &[legacy_bus, legacy_request] = sent[1];
        constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        CHECK(slow_bus == 0 && status_request.id == 3 && status_request.flags == fd_brs &&
              status_request.dlc == 1 && status_request.data[0] == 0x17);
        CHECK(legacy_bus == 2 && legacy_request.id == 3 && legacy_request.flags == fd_brs &&
              legacy_request.dlc == 2 && legacy_request.data[1] == 1);

        const uint8_t packet[] = {0x80, 0, 0, 0x3F, 0x80, 0, 0, 0x40, 0, 0, 0, 0xFF, 0x9C, 75, 80, 96};
        dcu_can_frame_t status{};
        status.id = 3;
        status.dlc = 10;
        status.flags = fd_brs;
        std::memcpy(status.data, packet, sizeof(packet));
        deliver(0, status); // Before matching hardware completion: not a valid reply.
        motor_control_prepare_input();
        CHECK(!axis_feedback(0)->ValidFlags);

        allow_tx_events = true;
        deliver(3, status);  // Controller not enabled by the Profile.
        deliver(2, status);  // Same CAN ID, but this device uses ENCOS.
        CHECK(parameters::inspect(2).busy);
        motor_control_prepare_input();
        CHECK(!axis_feedback(14)->ValidFlags && !axis_feedback(0)->ValidFlags);
        deliver(0, status);
        motor_control_prepare_input();
        CHECK(axis_feedback(0)->ValidFlags == 0x37 && axis_feedback(0)->ActualPosition == 17453);
        CHECK(axis_feedback(0)->ActualVelocity == 209440 && axis_feedback(0)->ActualCurrent == -100);
        CHECK(axis_feedback(0)->MotorTemperature == 250 && axis_feedback(0)->MosTemperature == 300);
        CHECK(!axis_feedback(14)->ValidFlags);
        check_txpdo_bytes();

        dcu_can_frame_t legacy{};
        legacy.id = 3;
        legacy.flags = fd_brs;
        legacy.dlc = 6;
        legacy.data[0] = 0xA0;
        legacy.data[1] = 1;
        legacy.data[2] = 0x40;
        legacy.data[3] = 0x80;
        deliver(2, legacy);
        CHECK(!parameters::inspect(2).busy);
        for (unsigned i = 0; i < 6 && sent.size() < 3; ++i) run_cycle();
        CHECK(sent.size() == 3);
        CHECK(sent[2].first == 2 && sent[2].second.id == 36 && sent[2].second.dlc == 1 &&
              sent[2].second.data[0] == 0x17);
        CHECK(!owner_change_allowed());
        // The sibling BJCX status poll no longer reserves the ENCOS device.
        CHECK(read_parameter(0, {2, 1, 0x101}, test_ms, true).accepted);
        status.id = 36;
        status.data[0] = 0xA1;
        deliver(2, status);
        motor_control_prepare_input();
        CHECK(!axis_feedback(15)->ValidFlags);
        status.data[0] = 0x80;
        status.data[1] = 1; // 12-bit alarm that cannot be represented by the legacy fault byte.
        status.data[3] = 0x7F;
        status.data[4] = 0xC0; // NaN position must not conceal the alarm.
        deliver(2, status);
        motor_control_prepare_input();
        CHECK(axis_feedback(14)->ActualPosition == 69813 && axis_feedback(0)->ActualPosition == 17453);
        CHECK(axis_feedback(15)->MotorFaultRaw == UINT8_MAX &&
              (axis_feedback(15)->StatusWord & 64U) && !(axis_feedback(15)->ValidFlags & 1));
        CHECK(object<mc_board_tx_t>(0x6000)->AxisOnlineMask == (1U | (1U << 14) | (1U << 15)));

        test_ms = 101;
        run_cycle();
        CHECK(sent.size() >= 4);
        motor_control_operational(false);
        status.id = 3;
        std::memcpy(status.data, packet, sizeof(packet));
        status.data[3] = 0x40;
        status.data[4] = 0x80;
        deliver(0, status); // An old reply from the closed session must not update the sample.
        motor_control_prepare_input();
        CHECK(axis_feedback(0)->ActualPosition == 17453);

        select_owner(OWNER_PDO);
        select_wire_profile(1);
        motor_control_operational(true);
        command_axis(0, false);
        command_axis(0);
        command_axis(MC_AXIS_ARM);
        command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
        for (const auto &[bus, frame] : sent)
            if (bus == 0 || (bus == 2 && frame.id == 36))
                CHECK(frame.dlc == 1 && frame.data[0] == 0x17);
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & (8U | MC_STATUS_ARMED_QUERY_ONLY)));
        return 0;
    }
    if (!std::strncmp(argv[1], "hra55h100_profile", sizeof("hra55h100_profile") - 1)) {
        const char *scenario = argv[1] + sizeof("hra55h100_profile") - 1;
        auto profile = *host_hra55h100_profile();
        const bool pipeline = !std::strncmp(scenario, "_pipeline", 9);
        dcu_motor_device_profile_t motors[]{profile.devices[0], profile.devices[0]};
        if (pipeline) {
            motors[1].can_id = 3;
            motors[1].neutral_mit_frame_limit = 2;
            profile.device_count = 2;
            profile.devices = motors;
        }
        fixture_profile = &profile;
        constexpr unsigned bus = 3;
        constexpr unsigned slot = 21;
        constexpr uint16_t can_id = 0x02;
        constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        CHECK(motor_control_init());
        const auto *device = adapter::configured_device(slot);
        CHECK(device && device->bus == bus && device->can_id == can_id);
        CHECK(device->protocol == DCU_MOTOR_PROTOCOL_BJCX && device->model_profile_id == 0 && device->neutral_mit_only);
        CHECK(device->neutral_mit_frame_limit == 30000);
        CHECK(!std::strcmp(device->type, "HRA55H-100"));
        CHECK(adapter::device_for_slot(slot) == 0 && adapter::slot_for_device(0) == slot);
        const uint8_t members = pipeline ? 3U : 1U;
        CHECK(object<mc_board_config_t>(0x2001)->AxisConfiguredMask == (uint32_t{members} << slot));
        CHECK(sent.empty());

        select_wire_profile(1);
        CHECK(test_input_size == (pipeline ? 160 : 128));
        CHECK(motor_control_boot_startup(uint64_t{startup_epoch} << 32));
        if (!std::strcmp(scenario, "_quota_equivalence")) CHECK(!write_limits(1U << 24));
        test_tx_eof = {UINT32_MAX - 2, true};
        test_ms = 1;
        run_cycle();
        CHECK(startup_state() == DCU_GATEWAY_READY && sent.empty());
        uint8_t wire[784]{};
        auto submit = [&](uint32_t sequence, uint16_t controls, bool enabled, int32_t position = 0,
                          uint32_t period_ns = 2000000, uint32_t window_us = 900) {
            test_ms += 2;
            motor_control_set_neutral_window(period_ns, test_ms * 1000U + window_us);
            mc_board_rx_t board{};
            board.InterfaceVersion = MC_INTERFACE_VERSION;
            board.CycleSeq = sequence;
            board.BoardControl = enabled ? 1 : 0;
            board.ChannelEnableMask = enabled ? 1U << bus : 0;
            board.AxisEnableMask = enabled ? members << slot : 0;
            board.CommandLeaseMs = 10;
            motor_pdo::store(wire, 0, board);
            const mc_channel_rx_t channel{1, members, 1, 2000};
            motor_pdo::store(wire, 40, channel);
            mc_axis_rx_t axis{};
            axis.Mode = 4;
            axis.ControlWord = controls;
            axis.TargetPosition = position;
            motor_pdo::store(wire, 48, axis);
            if (pipeline) motor_pdo::store(wire, 72, axis);
            motor_control_output(wire);
            run_cycle();
        };
        constexpr uint16_t start = MC_AXIS_ARM | MC_AXIS_START_MOTION;
        submit(1, start, true); // Not in OP.
        motor_control_operational(true);
        submit(2, start, true); // No safe baseline yet.
        submit(3, 0, false);
        submit(4, start, true); // Missing explicit ARM frame.
        submit(5, MC_AXIS_ARM, true);
        submit(6, start, true, 1); // Non-neutral data invalidates ARM.
        submit(7, start, true);
        CHECK(sent.empty() && !motor_control_neutral_feedback_pending());
        submit(8, MC_AXIS_ARM, true, 0, 1000000); // A 2 ms hint cannot override actual Sync0.
        submit(9, start, true, 0, 1000000);
        CHECK(sent.empty());
        submit(10, MC_AXIS_ARM, true);
        const bool retry_recovers = !std::strcmp(scenario, "_retry_recovers");
        const bool retry_expires = !std::strcmp(scenario, "_retry_expires");
        if (retry_recovers || retry_expires || pipeline) allow_tx_events = false;
        if (!std::strcmp(scenario, "_submit_rejected")) allow_send = false;
        submit(11, start, true, 0, 2000000, !std::strcmp(scenario, "_short_window") ? 349 : 900);
        if (!std::strcmp(scenario, "_submit_rejected") || !std::strcmp(scenario, "_short_window")) {
            allow_send = true;
            CHECK(sent.empty());
            submit(12, MC_AXIS_ARM, true);
            submit(13, start, true);
            CHECK(sent.empty());
            return 0;
        }
        CHECK(sent.size() == 1 && sent[0].first == bus && motor_control_neutral_feedback_pending());
        const auto &request = sent[0].second;
        const uint8_t mit[16] = {0x11};
        CHECK(request.id == can_id && request.flags == fd_brs && request.dlc == 10);
        CHECK(std::memcmp(request.data, mit, sizeof(mit)) == 0);

        dcu_can_frame_t reply{can_id, 10, fd_brs, {0x80, 0x10, 0, 0x3f, 0x80}};
        if (!std::strcmp(scenario, "_baseline_equivalence") || !std::strcmp(scenario, "_quota_equivalence")) {
            deliver(bus, reply, {1, true});
            motor_control_prepare_input();
            CHECK(axis_feedback(slot)->ValidFlags == 0x37);
            std::fwrite(wire, 1, motor_pdo::layout::active().bytes(1, false), stdout);
            std::fwrite(ssc_txpdo, 1, test_input_size, stdout);
            std::fwrite(request.data, 1, sizeof(mit), stdout);
            return 0;
        }
        if (pipeline) {
            // No sibling TX before the first retires, even if its submit-time gap elapsed.
            test_tx_eof = {20, true};
            run_cycle();
            run_cycle();
            CHECK(sent.size() == 1 && tx_events[bus].size() == 1);
            allow_tx_events = true;
            run_cycle();
            CHECK(sent.size() == 2 && sent.back().second.id == 3 && tx_events[bus].size() == 1);
            // Replies may still arrive in reverse order; TX pacing never waits on RX.
            reply.id = 3;
            deliver(bus, reply, {21, true});
            motor_control_prepare_input();
            CHECK(axis_feedback(slot + 1)->ValidFlags == 0x37 && !axis_feedback(slot)->ValidFlags);
            CHECK(motor_control_neutral_feedback_pending());
            if (!std::strcmp(scenario, "_pipeline_timeout")) {
                test_us += 350;
                run_cycle();
                motor_control_prepare_input();
                CHECK(!motor_control_neutral_feedback_pending() && (object<mc_board_tx_t>(0x6000)->BoardStatus & 32U));
                submit(12, start, true);
                CHECK(sent.size() == 2);
                return 0;
            }
            reply.id = can_id;
            deliver(bus, reply, {1, true}); // Valid for the first TX, not the second.
            CHECK(!motor_control_neutral_feedback_pending());
            motor_control_prepare_input();
            CHECK(axis_feedback(slot)->ValidFlags == 0x37 && axis_feedback(slot + 1)->ValidFlags == 0x37);
            submit(12, start, true);
            run_cycle();
            run_cycle();
            CHECK(sent.size() == 4 && sent[2].second.data[15] == 1 && sent[3].second.data[15] == 1);
            deliver(bus, reply, {21, true});
            reply.id = 3;
            deliver(bus, reply, {22, true});
            submit(13, start, true);
            run_cycle();
            run_cycle();
            CHECK(sent.size() == 5 && sent.back().second.id == can_id && sent.back().second.data[15] == 2);
            return 0;
        }
        if (retry_recovers || retry_expires) {
            const unsigned before = service_cancellations[bus];
            run_cycle(); // A recoverable CAN error has not retired the hardware token.
            CHECK(motor_control_neutral_feedback_pending() && sent.size() == 1);
            if (retry_expires) {
                test_us += 350;
                run_cycle();
                CHECK(service_cancellations[bus] == before + 1);
                CHECK(!motor_control_neutral_feedback_pending());
            }
            allow_tx_events = true;
            deliver(bus, reply, {1, true});
            motor_control_prepare_input();
            CHECK(axis_feedback(slot)->ValidFlags == (retry_recovers ? 0x37U : 0U));
            CHECK(sent.size() == 1 && !motor_control_neutral_feedback_pending());
            if (retry_expires) {
                CHECK(object<mc_board_tx_t>(0x6000)->BoardStatus & 32U);
                submit(12, start, true);
                CHECK(sent.size() == 1);
            }
            return 0;
        }
        if (*scenario && std::strcmp(scenario, "_endurance")) {
            dcu_can_eof_t eof{1, true};
            if (!std::strcmp(scenario, "_deadline"))
                test_us += 300; // Worker step adds 50 us: equal to the reply timeout.
            else if (!std::strcmp(scenario, "_cancel"))
                submit(11, 0, false); // Even a duplicate sequence conveys stop intent.
            else if (!std::strcmp(scenario, "_tx_stamp"))
                tx_events[bus].front().eof = {};
            else if (!std::strcmp(scenario, "_wrong_id"))
                reply.id = 1;
            else if (!std::strcmp(scenario, "_rx_stamp"))
                eof.valid = false;
            else if (!std::strcmp(scenario, "_stale_rx"))
                eof.tick = UINT32_MAX - 3;
            else if (!std::strcmp(scenario, "_equal_rx"))
                eof.tick = UINT32_MAX - 2;
            else if (!std::strcmp(scenario, "_alarm"))
                reply.data[2] = 1;
            else if (!std::strcmp(scenario, "_invalid_sample"))
                reply.data[3] = 0x7f, reply.data[4] = 0xc0; // NaN position.
            else if (!std::strcmp(scenario, "_bus_warning"))
                bus_states[bus] = DCU_CAN_BUS_ERROR_WARNING;
            else if (!std::strcmp(scenario, "_handoff"))
                motor_control_neutral_feedback_deadline();
            else if (!std::strcmp(scenario, "_duplicate_pdo"))
                submit(11, start, true);
            else if (!std::strcmp(scenario, "_gap")) {
                deliver(bus, reply, eof);
                motor_control_prepare_input();
                CHECK(axis_feedback(slot)->StatusWord & 16U);
                submit(13, start, true);
            }
            else
                CHECK(false);
            deliver(bus, reply, eof);
            CHECK(!motor_control_neutral_feedback_pending());
            motor_control_prepare_input();
            CHECK(!axis_feedback(slot)->ValidFlags && !(axis_feedback(slot)->StatusWord & 16U));
            if (std::strcmp(scenario, "_cancel"))
                CHECK(object<mc_board_tx_t>(0x6000)->BoardStatus & 32U);
            submit(14, MC_AXIS_ARM, true);
            submit(15, start, true);
            CHECK(sent.size() == 1);
            return 0;
        }
        deliver(bus, reply, {1, true}); // Valid wrap of the 32-bit MCAN timebase.
        motor_control_prepare_input();
        CHECK(axis_feedback(slot)->ValidFlags == 0x37 && axis_feedback(slot)->ActualPosition == 17453);
        CHECK(axis_feedback(slot)->StatusWord & 16U);
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 11);
        CHECK(object<mc_board_tx_t>(0x6000)->AxisOnlineMask == (1U << slot));
        CHECK(sent.size() == 1);
        motor_control_prepare_input();
        CHECK(axis_feedback(slot)->ActualPosition == 17453);
        CHECK(!(axis_feedback(slot)->StatusWord & 16U));
        const uint32_t frames = *scenario ? device->neutral_mit_frame_limit : 3;
        for (uint32_t i = 1; i < frames; ++i) {
            test_tx_eof.tick = 10U * i;
            submit(11 + i, start, true);
            CHECK(sent.size() == i + 1 && motor_control_neutral_feedback_pending());
            const auto &[actual_bus, frame] = sent.back();
            CHECK(actual_bus == bus && frame.id == can_id && frame.dlc == 10 && frame.flags == fd_brs);
            CHECK(std::memcmp(frame.data, mit, 15) == 0 && frame.data[15] == static_cast<uint8_t>(i));
            deliver(bus, reply, {test_tx_eof.tick + 1U, true});
            motor_control_prepare_input();
            CHECK(axis_feedback(slot)->ValidFlags == 0x37 && (axis_feedback(slot)->StatusWord & 16U));
            CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 11 + i);
            CHECK(!(object<mc_board_tx_t>(0x6000)->BoardStatus & 0x7cU));
        }
        if (*scenario) submit(11 + frames, start, true); // Ceiling blocks even without STOP.
        submit(12 + frames, 0, false);
        motor_control_operational(false);
        motor_control_operational(true);
        submit(13 + frames, 0, false);
        submit(14 + frames, MC_AXIS_ARM, true);
        submit(15 + frames, start, true);
        CHECK(sent.size() == frames && !motor_control_neutral_feedback_pending());
        return 0;
    }
    if (!std::strcmp(argv[1], "can_profile_config")) {
        fixture_profile = host_fixture_profile();
        CHECK(motor_control_init());
        CHECK(adapter::device_for_slot(14) == 0 && adapter::slot_for_device(0) == 14);
        const auto *device = adapter::configured_device(14);
        CHECK(device && device->can_id == 3 && device->model_profile_id == 0x8000);
        CHECK(device->position_max_urad == 9000000 && device->current_limit_dA == 60);
        CHECK(!adapter::configured_device(0) && !adapter::configured_device(15));
        CHECK(object<mc_board_config_t>(0x2001)->AxisConfiguredMask == (1U << 14));
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            const unsigned index = 0x2010 + c;
            CHECK(read_sdo<uint8_t>(index, 2) == (c == 2));
            CHECK(read_sdo<uint32_t>(index, 3) == (c == 2 ? 1000000U : 0U));
            CHECK(read_sdo<uint32_t>(index, 4) == (c == 2 ? 5000000U : 0U));
            CHECK(object<mc_channel_config_t>(index)->MotorConfiguredMask == (c == 2));
        }
        CHECK(sent.empty());
        select_owner(OWNER_SDO);
        CHECK(position_query(15, 1) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
        CHECK(position_query(14, 2) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
        run_cycle();
        CHECK(sent.empty());
        CHECK(position_query(14, 1) == 0);
        run_cycle();
        CHECK(sent.size() == 1 && sent[0].first == 2 && sent[0].second.id == 3);
        CHECK((sent[0].second.flags & DCU_CAN_FLAG_FD) && sent[0].second.data[1] == 1);
        dcu_can_frame_t reply = sent[0].second;
        reply.id = 4;
        reply.dlc = 6;
        reply.data[0] = 0xA0;
        deliver(2, reply);
        CHECK(parameters::inspect(2).busy);
        reply.id = 3;
        deliver(2, reply);
        CHECK(!parameters::inspect(2).busy);
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->AxisOnlineMask == (1U << 14));
        CHECK(axis_feedback(14)->StatusWord & 1U);
        CHECK(!(axis_feedback(0)->StatusWord & 1U));

        select_owner(OWNER_PDO);
        select_wire_profile(1);
        motor_control_operational(true);
        sent.clear();
        uint8_t wire[784]{};
        mc_board_rx_t board{};
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        board.CommandLeaseMs = 10;
        board.ChannelEnableMask = 1U << 2;
        uint8_t active_mask = 2;
        const auto submit = [&](unsigned slot, const mc_axis_rx_t &command) {
            const mc_channel_rx_t channel{1, active_mask, 1, 2000};
            motor_pdo::store(wire, motor_pdo::layout::active().channel(false, 2), channel);
            const auto offset = motor_pdo::layout::active().axis(false, slot);
            if (offset != motor_pdo::layout::absent) motor_pdo::store(wire, offset, command);
            ++board.CycleSeq;
            motor_pdo::store(wire, 0, board);
            motor_control_output(wire);
            run_cycle();
        };
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            const mc_channel_rx_t channel{1, 0, 0, 2000};
            if (motor_pdo::layout::active().buses & (1U << c))
                motor_pdo::store(wire, motor_pdo::layout::active().channel(false, c), channel);
        }
        mc_axis_rx_t command{};
        command.Mode = 1;
        command.CurrentLimit = 50;
        const uint16_t controls[]{0, 0, MC_AXIS_ARM, MC_AXIS_ARM | MC_AXIS_START_MOTION};
        for (uint16_t control : controls) {
            board.BoardControl = board.CycleSeq != 0;
            board.AxisEnableMask = board.CycleSeq ? 1U << 15 : 0;
            command.ControlWord = control;
            submit(15, command);
        }
        for (unsigned i = 0; i < 20; ++i) run_cycle();
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 4);
        CHECK(!(axis_feedback(15)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY));
        CHECK(object<mc_axis_rx_t>(0x7200 + 15)->ControlWord == 0);

        active_mask = 1;
        board.AxisEnableMask = 1U << 14;
        command.ControlWord = 0;
        submit(14, command);
        command.ControlWord = MC_AXIS_ARM;
        submit(14, command);
        command.ControlWord |= MC_AXIS_START_MOTION;
        command.TargetPosition = device->position_max_urad + 1;
        submit(14, command);
        motor_control_prepare_input();
        CHECK(axis_feedback(14)->BridgeFaultCode == 2);
        for (const auto &[bus, frame] : sent)
            CHECK(bus == 2 && frame.id == 3 && frame.dlc == 2); // Only bound-device queries, no control.
        return 0;
    }
    if (!std::strcmp(argv[1], "cross_bus_device_mapping")) {
        two_bus_devices[0] = test_device;
        two_bus_devices[0].id = "first_in_profile";
        two_bus_devices[1] = test_device;
        test_profile.devices = two_bus_devices;
        test_profile.device_count = 2;
        CHECK(!motor_control_init()); // Duplicate on the same bus: do not publish even the first slot.
        CHECK(!adapter::configured_device(0) && !object<mc_board_config_t>(0x2001)->AxisConfiguredMask);
        CHECK(sent.empty());
        two_bus_devices[0].bus = 1; // The same CAN ID on two physical buses is valid.
        CHECK(motor_control_init());
        CHECK(!motor_control_init());
        CHECK(adapter::device_for_slot(7) == 0 && adapter::device_for_slot(0) == 1);
        CHECK(adapter::slot_for_device(0) == 7 && adapter::slot_for_device(1) == 0);
        select_owner(OWNER_SDO);
        CHECK(position_query(7, 1) == 0 && position_query(0, 1) == 0);
        run_cycle();
        run_cycle();
        CHECK(sent.size() == 2 && sent[0].first == 0 && sent[1].first == 1);
        CHECK(sent[0].second.id == 1 && sent[1].second.id == 1);
        dcu_can_frame_t response{};
        response.id = 1;
        response.dlc = 6;
        response.data[0] = 0xA0;
        response.data[1] = 1;
        response.data[2] = 0x3F;
        response.data[3] = 0x80;
        deliver(1, response);
        CHECK(object<mc_transaction_t>(axis_object_base(7) + 7)->DoneSeq == 1);
        CHECK(object<mc_transaction_t>(axis_object_base(0) + 7)->DoneSeq == 0);
        CHECK(read_sdo<int32_t>(0x2400 + 16 * 7, 8) == 17453);
        response.data[2] = 0x40;
        response.data[3] = 0;
        deliver(0, response);
        CHECK(object<mc_transaction_t>(axis_object_base(0) + 7)->DoneSeq == 1);
        motor_control_prepare_input();
        CHECK(axis_diagnostics(0)->QueriedPosition == 34906 && axis_diagnostics(7)->QueriedPosition == 17453);
        CHECK(object<mc_board_tx_t>(0x6000)->AxisOnlineMask == ((1U << 7) | 1U));
        return 0;
    }
    if (!std::strcmp(argv[1], "empty_profile")) {
        test_profile.devices = nullptr;
        test_profile.device_count = 0;
        CHECK(motor_control_init());
        CHECK(object<mc_board_config_t>(0x2001)->AxisConfiguredMask == 0);
        for (unsigned slot = 0; slot < ENCOS_AXIS_COUNT; ++slot)
            CHECK(!adapter::configured_device(slot));
        select_owner(OWNER_PDO);
        select_wire_profile(1);
        motor_control_operational(true);
        command_axis(0, false, 0);
        command_axis(0, true, 0);
        command_axis(MC_AXIS_ARM, true, 0);
        command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION, true, 0);
        for (unsigned i = 0; i < 20; ++i) run_cycle();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 4 && sent.empty());
        return 0;
    }
    if (!std::strcmp(argv[1], "config_source")) {
        const unsigned last = axis_object_base(ENCOS_AXIS_COUNT - 1);
        auto *config = object<mc_board_config_t>(0x2001);
        CHECK(!config->MaxCommandLeaseMs && !config->FeedbackTimeoutMs);
        config->MaxCommandLeaseMs = 1;
        object<mc_routing_t>(0x3000)->CanId = 7;
        CHECK(motor_control_init());
        CHECK(adapter::configured_device(0)->can_id == 1);
        CHECK(config->MaxCommandLeaseMs == 10 && config->FeedbackTimeoutMs == 20);
        uint16_t value[2]{};
        for (unsigned index : {0x2002U, 0x3000U, 0x3001U, 0x3002U, 0x3003U, 0x3004U,
                               0x3008U, last, last + 8}) {
            CHECK(MC_OD_Read(index, 1, 2, value, 0) == ABORTIDX_UNSUPPORTED_ACCESS);
            CHECK(MC_OD_Write(index, 1, 2, value, 0) == ABORTIDX_READ_ONLY_ENTRY);
        }
        CHECK(MC_OD_Write(0x2001, 3, 4, value, 0) == ABORTIDX_READ_ONLY_ENTRY);
        CHECK(MC_OD_Write(0x2010, 1, 1, value, 0) == ABORTIDX_READ_ONLY_ENTRY);
        CHECK(sent.empty());
        return 0;
    }
    if (!std::strcmp(argv[1], "single_fd")) {
        const dcu_can_channel_config_t fd{2, 1000000, 5000000, true, false};
        test_profile.channels = &fd;
        test_profile.channel_count = 1;
        test_profile.devices = nullptr;
        test_profile.device_count = 0;
        test_profile.raw_rate_limit[2] = 1;
        test_profile.readiness = {1, 1, {}, 1000, 5};
        CHECK(motor_control_init() && motor_control_boot_startup(17));
        select_wire_profile(1);
        CHECK(!switch_request(3) && !switch_action(1));
        select_wire_profile(3);
        CHECK(gRxAssign.aEntries[0] == 0x1641 && gTxAssign.aEntries[0] == 0x1A42);
        std::memset(ssc_txpdo + 88, 0xA5, sizeof(ssc_txpdo) - 88);
        motor_control_operational(true);
        mc_raw_t wire{};
        const auto submit = [&]() { motor_control_output(&wire); run_cycle(); };
        wire.BatchSeq = 1;
        wire.ValidMask = wire.SlotCount = 1;
        wire.Channel = 2;
        wire.CanId = 0x321;
        wire.FrameFlags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        wire.Dlc = 15;
        uint8_t payload[64];
        for (unsigned i = 0; i < 64; ++i) payload[i] = 0xD3 ^ i;
        memcpy(reinterpret_cast<uint8_t *>(&wire) + 24, payload, 64);
        submit();
        CHECK(sent.empty());
        wire.ValidMask = wire.SlotCount = 0;
        submit();
        CHECK(object<mc_raw_t>(0x7301)->BatchSeq == 1 && sent.empty());
        wire.BatchSeq = 2;
        wire.ValidMask = 2;
        wire.SlotCount = 1;
        submit();
        wire.ValidMask = 1;
        wire.SlotCount = 0;
        submit();
        CHECK(sent.empty());
        wire.SlotCount = 1;
        submit();
        submit();
        CHECK(sent.size() == 1 && sent[0].first == 2 && sent[0].second.dlc == 15);
        CHECK(!memcmp(sent[0].second.data, payload, sizeof(payload)));
        CHECK(!memcmp(object<mc_raw_t>(0x7301), &wire, 88));
        auto frame = sent[0].second;
        deliver(2, frame);
        ++frame.id;
        deliver(2, frame);
        for (unsigned i = 0; i < 2; ++i) {
            motor_control_prepare_input();
            check_txpdo_bytes();
            const auto &feedback = *object<mc_raw_t>(0x6401);
            CHECK(feedback.CanId == 0x321 + i && feedback.Channel == 2 && feedback.Dlc == 15);
            CHECK(feedback.FrameFlags == wire.FrameFlags && feedback.SlotCount == 1 && feedback.ValidMask == 1);
            CHECK(ssc_txpdo[23] == 0x34 && !memcmp(ssc_txpdo + 24, payload, 64));
        }
        motor_control_prepare_input();
        CHECK(ssc_txpdo[4] == 0 && ssc_txpdo[5] == 0);
        CHECK(!object<mc_board_rx_t>(0x7000)->CycleSeq && !object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq);
        test_ms += test_profile.max_command_lease_ms;
        submit();
        check_raw_state(RawState::Faulted);
        motor_control_prepare_input();
        CHECK(std::all_of(ssc_txpdo, ssc_txpdo + 88, [](uint8_t b) { return b == 0; }));
        CHECK(std::all_of(ssc_txpdo + 88, ssc_txpdo + sizeof(ssc_txpdo), [](uint8_t b) { return b == 0xA5; }));
        return 0;
    }
    const bool switching = !std::strncmp(argv[1], "switch_raw", 10);
    const bool raw_fixture = switching || !std::strcmp(argv[1], "raw_queue") || !std::strcmp(argv[1], "raw_full_admission") ||
                             !std::strncmp(argv[1], "raw_large_", 10);
    const bool full_fixture = !std::strcmp(argv[1], "raw_full_admission") || !std::strcmp(argv[1], "raw_large_full");
    if (raw_fixture) {
        // Exercise generic Raw CAN mechanics without giving the production Profile a Raw writer.
        dcu::app::gateway_application::detail::Configuration config{};
        config.board = {10, 20};
        for (auto &channel : config.channels) channel = {2, 0, 8, true};
        parameters::initialize({});
        CHECK(dcu::app::gateway_application::detail::initialize_fixture(config));
        CHECK(select_mode(full_fixture ? OperatingMode::Raw : OperatingMode::MotorWithRaw));
        test_profile.device_count = 0;
        CHECK(motor_pdo::bind_publication(test_profile));
    } else {
        if (!std::strcmp(argv[1], "profiles") || !std::strcmp(argv[1], "latched_layouts") ||
            !std::strncmp(argv[1], "raw_", 4))
            test_profile.device_count = 0;
        if (!std::strcmp(argv[1], "transaction_deadlines") || !std::strcmp(argv[1], "transaction_cancellation")) {
            two_bus_devices[0] = test_device;
            two_bus_devices[1] = test_device;
            two_bus_devices[1].bus = 1;
            two_bus_devices[1].id = "other_encos";
            test_profile.devices = two_bus_devices;
            test_profile.device_count = 2;
        }
        CHECK(motor_control_init());
    }
    select_wire_profile(full_fixture ? 2 : 1);
    if (switching) {
        test_raw_switch(!std::strcmp(argv[1], "switch_raw_timeout"), !std::strcmp(argv[1], "switch_raw_drops"));
        return 0;
    }
    uint8_t first[1072], second[1072];
    if (!std::strcmp(argv[1], "single_motor_raw_fault")) {
        start_axis();
        const auto steady_command = [&]() {
            const auto first = sent.size();
            command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
            for (unsigned n = 0; n < 2000 / DCU_WORKER_PERIOD_US + 2; ++n) run_cycle();
            motor_control_prepare_input();
            CHECK(axis_feedback(0)->StatusWord & 8U);
            for (auto i = first; i < sent.size(); ++i)
                if (sent[i].first == 0 && sent[i].second.id == 1 && sent[i].second.dlc == 8)
                    return sent[i].second;
            CHECK(false);
            return dcu_can_frame_t{};
        };
        const auto before = steady_command();
        const auto cancels = cancellations;
        CHECK(switch_action(2) == 0);
        const auto after = steady_command();
        CHECK(!std::memcmp(before.data, after.data, before.dlc) && before.flags == after.flags);
        CHECK(cancellations == cancels && !object<mc_sideband_t>(0x6400)->ValidMask);
        check_raw_state(RawState::Faulted);
        return 0;
    }
    if (!std::strcmp(argv[1], "input_staging")) {
        motor_control_operational(true);
        uint8_t wire[784]{};
        mc_board_rx_t board{};
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        board.CycleSeq = 1;
        board.CommandLeaseMs = 10;
        std::memcpy(wire, &board, sizeof(board));
        motor_control_output(wire);
        std::memset(wire, 0xFF, sizeof(wire));
        motor_control_process();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 0);
        run_cycle();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1);
        CHECK(!std::memcmp(object<mc_board_rx_t>(0x7000), &board, sizeof(board)));
        std::memset(wire, 0, sizeof(wire));
        ++board.CycleSeq;
        std::memcpy(wire, &board, sizeof(board));
        motor_control_output(wire);
        motor_control_process();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1);
        run_cycle();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 2);
        return 0;
    }
    if (!std::strcmp(argv[1], "application_authority")) {
        start_axis();
        const auto published = object<mc_board_tx_t>(0x6000)->PublishSeq;
        const auto accepted = object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq;
        object<mc_board_tx_t>(0x6000)->PublishSeq = UINT32_MAX;
        object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = 0x80000000;
        object<mc_owner_t>(0x2003)->Owner = OWNER_NONE;
        auto *config = object<mc_board_config_t>(0x2001);
        config->ActivePdoProfile = 2;
        config->MaxCommandLeaseMs = 1;
        config->AxisConfiguredMask = 0;
        object<mc_channel_config_t>(0x2010)->Role = 2;
        CHECK(owner() == OWNER_PDO && mode() == OperatingMode::MotorWithRaw);
        CHECK(!preop_access_allowed());
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == published + 1);
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == accepted);
        command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == accepted + 1);
        CHECK(axis_feedback(0)->StatusWord & 8U);
        CHECK(!axis_feedback(0)->BridgeFaultCode);
        return 0;
    }
    if (!std::strcmp(argv[1], "sdo_sequence_mirror")) {
        select_owner(OWNER_SDO);
        motor_control_operational(true);
        command_axis(0, false);
        command_axis(0);
        auto *command = object<mc_sdo_command_t>(0x2200);
        command->LeaseMs = 10;
        command->Mode = 1;
        command->AcceptedSeq = 1;
        command->CommandResult = RESULT_INVALID;
        uint32_t sequence = 1;
        CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(command->CommandResult == RESULT_OK && command->AcceptedSeq == 1);
        command->ControlWord = MC_AXIS_ARM;
        command->AcceptedSeq = UINT32_MAX;
        ++sequence;
        CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(command->CommandResult == RESULT_OK && command->AcceptedSeq == 2);
        command->ControlWord = MC_AXIS_STOP;
        command->AcceptedSeq = 0;
        CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(command->CommandResult == RESULT_OK && command->AcceptedSeq == 2);
        adapter::refresh_axis_feedback(0);
        CHECK(axis_feedback(0)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY);
        return 0;
    }
    if (!std::strcmp(argv[1], "transaction_deadlines")) {
        const auto dialect = dcu::motor::ParameterDialect::EncosLegacy;
        CHECK(parameters::start_query(1, 1, 1, 1, 20, dialect, 1, 0));
        parameters::check_timeout(1, 119);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 0);
        parameters::check_timeout(1, 120);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 0);
        CHECK(axis_diagnostics(7)->LastServiceError == RESULT_TIMEOUT);
        parameters::check_timeout(1, 121);
        CHECK(service_cancellations[1] == 0);

        CHECK(parameters::start_query(1, 1, 1, 1, 200, dialect, 1, 0));
        parameters::mark_submitted(1, 250, ++test_tx_serial);
        parameters::check_timeout(1, 300);
        CHECK(parameters::inspect(1).sent);
        parameters::check_timeout(1, 349);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 0);
        parameters::check_timeout(1, 350);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 1);

        CHECK(parameters::start_query(1, 1, 1, 1, 400, dialect, 1, 0));
        parameters::mark_submitted(1, UINT32_MAX - 50U, ++test_tx_serial);
        parameters::check_timeout(1, 48);
        CHECK(parameters::inspect(1).busy);
        parameters::check_timeout(1, 49);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 2);

        auto *transaction = object<mc_transaction_t>(axis_object_base(7) + 7);
        transaction->Operation = 2;
        transaction->ParameterId = 3;
        transaction->RequestSeq = 1;
        parameters::Request request{};
        request.device = 1;
        request.bus = 1;
        request.dialect = dialect;
        request.can_id = 1;
        request.sequence = 1;
        request.parameter = 3;
        request.opcode = 2;
        request.write_valid = true;
        request.write.dlc = 4;
        request.write.data[0] = 0xC0;
        request.write.data[1] = 4;
        request.write.data[3] = 50;
        CHECK(parameters::commit(request, 500, true, false).accepted);
        simulate_service_transmitted(1, 550);
        dcu_can_frame_t acknowledgement{};
        acknowledgement.id = 1;
        acknowledgement.dlc = 3;
        acknowledgement.data[0] = 0x80;
        acknowledgement.data[1] = 4;
        acknowledgement.data[2] = 1;
        CHECK(parameters::consume(1, 1, acknowledgement, 649).acknowledged);
        CHECK(parameters::pending(1) && !parameters::inspect(1).sent);
        parameters::check_timeout(1, 748);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 2);
        parameters::mark_submitted(1, 748, ++test_tx_serial);
        parameters::check_timeout(1, 847);
        CHECK(parameters::inspect(1).sent);
        parameters::check_timeout(1, 848);
        CHECK(transaction->DoneSeq == 1 && transaction->Result == RESULT_TIMEOUT);
        CHECK(service_cancellations[1] == 3);
        CHECK(service_cancellations[0] == 0 && service_cancellations[2] == 0);
        return 0;
    }
    if (!std::strcmp(argv[1], "transaction_cancellation")) {
        const auto dialect = dcu::motor::ParameterDialect::EncosLegacy;
        auto *transaction = object<mc_transaction_t>(axis_object_base(7) + 7);
        transaction->Operation = 1;
        transaction->ParameterId = 0x101;
        transaction->RequestSeq = 9;
        parameters::Request request{};
        request.device = 1;
        request.bus = 1;
        request.dialect = dialect;
        request.can_id = 1;
        request.sequence = 9;
        request.parameter = 0x101;
        request.opcode = 1;
        CHECK(parameters::commit(request, 0, true, false).accepted);
        parameters::abort_internal(1);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 0);
        parameters::mark_submitted(1, 2, ++test_tx_serial);
        parameters::cancel(1, RESULT_IO);
        CHECK(!parameters::inspect(1).busy && parameters::pending(1) == nullptr);
        CHECK(transaction->DoneSeq == 9 && transaction->Result == RESULT_IO);
        CHECK(service_cancellations[1] == 1);
        parameters::cancel(1, RESULT_STATE);
        CHECK(service_cancellations[1] == 1 && transaction->Result == RESULT_IO);
        dcu_can_frame_t late{};
        late.id = 1;
        late.dlc = 6;
        late.data[0] = 0xA0;
        late.data[1] = 1;
        CHECK(!parameters::consume(1, 1, late, 3).valid);
        CHECK(transaction->DoneSeq == 9 && transaction->Result == RESULT_IO);

        axis_diagnostics(0)->LastServiceError = 123;
        for (unsigned submitted = 0; submitted < 2; ++submitted) {
            CHECK(parameters::start_query(0, 0, 1, 1, 10, dialect, 1, 0));
            if (submitted) parameters::mark_submitted(0, 10, ++test_tx_serial);
            parameters::abort_internal(0);
            CHECK(!parameters::inspect(0).busy && service_cancellations[0] == submitted);
            CHECK(axis_diagnostics(0)->LastServiceError == 123);
        }
        transaction->RequestSeq = 10;
        request.sequence = 10;
        CHECK(parameters::commit(request, 10, true, false).accepted);
        CHECK(parameters::start_query(2, 14, 1, 1, 10, dialect, 1, 0));
        parameters::reset_session(RESULT_STATE);
        CHECK(transaction->DoneSeq == 10 && transaction->Result == RESULT_STATE);
        CHECK(service_cancellations[1] == 1 && service_cancellations[2] == 0);
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            CHECK(parameters::available(c, 10) && parameters::pending(c) == nullptr);
        return 0;
    }
    if (!std::strcmp(argv[1], "transaction_observation")) {
        const auto dialect = dcu::motor::ParameterDialect::EncosLegacy;
        axis_diagnostics(0)->QueriedPosition = -11;
        axis_diagnostics(0)->QueriedVelocity = -22;
        for (unsigned query = 1; query <= 2; ++query) {
            CHECK(parameters::start_query(0, 0, query, 1, query, dialect, 1, 0));
            simulate_service_transmitted(0, query);
            dcu_can_frame_t response{};
            response.id = 1;
            response.dlc = 6;
            response.data[0] = 0xA0;
            response.data[1] = query;
            response.data[2] = 0x3F;
            response.data[3] = 0x80;
            const auto observation = parameters::consume(0, 0, response, query);
            CHECK(observation.valid && observation.update_actual);
            CHECK(observation.field == (query == 1 ? parameters::Field::Position : parameters::Field::Velocity));
            CHECK(observation.value == (query == 1 ? 17453 : 104719));
            CHECK(object<mc_query_t>(0x3005)->ValidBits & (1u << (query - 1)));
            CHECK(axis_diagnostics(0)->QueriedPosition == -11 && axis_diagnostics(0)->QueriedVelocity == -22);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "codec")) {
        uint8_t output[784];
        for (unsigned i = 0; i < sizeof(output); ++i) output[i] = i * 17u;
        auto board = motor_pdo::board(output);
        CHECK(!std::memcmp(&board, output, sizeof(board)));
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            auto channel = motor_pdo::channel(output, c);
            auto bus = motor_pdo::raw_bus(output, c);
            CHECK(!std::memcmp(&channel, output + 32 + 8 * c, sizeof(channel)));
            CHECK(!std::memcmp(&bus, output + 32 + 128 * c, sizeof(bus)));
        }
        for (unsigned a = 0; a < 1; ++a) {
            auto axis = motor_pdo::axis(output, a);
            CHECK(!std::memcmp(&axis, output + 64 + 24 * a, sizeof(axis)));
        }
        auto raw = motor_pdo::raw(output);
        CHECK(!std::memcmp(&raw, output + 88, sizeof(raw)));
        std::memset(first, 0xA5, sizeof(first));
        std::memset(second, 0xA5, sizeof(second));
        mc_board_tx_t b{};
        std::memset(&b, 0x11, sizeof(b));
        motor_pdo::write_board(first, b);
        std::memcpy(second, &b, sizeof(b));
        mc_channel_tx_t ch{};
        std::memset(&ch, 0x22, sizeof(ch));
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            motor_pdo::write_channel(first, c, ch);
            std::memcpy(second + 32 + 8 * c, &ch, sizeof(ch));
        }
        mc_axis_tx_t axis{};
        std::memset(&axis, 0x33, sizeof(axis));
        for (unsigned a = 0; a < 1; ++a) {
            motor_pdo::write_axis(first, a, axis);
            std::memcpy(second + 64 + 32 * a, &axis, sizeof(axis));
        }
        motor_pdo::write_raw(first, raw);
        std::memcpy(second + 96, &raw, sizeof(raw));
        CHECK(!std::memcmp(first, second, sizeof(first)));
        for (unsigned i = 0; i < 2; ++i) {
            auto slot = motor_pdo::sideband_slot(raw, i);
            CHECK(!std::memcmp(&slot, reinterpret_cast<uint8_t *>(&raw) + 16 + 16 * i, sizeof(slot)));
        }
        return 0;
    }
    if (!std::strncmp(argv[1], "service_", 8)) {
        select_owner(OWNER_SDO);
        auto *transaction = object<mc_transaction_t>(0x3007);
        transaction->Operation = 1;
        uint32_t sequence = 1;
        if (!std::strcmp(argv[1], "service_extended")) {
            auto *snapshot = object<mc_query_t>(0x3005);
            dcu_can_frame_t response{};
            response.id = 1;
            transaction->ParameterId = 0x127;
            CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            ++test_ms;
            run_cycle();
            response.dlc = 7;
            response.data[0] = 0xA0; response.data[1] = 39; response.data[2] = 2;
            response.data[3] = 0xFE; response.data[4] = 0xDC;
            response.data[5] = 0x12; response.data[6] = 0x34;
            deliver(0, response);
            CHECK(snapshot->SnapshotSeq == 0 && !(snapshot->ValidBits & 0x1C0));
            response.data[2] = 1;
            deliver(0, response);
            CHECK(transaction->Result == RESULT_OK && transaction->DoneSeq == sequence);
            CHECK(snapshot->SnapshotSeq == 1 && (snapshot->ValidBits & 0x1C0) == 0x1C0);
            CHECK(snapshot->EncoderOutputEnabled == 1 && snapshot->EncoderOriginalRaw == -292);
            CHECK(snapshot->EncoderCalibratedRaw == 0x1234);
            return 0;
        }
        if (!std::strcmp(argv[1], "service_snapshot")) {
            transaction->Operation = 1;
            transaction->ParameterId = 0x101;
            CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            test_ms = 1;
            run_cycle();
            CHECK(parameters::inspect(0).sent);
            object<mc_routing_t>(0x3000)->CanId = 2;
            object<mc_routing_t>(0x3000)->ModelProfileId = 0;
            object<mc_channel_config_t>(0x2010)->FrameFormat = 1;
            dcu_can_frame_t response{};
            response.id = 1;
            response.dlc = 6;
            response.data[0] = 0xA0;
            response.data[1] = 1;
            response.data[2] = 0x3F;
            response.data[3] = 0x80;
            deliver(0, response);
            CHECK(transaction->DoneSeq == sequence && transaction->Result == RESULT_OK);
            CHECK(object<mc_query_t>(0x3005)->PositionDeg == 1.0F);
            CHECK(read_sdo<int32_t>(0x6200, 4) == 17453);
            return 0;
        }
        if (!std::strcmp(argv[1], "service_cache")) {
            transaction->Operation = 1;
            const auto begin_query = [&](uint8_t code) {
                transaction->ParameterId = 0x100 + code;
                ++sequence;
                CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
                ++test_ms;
                run_cycle();
                CHECK(parameters::inspect(0).sent);
            };
            dcu_can_frame_t response{};
            response.id = 1;
            response.data[0] = 0xA0;
            begin_query(23);
            response.dlc = 6; response.data[1] = 23;
            response.data[2] = 0xFE; response.data[3] = 0xDC;
            response.data[4] = 0x12; response.data[5] = 0x34;
            deliver(0, response);
            CHECK(transaction->DoneSeq == sequence && transaction->Result == RESULT_OK);
            CHECK(transaction->RawResponse[2] == 0xFE && transaction->RawResponse[5] == 0x34);
            CHECK(object<mc_range_t>(0x3002)->KpMin == 0);
            begin_query(29);
            auto *identity = object<mc_identity_t>(0x3006);
            for (uint8_t part : {2, 1, 3}) {
                response.dlc = 7;
                response.data[1] = 29;
                response.data[2] = part;
                response.data[3] = response.data[4] = response.data[5] = response.data[6] = part;
                deliver(0, response);
                if (part != 3) CHECK(!identity->UuidPart1 && parameters::inspect(0).busy);
            }
            CHECK(identity->UuidPart1 == 0x01010101 && identity->UuidPart2 == 0x02020202 && identity->UuidPart3 == 0x03030303);
            begin_query(30);
            response.dlc = 8; response.data[1] = 30;
            for (uint8_t i = 0; i < 6; ++i) response.data[i + 2] = i + 1;
            deliver(0, response);
            for (uint8_t i = 0; i < 6; ++i) CHECK(read_sdo<uint8_t>(0x3006, i + 4) == i + 1);
            auto *cache = object<mc_query_t>(0x3005);
            cache->PhaseCurrentA = 1;
            const auto before = *cache;
            begin_query(3);
            response.dlc = 6; response.data[1] = 3;
            response.data[2] = 0x44; response.data[3] = 0x7A; response.data[4] = response.data[5] = 0;
            deliver(0, response);
            CHECK(!std::memcmp(cache, &before, sizeof(before)));
            CHECK(parameters::inspect(0).busy && transaction->Result == RESULT_BUSY);
            return 0;
        }
        if (!std::strcmp(argv[1], "service_observations")) {
            const int32_t expected[] = {17453, 104719, 100, 1000};
            for (unsigned k = 1; k <= 4; ++k) {
                transaction->ParameterId = 0x100 + k;
                transaction->Operation = 1;
                sequence = k;
                CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
                test_ms = k;
                run_cycle();
                dcu_can_frame_t response{};
                response.id = 1;
                response.dlc = 6;
                response.data[0] = 0xA0;
                response.data[1] = k;
                response.data[2] = 0x3F;
                response.data[3] = 0x80;
                deliver(0, response);
                CHECK(transaction->Result == RESULT_OK && transaction->DoneSeq == sequence);
                (void)read_sdo<uint16_t>(0x6200, 1);
                auto *feedback = axis_feedback(0);
                int32_t actual = k == 1 ? feedback->ActualPosition : k == 2 ? feedback->ActualVelocity :
                    k == 3 ? feedback->ActualCurrent : feedback->ActualPower;
                CHECK(actual == expected[k - 1]);
                if (k == 1) CHECK(axis_diagnostics(0)->QueriedPosition == expected[0]);
                if (k == 2) CHECK(axis_diagnostics(0)->QueriedVelocity == expected[1]);
            }
            return 0;
        }
        if (!std::strcmp(argv[1], "service_rejected")) {
            const struct { uint8_t op; uint16_t parameter; } rejected[] = {{2, 3}, {1, 0x124}};
            for (auto entry : rejected) {
                transaction->Operation = entry.op;
                transaction->ParameterId = entry.parameter;
                CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) ==
                      ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
                CHECK(transaction->Result == RESULT_UNSUPPORTED && sent.empty() && !parameters::inspect(0).busy);
                ++sequence;
            }
            return 0;
        }
        transaction->ParameterId = 0x101;
        CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence),
                          reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(transaction->Result == RESULT_BUSY);
        if (!std::strcmp(argv[1], "service_cancel")) {
            uint16_t requested = OWNER_PDO;
            CHECK(MC_OD_Write(0x2003, 2, 1, &requested, 0) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
            CHECK(object<mc_owner_t>(0x2003)->Owner == OWNER_SDO);
            motor_control_operational(false);
            CHECK(transaction->DoneSeq == sequence && transaction->Result == RESULT_STATE);
            CHECK(!parameters::inspect(0).busy);
            return 0;
        }
        test_ms = 1;
        run_cycle();
        CHECK(parameters::inspect(0).sent);
        if (!std::strcmp(argv[1], "service_timeout")) {
            test_ms = 101;
            run_cycle();
            CHECK(!parameters::inspect(0).busy && transaction->Result == RESULT_TIMEOUT);
            CHECK(transaction->DoneSeq == sequence);
            return 0;
        }
        CHECK(false);
    }
    if (!std::strcmp(argv[1], "raw_queue")) {
        motor_control_operational(true);
        object<mc_channel_config_t>(0x2010)->Role = 2;
        dcu_can_frame_t frame{};
        for (unsigned i = 0; i < raw_cfg::queue_capacity + 3; ++i) {
            frame.id = i;
            incoming[0].emplace_back(frame, dcu_can_rx_stamp_t{});
        }
        run_cycle();
        run_cycle();
        Publication snapshot{};
        prepare_publication(snapshot);
        CHECK(snapshot.channels[0].raw_drops == 4);
        for (unsigned i = 0; i < raw_cfg::queue_capacity - 1; ++i) {
            CHECK(take_raw_frame(0, frame) && frame.id == i);
        }
        CHECK(!take_raw_frame(0, frame));
        return 0;
    }
    if (!std::strcmp(argv[1], "same_ms")) {
        motor_control_prepare_input();
        const uint32_t sequence = object<mc_board_tx_t>(0x6000)->PublishSeq;
        copy_txpdo(first);
        copy_txpdo(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == sequence);
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == sequence + 1U);
        test_ms = 1;
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == sequence + 2U);
        return 0;
    }
    if (!std::strcmp(argv[1], "immutable")) {
        test_ms = 99;
        start_axis();
        deliver(0, type1_feedback());
        test_ms = 100;
        motor_control_prepare_input();
        CHECK(axis_feedback(0)->StatusWord & 16U);
        CHECK(axis_feedback(0)->FeedbackAgeMs == 1U);
        copy_txpdo(first);
        axis_feedback(0)->ActualPosition = 12345;
        copy_txpdo(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & 16U));
        copy_txpdo(second);
        CHECK(std::memcmp(first, second, test_input_size) != 0);
        mc_axis_tx_t axis;
        std::memcpy(&axis, second + 64, sizeof(axis));
        mc_axis_tx_t before;
        std::memcpy(&before, first + 64, sizeof(before));
        CHECK(axis.ActualPosition == before.ActualPosition && axis.FeedbackAgeMs == 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "profiles")) {
        CHECK(boot_startup({1, 1, {}, 1000, 5}, 1));
        for (unsigned profile : {0U, 1U, 2U, 3U, 1U, 3U, 2U}) {
            request_test_layout(profile);
            std::memset(ssc_txpdo + test_input_size, 0xA5, sizeof(ssc_txpdo) - test_input_size);
            motor_control_prepare_input();
            check_txpdo_bytes();
            mc_board_tx_t board;
            std::memcpy(&board, ssc_txpdo, sizeof(board));
            if (profile != 3) CHECK(board.PublishSeq == object<mc_board_tx_t>(0x6000)->PublishSeq);
            for (unsigned i = test_input_size; i < sizeof(ssc_txpdo); ++i)
                CHECK(ssc_txpdo[i] == 0xA5);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "latched_layouts")) {
        CHECK(boot_startup({1, 1, {}, 1000, 5}, 1));
        motor_control_operational(true);
        const auto raw_offset = motor_pdo::layout::active().sideband(true);
        motor_control_prepare_input();
        check_txpdo_bytes();
        CHECK(!std::memcmp(ssc_txpdo + raw_offset, "T1R2", 4));
        const auto first_page = ssc_txpdo[raw_offset + 4];
        motor_control_prepare_input();
        CHECK(ssc_txpdo[raw_offset + 4] == (first_page ^ 1U));
        const auto active = readiness();
        CHECK(request_layout(OperatingMode::Raw, active.profile_id, active.revision, active.digest));
        motor_control_operational(false);
        CHECK(reset_layout());
        select_wire_profile(2);
        motor_control_prepare_input();
        check_txpdo_bytes();
        request_test_layout(1);
        const mc_sideband_t cleared{};
        CHECK(!std::memcmp(ssc_txpdo + raw_offset, &cleared, sizeof(cleared)));
        motor_control_operational(true);
        motor_control_prepare_input();
        check_txpdo_bytes();
        CHECK(!std::memcmp(ssc_txpdo + raw_offset, "T1R2", 4));
        return 0;
    }
    if (!std::strcmp(argv[1], "sdo_publication")) {
        start_axis();
        copy_txpdo(first);
        const uint32_t sequence = object<mc_board_tx_t>(0x6000)->PublishSeq;
        deliver(0, type1_feedback());
        const auto count = read_sdo<uint32_t>(0x2400, 4);
        CHECK(count >= 2);
        CHECK(read_sdo<uint16_t>(0x6200, 1) & 16U);
        CHECK(read_sdo<uint16_t>(0x6200, 1) & 16U);
        CHECK(read_sdo<uint32_t>(0x2400, 4) == count);
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == sequence);
        copy_txpdo(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        motor_control_prepare_input();
        copy_txpdo(first);
        mc_axis_tx_t snapshot{};
        std::memcpy(&snapshot, first + 64, sizeof(snapshot));
        CHECK(snapshot.StatusWord & 16U);
        CHECK(!(read_sdo<uint16_t>(0x6200, 1) & 16U));
        copy_txpdo(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & 16U));
        return 0;
    }
    if (!std::strcmp(argv[1], "accepted_pdo")) {
        start_axis();
        const auto accepted = *object<mc_axis_rx_t>(0x7200);
        CHECK(accepted.ControlWord == (MC_AXIS_ARM | MC_AXIS_START_MOTION));
        CHECK(accepted.Mode == 1 && accepted.CurrentLimit == 50);
        command_axis(MC_AXIS_STOP);
        CHECK(!std::memcmp(&accepted, object<mc_axis_rx_t>(0x7200), sizeof(accepted)));
        return 0;
    }
    if (!std::strcmp(argv[1], "accepted_sdo")) {
        select_owner(OWNER_SDO);
        motor_control_operational(true);
        command_axis(0, false);
        command_axis(0);
        auto *command = object<mc_sdo_command_t>(0x2200);
        command->LeaseMs = 10;
        command->Mode = 1;
        command->CurrentLimit = 50;
        for (uint32_t sequence = 1; sequence <= 2; ++sequence) {
            command->ControlWord = sequence == 1 ? 0 : MC_AXIS_ARM;
            CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            CHECK(command->CommandResult == RESULT_OK && command->AcceptedSeq == sequence);
        }
        CHECK(object<mc_axis_rx_t>(0x7200)->ControlWord == 0);
        size_t read = 0;
        for (unsigned n = 0; n < 80 && !(read_sdo<uint16_t>(0x6200, 1) & 4U); ++n) {
            run_cycle();
            while (read < sent.size()) {
                const auto [bus, request] = sent[read++];
                if (request.dlc != 2 || request.data[1] > 3) continue;
                dcu_can_frame_t response{};
                response.id = request.id;
                response.dlc = 6;
                response.data[0] = 0xA0;
                response.data[1] = request.data[1];
                deliver(bus, response);
            }
        }
        CHECK(read_sdo<uint16_t>(0x6200, 1) & 4U);
        uint32_t sequence = 3;
        command->ControlWord = MC_AXIS_ARM | MC_AXIS_START_MOTION;
        command->TargetPosition = 1000;
        CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(command->CommandResult == RESULT_OK && command->AcceptedSeq == sequence);
        const auto accepted = *object<mc_axis_rx_t>(0x7200);
        CHECK(!std::memcmp(&accepted, command, sizeof(accepted)));
        sequence = 4;
        command->Mode = 2;
        command->TargetPosition = 0;
        CHECK(MC_OD_Write(0x2200, 11, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        CHECK(command->CommandResult == RESULT_INVALID && command->AcceptedSeq == 3);
        CHECK(!std::memcmp(&accepted, object<mc_axis_rx_t>(0x7200), sizeof(accepted)));
        CHECK(read_sdo<uint8_t>(0x6200, 11) == 8);
        return 0;
    }
    if (!std::strcmp(argv[1], "cycle_fault")) {
        start_axis();
        uint8_t pending[784]{};
        motor_control_output(pending);
        motor_control_cycle_fault();
        check_raw_state(RawState::Faulted);
        CHECK(cancellations != 0);
        const size_t before = sent.size();
        run_cycle();
        CHECK(sent.size() == before);
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & (8U | MC_STATUS_ARMED_QUERY_ONLY)));
        CHECK(object<mc_board_tx_t>(0x6000)->BoardStatus & 0x40U);
        CHECK(!(object<mc_board_tx_t>(0x6000)->BoardStatus & 2U));
        return 0;
    }
    if (!std::strcmp(argv[1], "invalid_stop")) {
        start_axis();
        uint8_t command[784] = {};
        mc_board_rx_t board{};
        board.BoardControl = 2U;
        board.InterfaceVersion = 0; /* Even invalid PDO must not suppress Stop. */
        std::memcpy(command, &board, sizeof(board));
        motor_control_output(command);
        CHECK(axis_feedback(0)->StatusWord & 8U);
        run_cycle();
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & (8U | MC_STATUS_ARMED_QUERY_ONLY)));
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_objects")) {
        dcu::motor::MotorInstance first_axis, second_axis;
        CHECK(bind_test_device(first_axis, 0) && bind_test_device(second_axis, 1));
        dcu::motor::Command intent{};
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        intent.control_bits = MC_AXIS_ARM;
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        first_axis.stop(3);
        dcu::motor::MotorFeedback first_feedback{}, second_feedback{};
        dcu::motor::MotorDiagnostics first_diagnostics{}, second_diagnostics{};
        first_axis.read_feedback(0, 20, first_feedback, first_diagnostics);
        second_axis.read_feedback(0, 20, second_feedback, second_diagnostics);
        CHECK(!(first_feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY));
        CHECK(second_feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(first_feedback.bridge_fault == 3 && !second_feedback.bridge_fault);
        first_axis.disable();
        CHECK(!first_axis.configured() && second_axis.configured());
        CHECK(!bind_test_device(first_axis, 0, 2));
        CHECK(!first_axis.configured());
        CHECK(!read_parameter(ENCOS_AXIS_COUNT, {}, 0, true).accepted);
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_phases")) {
        dcu::motor::MotorInstance axis;
        CHECK(bind_test_device(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Velocity;
        command.current_limit_dA = 50;
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        dcu_can_frame_t frame{};
        const auto status = [&]() {
            axis.read_feedback(1, 20, feedback, diagnostics);
            return feedback.status_word;
        };
        CHECK(!axis.active() && !axis.control_sent() && !axis.encode_control(frame));
        axis.control_submitted(1, 1);
        CHECK(!axis.control_sent() && !(status() & MC_STATUS_ARMED_QUERY_ONLY));
        command.control_bits = MC_AXIS_ARM;
        CHECK(!axis.accept(command, true, 1));
        command.control_bits = 0;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM | MC_AXIS_START_MOTION;
        CHECK(!axis.accept(command, true, 1)); // ARM and StartMotion cannot share an edge.
        command.control_bits = 0;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        CHECK(!axis.active() && !axis.control_sent() && (status() & MC_STATUS_ARMED_QUERY_ONLY));
        for (uint8_t code = 1; code <= 3; ++code) query_observation(axis, code, 0);
        command.control_bits |= MC_AXIS_START_MOTION;
        CHECK(axis.accept(command, true, 1));
        CHECK(axis.active() && !axis.control_sent() && axis.encode_control(frame));
        axis.consume_feedback(type1_feedback(), 1, 1);
        CHECK(!(status() & 8U)); // No first submission yet: unsolicited Type-1 cannot start motion.
        axis.check_timeouts(21, false, 20);
        CHECK(axis.active() && !axis.control_sent());
        axis.control_submitted(21, 1);
        CHECK(axis.control_sent() && feedback.mode == dcu::motor::ControlMode::Position);
        axis.check_timeouts(40, false, 20);
        CHECK(axis.active());
        axis.consume_feedback(type1_feedback(), 40, 40);
        axis.control_submitted(40, 1);
        axis.read_feedback(40, 20, feedback, diagnostics);
        CHECK((feedback.status_word & 8U) && !(feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY));
        CHECK(feedback.mode == dcu::motor::ControlMode::Velocity);
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 40));
        CHECK(!axis.active() && !axis.control_sent() && (status() & MC_STATUS_ARMED_QUERY_ONLY));
        command.control_bits |= MC_AXIS_START_MOTION;
        CHECK(!axis.accept(command, true, 40)); // Running cleared the old query sample validity.
        axis.stop(0);
        CHECK(!axis.encode_control(frame));
        CHECK(!(status() & (8U | MC_STATUS_ARMED_QUERY_ONLY)));
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_startup_deadlines")) {
        dcu::motor::MotorInstance axis;
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        for (unsigned variant = 0; variant < 2; ++variant) {
            CHECK(bind_test_device(axis, 0));
            dcu::motor::Command command{};
            command.mode = dcu::motor::ControlMode::Position;
            command.current_limit_dA = 50;
            CHECK(axis.accept(command, true, 1));
            command.control_bits = MC_AXIS_ARM;
            CHECK(axis.accept(command, true, 1));
            for (uint8_t code = 1; code <= 3; ++code) query_observation(axis, code, 0);
            command.control_bits |= MC_AXIS_START_MOTION;
            CHECK(axis.accept(command, true, 1));
            if (variant == 0) {
                axis.check_startup(1001);
            } else {
                axis.control_submitted(UINT32_MAX - 5, 1);
                axis.check_timeouts(13, false, 20);
                CHECK(axis.active());
                axis.check_timeouts(14, false, 20);
            }
            axis.read_feedback(1001, 20, feedback, diagnostics);
            CHECK(!axis.active() && !axis.control_sent());
            CHECK(feedback.bridge_fault == (variant == 0 ? 8 : 4));
            CHECK(!axis.accept(command, true, 1001));
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_rearm")) {
        dcu::motor::MotorInstance axis;
        CHECK(bind_test_device(axis, 0));
        dcu::motor::Command command{};
        CHECK(axis.accept(command, true, 0));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 0));
        axis.stop(0);
        CHECK(!axis.accept(command, true, 0));
        command.control_bits = MC_AXIS_STOP;
        CHECK(axis.accept(command, true, 0));
        command.control_bits = MC_AXIS_ARM;
        CHECK(!axis.accept(command, true, 0));
        command.control_bits = 0;
        CHECK(axis.accept(command, false, 0));
        command.control_bits = MC_AXIS_ARM;
        CHECK(!axis.accept(command, true, 0));
        command.control_bits = 0;
        CHECK(axis.accept(command, true, 0));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 0));
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        axis.read_feedback(0, 20, feedback, diagnostics);
        CHECK(feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(!axis.active() && !axis.control_sent());
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_config_snapshot")) {
        dcu::motor::MotorInstance axis;
        CHECK(bind_test_device(axis, 0));
        CHECK(parameters::start_query(0, 0, 3, 1, 1, dcu::motor::ParameterDialect::EncosLegacy, 1, 0));
        simulate_service_transmitted(0, 1);
        dcu_can_frame_t response{};
        response.id = 1;
        response.dlc = 6;
        response.data[0] = 0xA0;
        response.data[1] = 3;
        response.data[2] = 0x3F;
        response.data[3] = 0x80;
        axis.consume_feedback(response, 1, 1);
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        axis.read_feedback(1, 20, feedback, diagnostics);
        const auto torque = feedback.sample.torque_mNm;
        CHECK(torque > 0 && feedback.sample.current_cA == 100);
        auto changed = axis_configuration();
        changed.route.can_id = 2;
        changed.encos = {};
        axis.read_feedback(1, 20, feedback, diagnostics);
        CHECK(feedback.sample.torque_mNm == torque);
        CHECK(axis.matches(1) && !axis.matches(2));
        axis.read_feedback(2 + dcu::motor::encos_runtime::query_current_ms, 20, feedback, diagnostics);
        CHECK(feedback.sample.torque_mNm == torque && !(feedback.sample.valid_fields & 8U));
        CHECK(!axis.bind(0, changed, 0));
        CHECK(!axis.configured());
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_runtime_authority")) {
        dcu::motor::MotorInstance axis;
        CHECK(bind_test_device(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Position;
        command.current_limit_dA = 50;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        query_observation(axis, 1, 0x3F800000);
        query_observation(axis, 2, 0);
        query_observation(axis, 3, 0);
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        CHECK(axis.read_feedback(1, 20, feedback, diagnostics).online);
        CHECK(diagnostics.queried_position_urad == 17453 && !diagnostics.queried_velocity_urad_s);
        CHECK(axis.query_ready(1));
        axis_feedback(0)->ActualCurrent = INT16_MAX;
        axis_feedback(0)->MotorFaultRaw = 7;
        axis_feedback(0)->BridgeFaultCode = 9;
        axis_diagnostics(0)->QueriedPosition = INT32_MAX;
        axis_diagnostics(0)->QueriedVelocity = INT32_MAX;
        feedback.sample.current_cA = INT16_MAX;
        diagnostics.queried_position_urad = INT32_MAX;
        CHECK(axis.query_ready(1));
        command.control_bits |= MC_AXIS_START_MOTION;
        command.position_urad = 17453;
        CHECK(axis.accept(command, true, 1) && axis.active());
        axis.stop(3);
        axis_feedback(0)->BridgeFaultCode = 0;
        axis_diagnostics(0)->ActiveFaultBits = 0;
        CHECK(!axis.query_ready(1));
        CHECK(axis.read_feedback(1, 20, feedback, diagnostics).fault);
        CHECK(feedback.bridge_fault == 3 && diagnostics.active_fault_bits == (1U << 3));
        CHECK(diagnostics.queried_position_urad == 17453 && !diagnostics.queried_velocity_urad_s);
        command.control_bits = MC_AXIS_RESET;
        CHECK(axis.accept(command, true, 1));
        CHECK(!axis.read_feedback(1, 20, feedback, diagnostics).fault);
        CHECK(!feedback.bridge_fault && !diagnostics.active_fault_bits);
        CHECK(axis.query_ready(1));
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_publication")) {
        auto config = axis_configuration();
        config.power_query_period_ms = 100;
        dcu::motor::MotorInstance axis;
        CHECK(axis.bind(0, config, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Position;
        command.current_limit_dA = 50;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        for (uint8_t code = 1; code <= 4; ++code) query_observation(axis, code, 0);
        command.control_bits |= MC_AXIS_START_MOTION;
        CHECK(axis.accept(command, true, 1));
        axis.control_submitted(1, 1);
        axis.consume_feedback(type1_feedback(), 1, 1);
        dcu::motor::MotorFeedback feedback{}, repeated{};
        dcu::motor::MotorDiagnostics diagnostics{};
        diagnostics.tx_drop_count = 88;
        axis.record_tx_drop();
        const dcu::motor::MotorInstance &view = axis;
        view.read_feedback(1, 20, feedback, diagnostics);
        view.read_feedback(1, 20, repeated, diagnostics);
        CHECK(same_feedback(feedback, repeated));
        CHECK((feedback.status_word & (16U | 1024U)) == (16U | 1024U));
        CHECK(diagnostics.tx_drop_count == 1);
        CHECK(diagnostics.tx_enqueue_count == 1 && diagnostics.rx_valid_count == 5);
        axis.acknowledge_publication();
        view.read_feedback(1, 20, repeated, diagnostics);
        CHECK(!(repeated.status_word & (16U | 1024U)));
        CHECK((feedback.status_word & (16U | 1024U)) == (16U | 1024U));
        feedback.status_word &= ~(16U | 1024U);
        CHECK(same_feedback(feedback, repeated));
        view.read_feedback(21, 20, repeated, diagnostics);
        CHECK(!(repeated.sample.valid_fields & 63U));
        view.read_feedback(21, 30, repeated, diagnostics);
        CHECK((repeated.sample.valid_fields & 7U) == 7U);
        axis.consume_feedback(type1_feedback(), 22, 22);
        view.read_feedback(22, 20, repeated, diagnostics);
        CHECK((repeated.status_word & (16U | 1024U)) == 16U);
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_history")) {
        dcu::motor::MotorInstance axis;
        CHECK(bind_test_device(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Position;
        command.current_limit_dA = 50;
        command.position_urad = 17453;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        query_observation(axis, 1, 0x3F800000);
        query_observation(axis, 2, 0);
        query_observation(axis, 3, 0);
        command.control_bits |= MC_AXIS_START_MOTION;
        CHECK(axis.accept(command, true, 1));
        axis.control_submitted(1, 1);
        dcu_can_frame_t invalid{};
        invalid.id = 1;
        invalid.dlc = 8;
        axis.consume_feedback(invalid, 1, 1);
        axis.stop(4);
        dcu::motor::MotorFeedback feedback{};
        dcu::motor::MotorDiagnostics diagnostics{};
        axis.read_feedback(1, 20, feedback, diagnostics);
        CHECK(diagnostics.tx_enqueue_count == 1 && diagnostics.rx_valid_count == 3);
        CHECK(diagnostics.protocol_error_count == 1 && diagnostics.active_fault_bits == (1U << 4));
        CHECK(bind_test_device(axis, 0));
        axis.read_feedback(2, 20, feedback, diagnostics);
        CHECK(diagnostics.tx_enqueue_count == 1 && diagnostics.rx_valid_count == 3);
        CHECK(diagnostics.protocol_error_count == 1 && diagnostics.active_fault_bits == (1U << 4));
        CHECK(diagnostics.queried_position_urad == 17453 && diagnostics.position_query_age_us == UINT32_MAX);
        CHECK(!feedback.bridge_fault && !feedback.sample.position_urad && !feedback.sample.valid_fields);
        CHECK(feedback.feedback_age_ms == UINT16_MAX && feedback.power_age_ms == UINT16_MAX);
        CHECK(!axis.query_ready(2));
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_codec")) {
        mc_sideband_t raw{};
        dcu::app::RawCanBatch batch{};
        raw.BatchSeq = UINT32_MAX;
        raw.SlotCount = 2;
        for (unsigned i = 0; i < 2; ++i) {
            motor_pdo::SidebandSlot slot{};
            slot.id = 0x100 + i;
            slot.channel = i + 2;
            slot.flags = 2 + i;
            slot.dlc = 9 + i;
            for (unsigned j = 0; j < 8; ++j) slot.data[j] = i * 19 + j;
            motor_pdo::write_sideband_slot(raw, i, slot);
        }
        for (unsigned mask = 0; mask < 256; ++mask) {
            raw.ValidMask = mask;
            motor_pdo::decode_raw_sideband(raw, batch);
            CHECK(batch.sequence == UINT32_MAX && batch.kind == dcu::app::RawCanBatch::Kind::Sideband);
            CHECK(batch.valid == (mask < 4));
            if (!batch.valid) { CHECK(batch.count == 0); continue; }
            unsigned count = 0;
            for (unsigned i = 0; i < 2; ++i) {
                if (!(mask & (1u << i))) continue;
                const auto wire = motor_pdo::sideband_slot(raw, i);
                const auto &slot = batch.slots[count++];
                CHECK(slot.bus == wire.channel && slot.frame.id == wire.id);
                CHECK(slot.frame.flags == wire.flags && slot.frame.dlc == wire.dlc);
                CHECK(!std::memcmp(slot.frame.data, wire.data, 8));
                for (unsigned b = sizeof(wire.data); b < sizeof(slot.frame.data); ++b)
                    CHECK(slot.frame.data[b] == 0);
            }
            CHECK(batch.count == count);
        }
        raw.ValidMask = 3;
        raw.SlotCount = 0;
        motor_pdo::decode_raw_sideband(raw, batch);
        CHECK(batch.count == 2);
        for (unsigned which = 0; which < 6; ++which) {
            auto invalid = raw;
            switch (which) {
            case 0: invalid.SlotCount = 3; break;
            case 1: invalid.Flags = 1; break;
            case 2: invalid.DropCount = 1; break;
            case 3: invalid.Reserved = 1; break;
            case 4: invalid.Reserved1 = 1; break;
            case 5: invalid.Reserved2 = 1; break;
            }
            motor_pdo::decode_raw_sideband(invalid, batch);
            CHECK(!batch.valid && batch.count == 0 && batch.sequence == UINT32_MAX);
        }
        canfd_bus_pdo_t full{};
        full.cycle_seq = UINT16_MAX;
        for (unsigned i = 0; i < 7; ++i) {
            full.slot[i].can_id = 0x100 + i;
            full.slot[i].flags = i & 1 ? 25 : 0;
            full.slot[i].dlc = 8 - i;
            for (unsigned j = 0; j < 8; ++j) full.slot[i].data[j] = i * 19 + j;
        }
        for (unsigned mask = 0; mask < 256; ++mask) {
            full.valid_mask = mask;
            motor_pdo::decode_raw_bus(full, 3, batch);
            CHECK(batch.sequence == UINT16_MAX && batch.kind == dcu::app::RawCanBatch::Kind::FullBus);
            CHECK(batch.bus == 3 && batch.valid == (mask < 128));
            if (!batch.valid) { CHECK(batch.count == 0); continue; }
            unsigned count = 0;
            for (unsigned i = 0; i < 7; ++i) {
                if (!(mask & (1u << i)) || !(full.slot[i].flags & 1)) continue;
                const auto &slot = batch.slots[count++];
                CHECK(slot.bus == 3 && slot.frame.id == full.slot[i].can_id);
                CHECK(slot.frame.flags == 24 && slot.frame.dlc == full.slot[i].dlc);
                CHECK(!std::memcmp(slot.frame.data, full.slot[i].data, 8));
            }
            CHECK(batch.count == count);
        }
        full.valid_mask = 1;
        for (unsigned flags = 0; flags < 256; ++flags) {
            full.slot[0].flags = flags;
            motor_pdo::decode_raw_bus(full, 0, batch);
            CHECK(batch.valid && batch.count == (flags & 1));
            if (batch.count) CHECK(batch.slots[0].frame.flags == (flags & ~33u));
        }
        motor_pdo::decode_raw_bus(full, 256, batch);
        CHECK(!batch.valid);
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_large_sideband") || !std::strcmp(argv[1], "raw_large_full")) {
        const bool full = !std::strcmp(argv[1], "raw_large_full");
        RawPdoFixture input(full ? 2 : 1);
        input.start();
        if (full) {
            input.full[0].slot[0].flags = CANFD_FLAG_VALID | DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
            input.full[0].slot[0].dlc = 15;
            for (unsigned c = 1; c < ENCOS_CHANNEL_COUNT; ++c) input.full[c].valid_mask = 0;
        } else {
            auto slot = motor_pdo::sideband_slot(input.sideband, 0);
            slot.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
            slot.dlc = 15;
            motor_pdo::write_sideband_slot(input.sideband, 0, slot);
            input.sideband.BatchSeq = 1;
        }
        dcu_can_frame_t rx{};
        rx.id = 0x201;
        rx.dlc = 15;
        rx.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        std::memset(rx.data, 0xa5, sizeof(rx.data));
        incoming[0].emplace_back(rx, dcu_can_rx_stamp_t{});
        rx.id = 0x202;
        rx.dlc = 8;
        incoming[0].emplace_back(rx, dcu_can_rx_stamp_t{});
        input.submit();
        CHECK(sent.empty());
        const auto *board = object<mc_board_tx_t>(0x6000);
        CHECK(board->RxDropCount == 1 && board->TxDropCount == 1);
        check_txpdo_bytes();
        if (full) {
            const auto *feedback = object<canfd_bus_pdo_t>(0x6500);
            CHECK(feedback->valid_mask == 1 && feedback->counter0 == 1);
            CHECK(feedback->slot[0].can_id == rx.id && feedback->slot[0].dlc == 8);
            CHECK(!std::memcmp(feedback->slot[0].data, rx.data, sizeof(feedback->slot[0].data)));
        } else {
            const auto *feedback = object<mc_sideband_t>(0x6400);
            CHECK(feedback->ValidMask == 1 && feedback->DropCount == 1);
            const auto slot = motor_pdo::sideband_slot(*feedback, 0);
            CHECK(slot.id == rx.id && slot.dlc == 8);
            CHECK(!std::memcmp(slot.data, rx.data, sizeof(slot.data)));
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_applied_policy")) {
        RawPdoFixture input(1);
        input.start();
        object<mc_channel_config_t>(0x2010)->Role = 2;
        input.sideband.BatchSeq = 1;
        input.submit();
        CHECK(sent.empty());
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_full_admission")) {
        RawPdoFixture input(2);
        input.start();
        input.full[0].cycle_seq = UINT16_MAX;
        input.full[0].slot[0].can_id = 0x7FF;
        input.full[1].cycle_seq = 5;
        input.full[1].slot[0].can_id = 1;
        input.full[2].valid_mask = input.full[3].valid_mask = 0;
        input.submit();
        CHECK(sent.size() == 2 && std::get<1>(sent.front()).id == 0x7FF);
        input.full[0].slot[0].data[0] = 77;
        input.full[1].cycle_seq = 6;
        input.submit();
        CHECK(sent.size() == 3 && std::get<0>(sent.back()) == 1);
        CHECK(object<canfd_bus_pdo_t>(0x7400)->slot[0].data[0] == 77);
        input.full[0].cycle_seq = 0;
        input.submit();
        CHECK(sent.size() == 4);
        input.full[0].cycle_seq = UINT16_MAX;
        input.submit();
        CHECK(sent.size() == 5);
        input.full[0].cycle_seq = 1;
        input.full[0].valid_mask = 0x80;
        input.submit();
        input.full[0].valid_mask = 1;
        input.submit();
        CHECK(sent.size() == 5);
        input.full[0].cycle_seq = 2;
        input.submit();
        CHECK(sent.size() == 6);
        input.full[0].cycle_seq = 3;
        input.submit(false);
        CHECK(sent.size() == 6 && object<canfd_bus_pdo_t>(0x7400)->cycle_seq == 2);
        input.submit();
        CHECK(sent.size() == 7 && object<canfd_bus_pdo_t>(0x7400)->cycle_seq == 3);
        input.full[0].cycle_seq = 4;
        input.submit(true, 15, false);
        CHECK(sent.size() == 7 && object<canfd_bus_pdo_t>(0x7400)->cycle_seq == 3);
        input.submit();
        CHECK(sent.size() == 8);
        run_cycle();
        CHECK(sent.size() == 8);
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_objects")) {
        dcu::app::RawCanGateway first_gateway, second_gateway;
        dcu_can_frame_t frame{};
        frame.id = 101;
        first_gateway.receive(0, frame);
        CHECK(!second_gateway.take(0, frame));
        CHECK(first_gateway.take(0, frame) && frame.id == 101);
        for (unsigned n = 0; n < raw_cfg::queue_capacity + 1; ++n)
            first_gateway.receive(0, frame);
        CHECK(first_gateway.rx_drops(0) == 2 && second_gateway.rx_drops(0) == 0);
        CHECK(!first_gateway.take(ENCOS_CHANNEL_COUNT, frame));
        return 0;
    }
    if (!std::strncmp(argv[1], "raw_", 4)) {
        dcu::app::RawCanGateway gateway;
        dcu::app::BusScheduler budgets[ENCOS_CHANNEL_COUNT];
        dcu::app::RawCanPolicy policy{};
        policy.permitted = true;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            budgets[c].reset_budget(0);
            policy.buses[c].can_send = c == 0;
            policy.buses[c].rate_limit = 1;
        }
        auto *raw = object<mc_sideband_t>(0x7300);
        *raw = {};
        raw->ValidMask = 1;
        raw->SlotCount = 1;
        motor_pdo::SidebandSlot slot{};
        slot.id = 0x100;
        slot.dlc = 1;
        slot.data[0] = 42;
        motor_pdo::write_sideband_slot(*raw, 0, slot);
        const auto process_sideband = [&](uint32_t now) {
            dcu::app::RawCanBatch batch{};
            motor_pdo::decode_raw_sideband(*raw, batch);
            return gateway.process(batch, policy, now, raw_submit(budgets));
        };
        if (!std::strcmp(argv[1], "raw_costs")) {
            for (unsigned variant = 0; variant < 4; ++variant) {
                const bool fd_request = (variant & 1) != 0, fd_response = (variant & 2) != 0;
                slot.dlc = 8;
                slot.flags = fd_request ? 24 : 0;
                policy.buses[0].fd_response = fd_response;
                motor_pdo::write_sideband_slot(*raw, 0, slot);
                const unsigned cost = (fd_request ? 64U : 138U) + (fd_response ? 64U : 138U);
                budgets[0].reset_budget(2 * variant);
                budgets[0].charge(motor_cfg::can_budget_cap_us - cost + 1);
                raw->BatchSeq = 2 * variant + 1;
                CHECK(process_sideband(2 * variant).total == 1);
                CHECK(sent.size() == variant && budgets[0].can_afford(cost - 1));
                budgets[0].reset_budget(2 * variant);
                budgets[0].charge(motor_cfg::can_budget_cap_us - cost);
                raw->BatchSeq = 2 * variant + 2;
                CHECK(process_sideband(2 * variant).total == 0);
                CHECK(sent.size() == variant + 1 && !budgets[0].can_afford(1));
            }
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_limits")) {
            raw->ValidMask = 3;
            raw->SlotCount = 2;
            slot.id = 0x101;
            motor_pdo::write_sideband_slot(*raw, 1, slot);
            raw->BatchSeq = 1;
            auto drops = process_sideband(0);
            CHECK(sent.size() == 1 && drops.total == 1 && drops.channel[0] == 1);
            CHECK(process_sideband(0).total == 0 && sent.size() == 1);
            raw->BatchSeq = 2;
            CHECK(process_sideband(1).total == 2 && sent.size() == 1);
            raw->BatchSeq = 3;
            CHECK(process_sideband(2).total == 1 && sent.size() == 2);
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_filter")) {
            policy.buses[0].protected_count = 2;
            policy.buses[0].protected_ids[0] = 1;
            policy.buses[0].protected_ids[1] = 0x7ffU;
            slot.id = 1;
            motor_pdo::write_sideband_slot(*raw, 0, slot);
            raw->BatchSeq = 1;
            CHECK(process_sideband(0).total == 1 && sent.empty());
            slot.id = 0x7FF;
            motor_pdo::write_sideband_slot(*raw, 0, slot);
            raw->BatchSeq = 2;
            CHECK(process_sideband(0).total == 1 && sent.empty());
            slot.id = 0x100;
            motor_pdo::write_sideband_slot(*raw, 0, slot);
            const unsigned cost = dcu::app::BusScheduler::frame_cost(false, 1) +
                                  dcu::app::BusScheduler::frame_cost(false, 8);
            budgets[0].charge(motor_cfg::can_budget_cap_us - cost);
            raw->BatchSeq = 3;
            allow_send = false;
            CHECK(process_sideband(0).total == 1 && sent.empty());
            CHECK(budgets[0].can_afford(cost));
            allow_send = true;
            raw->BatchSeq = 4;
            CHECK(process_sideband(0).total == 0 && sent.size() == 1);
            CHECK(!budgets[0].can_afford(cost));
            raw->BatchSeq = 5;
            policy.permitted = false;
            CHECK(process_sideband(2).total == 0 && sent.size() == 1);
            policy.permitted = true;
            CHECK(process_sideband(2).total == 0 && sent.size() == 1);
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_full_session")) {
            uint8_t data[544]{};
            canfd_bus_pdo_t bus{};
            bus.cycle_seq = 1;
            bus.valid_mask = 1;
            bus.slot[0].can_id = 0x100;
            bus.slot[0].flags = 1;
            bus.slot[0].dlc = 1;
            std::memcpy(data + 32, &bus, sizeof(bus));
            dcu::app::RawCanBatch batch{};
            motor_pdo::decode_raw_bus(motor_pdo::raw_bus(data, 0), 0, batch);
            CHECK(gateway.process(batch, policy, 0, raw_submit(budgets)).total == 0 && sent.size() == 1);
            CHECK(gateway.process(batch, policy, 2, raw_submit(budgets)).total == 0 && sent.size() == 1);
            gateway.reset_session();
            CHECK(gateway.process(batch, policy, 2, raw_submit(budgets)).total == 0 && sent.size() == 2);
            return 0;
        }
    }
    if (!std::strcmp(argv[1], "startup_gate")) {
        select_owner(OWNER_PDO);
        motor_control_operational(true);
        command_axis(0, false);
        command_axis(0);
        command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & 8U));
        CHECK(axis_diagnostics(0)->TxEnqueueCount == 0);
        command_axis(0);
        command_axis(MC_AXIS_ARM);
        motor_control_prepare_input();
        CHECK(axis_feedback(0)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY);
        return 0;
    }
    if (!std::strcmp(argv[1], "lease_timeout") || !std::strcmp(argv[1], "feedback_timeout") ||
        !std::strcmp(argv[1], "bus_off")) {
        start_axis();
        uint16_t fault;
        if (!std::strcmp(argv[1], "lease_timeout")) {
            test_ms = 10;
            run_cycle();
            fault = 3;
        } else if (!std::strcmp(argv[1], "feedback_timeout")) {
            test_ms = 19;
            command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
            test_ms = 20;
            command_axis(MC_AXIS_ARM | MC_AXIS_START_MOTION);
            fault = 4;
        } else {
            bus_states[0] = 3;
            run_cycle();
            fault = 5;
        }
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & 8U));
        CHECK(axis_feedback(0)->BridgeFaultCode == fault);
        CHECK(cancellations != 0);
        return 0;
    }
    return 2;
}
