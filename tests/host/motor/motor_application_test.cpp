#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <tuple>
#include <vector>
#include "gateway_application.hpp"
#include "motor_instance.hpp"
#include "raw_can_gateway.hpp"
#include "detail/raw_can_submit.hpp"
#include "cfg/gateway_schedule_policy.hpp"
#include "cfg/encos_runtime.hpp"
#include "encos_parameter_transactions.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_axis_codec.hpp"
#include "motor_adapter.hpp"
#include "motor_ethercat.h"
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
static std::deque<std::pair<dcu_can_frame_t, uint32_t>> incoming[ENCOS_CHANNEL_COUNT];
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
uint32_t dcu_can_rx_drops(unsigned) { return 0; }
uint16_t dcu_can_bus_state(uint32_t bus) { return bus_states[bus]; }
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, uint32_t *stamp)
{
    if (incoming[bus].empty()) return false;
    *frame = incoming[bus].front().first;
    *stamp = incoming[bus].front().second;
    incoming[bus].pop_front();
    return true;
}
bool dcu_can_pending(unsigned) { return false; }
unsigned dcu_can_tx_free(unsigned) { return 8; }
uint32_t dcu_can_tx_progress(unsigned) { return 0; }
void dcu_can_cancel(unsigned) { ++cancellations; }
static dcu_can_tx_id_t test_tx_serial;
static std::deque<dcu_can_tx_event_t> tx_events[ENCOS_CHANNEL_COUNT];
bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id)
{
    CHECK(bus < ENCOS_CHANNEL_COUNT && id);
    ++service_cancellations[bus];
    ++cancellations;
    for (auto &event : tx_events[bus])
        if (event.id == id) event.status = DCU_CAN_TX_CANCELLED;
    return true;
}
dcu_can_tx_id_t dcu_can_submit(unsigned bus, const dcu_can_frame_t *frame)
{
    if (!allow_send) return 0;
    sent.emplace_back(bus, *frame);
    tx_events[bus].push_back({++test_tx_serial, DCU_CAN_TX_COMPLETED});
    return test_tx_serial;
}
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    if (!allow_tx_events || tx_events[bus].empty()) return false;
    *event = tx_events[bus].front();
    tx_events[bus].pop_front();
    return true;
}

static void simulate_service_transmitted(unsigned bus, uint32_t now)
{
    const auto id = ++test_tx_serial;
    parameters::mark_submitted(bus, now, id);
    parameters::finish_tx(bus, {id, DCU_CAN_TX_COMPLETED});
}

static bool use_generated_profile, use_mixed_profile;
static const dcu_can_channel_config_t test_channels[] = {
    {0, 1000000, 0, false}, {1, 1000000, 0, false}, {2, 1000000, 0, false}, {3, 1000000, 0, false},
};
static const dcu_motor_device_profile_t test_device{
    "test_encos", 0, 1, 0x8000, 0, -10000000, 10000000, 20000000, 10000, 100,
    {{{0, 500}, {0, 5}, {-10, 10}, {-20, 20}, {-10, 10}, {-10, 10}, 0.5}, 100, 0}, 0, 0,
};
static dcu_production_profile_t test_profile{test_channels, ENCOS_CHANNEL_COUNT, &test_device, 1, 10, 20, {}};
static dcu_motor_device_profile_t two_bus_devices[2];

extern "C" const dcu_production_profile_t *host_fixture_profile(void);
extern "C" const dcu_production_profile_t *host_mixed_profile(void);
extern "C" const dcu_production_profile_t *dcu_production_profile(void)
{
    if (use_mixed_profile) return host_mixed_profile();
    return use_generated_profile ? host_fixture_profile() : &test_profile;
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
void deliver(unsigned bus, const dcu_can_frame_t &frame)
{
    incoming[bus].emplace_back(frame, test_ms);
    run_cycle();
}

static void select_wire_profile(unsigned profile)
{
    uint16_t indices[39];
    gRxAssign.u16SubIndex0 = motor_pdo::mapping_indices(profile, false, indices);
    std::memcpy(gRxAssign.aEntries, indices, gRxAssign.u16SubIndex0 * 2U);
    gTxAssign.u16SubIndex0 = motor_pdo::mapping_indices(profile, true, indices);
    std::memcpy(gTxAssign.aEntries, indices, gTxAssign.u16SubIndex0 * 2U);
    uint16_t in = 0, out = 0;
    CHECK(motor_control_mapping(&in, &out) == 0);
    test_input_size = in;
}

static void copy_txpdo(void *data)
{
    const auto profile = static_cast<unsigned>(mode());
    std::memcpy(data, ssc_txpdo, dcu::ethercat::pdo_layout::profiles[profile].input);
}

static void check_txpdo_bytes()
{
    const auto profile = static_cast<unsigned>(mode());
    const size_t size = dcu::ethercat::pdo_layout::profiles[profile].input;
    const auto *wire = ssc_txpdo;
    size_t offset = 0;
    const auto check = [&](const auto *mirror) {
        CHECK(offset + sizeof(*mirror) <= size);
        CHECK(!std::memcmp(wire + offset, mirror, sizeof(*mirror)));
        offset += sizeof(*mirror);
    };
    check(object<mc_board_tx_t>(0x6000));
    if (profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            check(object<canfd_bus_pdo_t>(0x6500 + 16 * c));
    } else {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            check(object<mc_channel_tx_t>(0x6100 + c));
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
            check(axis_feedback(a));
        check(object<mc_imu_t>(0x6300));
        if (profile == 1) check(object<mc_raw_t>(0x6400));
    }
    CHECK(offset == size);
    if (std::getenv("DCU_DUMP_TXPDO"))
        CHECK(std::fwrite(wire, 1, size, stdout) == size);
}

static dcu::motor::MotorConfiguration axis_configuration()
{
    return {{1, -10000000, 10000000, 20000000, 10000, 100}, test_device.encos, 0x8000, 0};
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
    mc_raw_t sideband{};
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
                std::memcpy(data + 32 + 8 * c, &channel, sizeof(channel));
            }
            std::memcpy(data + 736, &sideband, sizeof(sideband));
        } else {
            std::memcpy(data + 32, full, sizeof(full));
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

static uint32_t startup_epoch;
static uint32_t startup_sessions;

static uint64_t issue_startup_session()
{
    return (uint64_t{startup_epoch} << 32) | ++startup_sessions;
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
    std::memcpy(wire + 32 + 8 * 2, &channel, sizeof(channel));
    mc_axis_rx_t axis{};
    axis.ControlWord = control;
    axis.Mode = 1;
    axis.CurrentLimit = 50;
    std::memcpy(wire + 64 + 24 * 14, &axis, sizeof(axis));
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

static void test_production_startup(bool journal_ok)
{
    use_mixed_profile = true;
    CHECK(motor_control_init());
    select_wire_profile(1);
    auto *board = object<mc_board_tx_t>(0x6000);
    startup_epoch = 7;
    CHECK(motor_control_boot_startup(journal_ok ? uint64_t{startup_epoch} << 32 : 0,
                                     issue_startup_session) == journal_ok);
    if (!journal_ok) {
        CHECK(startup_state() == DCU_GATEWAY_FAULT);
        motor_control_operational(true);
        startup_business(1, 0, false);
        startup_business(2, MC_AXIS_ARM | MC_AXIS_START_MOTION);
        CHECK(!board->AcceptedCycleSeq);
        CHECK(sent.empty() && !startup_sessions);
        return;
    }
    uint16_t requested_owner = OWNER_PDO;
    CHECK(MC_OD_Write(0x2003, 2, 1, &requested_owner, 0) == ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
    CHECK(MC_OD_Write(0x2002, 3, 2, &requested_owner, 0) == ABORTIDX_READ_ONLY_ENTRY);
    CHECK(read_sdo<uint32_t>(0x2002, 1) == startup_epoch);
    CHECK(read_sdo<uint16_t>(axis_object_base(14), 1) == 3);
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
    CHECK(board->AcceptedCycleSeq == 1 && !startup_sessions);

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

    const auto before_drop = cancellations;
    motor_control_operational(false);
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
    CHECK(read_sdo<uint32_t>(0x2002, 2) > ready && read_sdo<uint32_t>(0x2002, 4) == 0);
    startup_business(5, 0, false);
    CHECK(board->AcceptedCycleSeq == 5);
    startup_business(4, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(board->AcceptedCycleSeq == 5);
    startup_business(6, 0);
    CHECK(board->AcceptedCycleSeq == 6);
    test_ms += 10;
    startup_business(7, MC_AXIS_ARM | MC_AXIS_START_MOTION);
    CHECK(!board->AcceptedCycleSeq && !startup_sessions);
    serve_startup_reads(read, true);
}

static void test_dynamic_policy(const char *encoded)
{
    using namespace dcu::app::profile_update;
    uint16_t wire[candidate_bytes / sizeof(uint16_t)]{};
    auto *file = std::fopen(encoded, "rb");
    CHECK(file && std::fread(wire, 1, sizeof(wire), file) == sizeof(wire) && std::fgetc(file) == EOF);
    std::fclose(file);
    test_profile.devices = nullptr;
    test_profile.device_count = 0;
    test_profile.readiness = {1, 1, {}, 1000, 5};
    CHECK(motor_control_init());
    select_wire_profile(1);
    startup_epoch = 7;
    CHECK(motor_control_boot_startup(uint64_t{startup_epoch} << 32, issue_startup_session));
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
    wire[17] ^= 1;
    CHECK(offer() == ABORTIDX_VALUE_EXCEEDED && result() == Status::Invalid);
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
    CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1 && !startup_sessions && sent.empty());
    motor_control_operational(false);
    active(2, 8, 15, 750);
}

int main(int argc, char **argv)
{
    if (argc == 3 && !std::strcmp(argv[1], "dynamic_policy")) {
        test_dynamic_policy(argv[2]);
        return 0;
    }
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "production_startup") || !std::strcmp(argv[1], "production_no_journal")) {
        test_production_startup(!std::strcmp(argv[1], "production_startup"));
        return 0;
    }
    if (!std::strcmp(argv[1], "mixed_motor_profile")) {
        use_mixed_profile = true;
        CHECK(motor_control_init());
        CHECK(adapter::configured_device(14)->model_profile_id == 0x8000);
        CHECK(adapter::configured_device(15)->model_profile_id == 0x8101);
        CHECK(adapter::configured_device(0)->model_profile_id == 0x8102);
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
        CHECK(!read_parameter(0, {2, 1, 0x101}, test_ms, true).accepted);
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
    if (!std::strcmp(argv[1], "can_profile_config")) {
        use_generated_profile = true;
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
            motor_pdo::store(wire, motor_pdo::layout::output_channels_offset + 2 * sizeof(channel), channel);
            motor_pdo::store(wire, motor_pdo::layout::output_axes_offset + slot * sizeof(command), command);
            ++board.CycleSeq;
            motor_pdo::store(wire, 0, board);
            motor_control_output(wire);
            run_cycle();
        };
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            const mc_channel_rx_t channel{1, 0, 0, 2000};
            motor_pdo::store(wire, motor_pdo::layout::output_channels_offset + c * sizeof(channel), channel);
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
        two_bus_devices[0].bus = 1;
        two_bus_devices[1] = test_device; // Same CAN ID, but on bus 0 and second in the Profile.
        test_profile.devices = two_bus_devices;
        test_profile.device_count = 2;
        CHECK(motor_control_init());
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
    const bool raw_fixture = !std::strcmp(argv[1], "raw_queue") || !std::strcmp(argv[1], "raw_full_admission") ||
                             !std::strncmp(argv[1], "raw_large_", 10);
    if (raw_fixture) {
        // Exercise generic Raw CAN mechanics without giving the production Profile a Raw writer.
        Configuration config{};
        config.board = {10, 20};
        for (auto &channel : config.channels) channel = {2, 0, 8, true};
        parameters::initialize({});
        CHECK(initialize(config));
        CHECK(motor_pdo::bind_publication());
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
    select_wire_profile(1);
    uint8_t first[1072], second[1072];
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
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
            auto axis = motor_pdo::axis(output, a);
            CHECK(!std::memcmp(&axis, output + 64 + 24 * a, sizeof(axis)));
        }
        auto raw = motor_pdo::raw(output);
        CHECK(!std::memcmp(&raw, output + 736, sizeof(raw)));
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
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
            motor_pdo::write_axis(first, a, axis);
            std::memcpy(second + 64 + 32 * a, &axis, sizeof(axis));
        }
        mc_imu_t imu{};
        std::memset(&imu, 0x44, sizeof(imu));
        motor_pdo::write_imu(first, imu);
        std::memcpy(second + 960, &imu, sizeof(imu));
        motor_pdo::write_raw(first, raw);
        std::memcpy(second + 1024, &raw, sizeof(raw));
        CHECK(!std::memcmp(first, second, sizeof(first)));
        for (unsigned i = 0; i < 2; ++i) {
            auto slot = motor_pdo::raw_slot(raw, i);
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
        object<mc_channel_config_t>(0x2010)->Role = 2;
        dcu_can_frame_t frame{};
        for (unsigned i = 0; i < raw_cfg::queue_capacity + 3; ++i) {
            frame.id = i;
            incoming[0].emplace_back(frame, 0);
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
        for (unsigned profile = 0; profile < 3; ++profile) {
            select_wire_profile(profile);
            std::memset(ssc_txpdo + test_input_size, 0xA5, sizeof(ssc_txpdo) - test_input_size);
            motor_control_prepare_input();
            check_txpdo_bytes();
            mc_board_tx_t board;
            std::memcpy(&board, ssc_txpdo, sizeof(board));
            CHECK(board.PublishSeq == object<mc_board_tx_t>(0x6000)->PublishSeq);
            for (unsigned i = test_input_size; i < sizeof(ssc_txpdo); ++i)
                CHECK(ssc_txpdo[i] == 0xA5);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "latched_layouts")) {
        const auto axis_offset = dcu::ethercat::pdo_layout::input_axes_offset;
        const auto raw_offset = dcu::ethercat::pdo_layout::motor_input_bytes;
        motor_control_prepare_input();
        check_txpdo_bytes();
        std::memcpy(first, ssc_txpdo + axis_offset, sizeof(mc_axis_tx_t));
        CHECK(!std::memcmp(ssc_txpdo + raw_offset, "T1R2", 4));
        const auto first_page = ssc_txpdo[raw_offset + 4];
        motor_control_prepare_input();
        CHECK(ssc_txpdo[raw_offset + 4] == (first_page ^ 1U));
        select_wire_profile(2);
        motor_control_prepare_input();
        check_txpdo_bytes();
        CHECK(std::memcmp(first, ssc_txpdo + axis_offset, sizeof(mc_axis_tx_t)));
        select_wire_profile(1);
        // The motor template is already restored when the SSC accepts the mapping.
        CHECK(!std::memcmp(first, ssc_txpdo + axis_offset, sizeof(mc_axis_tx_t)));
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
        mc_raw_t raw{};
        dcu::app::RawCanBatch batch{};
        raw.BatchSeq = UINT32_MAX;
        raw.SlotCount = 2;
        for (unsigned i = 0; i < 2; ++i) {
            motor_pdo::RawSlot slot{};
            slot.id = 0x100 + i;
            slot.channel = i + 2;
            slot.flags = 2 + i;
            slot.dlc = 9 + i;
            for (unsigned j = 0; j < 8; ++j) slot.data[j] = i * 19 + j;
            motor_pdo::write_raw_slot(raw, i, slot);
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
                const auto wire = motor_pdo::raw_slot(raw, i);
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
            auto slot = motor_pdo::raw_slot(input.sideband, 0);
            slot.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
            slot.dlc = 15;
            motor_pdo::write_raw_slot(input.sideband, 0, slot);
            input.sideband.BatchSeq = 1;
        }
        dcu_can_frame_t rx{};
        rx.id = 0x201;
        rx.dlc = 15;
        rx.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        std::memset(rx.data, 0xa5, sizeof(rx.data));
        incoming[0].emplace_back(rx, 0);
        rx.id = 0x202;
        rx.dlc = 8;
        incoming[0].emplace_back(rx, 0);
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
            const auto *feedback = object<mc_raw_t>(0x6400);
            CHECK(feedback->ValidMask == 1 && feedback->DropCount == 1);
            const auto slot = motor_pdo::raw_slot(*feedback, 0);
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
        auto *raw = object<mc_raw_t>(0x7300);
        *raw = {};
        raw->ValidMask = 1;
        raw->SlotCount = 1;
        motor_pdo::RawSlot slot{};
        slot.id = 0x100;
        slot.dlc = 1;
        slot.data[0] = 42;
        motor_pdo::write_raw_slot(*raw, 0, slot);
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
                motor_pdo::write_raw_slot(*raw, 0, slot);
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
            motor_pdo::write_raw_slot(*raw, 1, slot);
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
            motor_pdo::write_raw_slot(*raw, 0, slot);
            raw->BatchSeq = 1;
            CHECK(process_sideband(0).total == 1 && sent.empty());
            slot.id = 0x7FF;
            motor_pdo::write_raw_slot(*raw, 0, slot);
            raw->BatchSeq = 2;
            CHECK(process_sideband(0).total == 1 && sent.empty());
            slot.id = 0x100;
            motor_pdo::write_raw_slot(*raw, 0, slot);
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
