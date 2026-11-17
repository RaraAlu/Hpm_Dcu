#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <tuple>
#include <vector>
#include "motor_application.hpp"
#include "axis_controller.hpp"
#include "raw_can_gateway.hpp"
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
namespace parameters = dcu::app::parameters;
namespace motor_cfg = dcu::app::config::motor_control;
namespace motor_pdo = dcu::ethercat::motor_pdo;
namespace axis_codec = dcu::ethercat::motor_axis_codec;
namespace adapter = dcu::ethercat::motor_adapter;
using namespace dcu::app::motor_application;
using namespace dcu::ethercat::motor_dictionary;

static uint32_t test_ms, test_us, cancellations;
static uint16_t test_input_size;
static std::deque<std::pair<dcu_can_frame_t, uint32_t>> incoming[ENCOS_CHANNEL_COUNT];
static std::vector<std::tuple<unsigned, dcu_can_frame_t, unsigned>> sent;
static bool allow_send = true;
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
uint32_t dcu_can_completions(unsigned) { return 0; }
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
bool dcu_can_axis_pending(unsigned) { return false; }
unsigned dcu_can_tx_free(unsigned) { return 8; }
uint32_t dcu_can_tx_progress(unsigned) { return 0; }
void dcu_can_cancel(unsigned) { ++cancellations; }
void dcu_can_cancel_axis(unsigned) { ++cancellations; }
void dcu_can_cancel_service(unsigned bus)
{
    CHECK(bus < ENCOS_CHANNEL_COUNT);
    CHECK(parameters::inspect(bus).busy);
    ++service_cancellations[bus];
    ++cancellations;
}
bool dcu_can_send(unsigned bus, const dcu_can_frame_t *frame, unsigned axis)
{
    if (!allow_send) return false;
    sent.emplace_back(bus, *frame, axis);
    return true;
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

void configure_axis(unsigned a)
{
    auto *board = object<mc_board_config_t>(0x2001);
    board->AxisConfiguredMask |= 1u << a;
    auto *channel = object<mc_channel_config_t>(0x2010 + a / ENCOS_MOTORS_PER_CHANNEL);
    channel->Role = 1;
    channel->MotorConfiguredMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
    auto *route = object<mc_routing_t>(axis_object_base(a));
    route->ModelProfileId = 0x8000;
    route->PositionLimitMin = -10000000;
    route->PositionLimitMax = 10000000;
    route->VelocityLimit = 20000000;
    route->CurrentLimit = 100;
    route->TorqueLimit = 10000;
    auto *basic = object<mc_basic_t>(axis_object_base(a) + 1);
    basic->TorqueConstantRaw = 50;
    basic->CanTimeoutMs = 100;
    *object<mc_range_t>(axis_object_base(a) + 2) = {0, 500, 0, 5, -1000, 1000, -2000, 2000, -100, 100, -100, 100};
}
bool configure_controller(dcu::app::AxisController &controller, unsigned a, bool enabled = true, uint8_t format = 0)
{
    return controller.configure(enabled, format, axis_codec::configuration(*object<mc_routing_t>(axis_object_base(a)),
                                *object<mc_basic_t>(axis_object_base(a) + 1),
                                *object<mc_range_t>(axis_object_base(a) + 2)));
}
static bool same_feedback(const dcu::app::AxisFeedback &a, const dcu::app::AxisFeedback &b)
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
void query_observation(dcu::app::AxisController &axis, uint8_t code, uint32_t bits, uint32_t now = 1)
{
    CHECK(parameters::start_query(0, 0, code, 1, now, dcu::app::ParameterDialect::EncosLegacy, 1, 0));
    parameters::mark_sent(0, now);
    dcu_can_frame_t response{};
    response.id = 1;
    response.dlc = 6;
    response.data[0] = 0xA0;
    response.data[1] = code;
    for (unsigned i = 0; i < 4; ++i) response.data[2 + i] = bits >> (24 - 8 * i);
    axis.consume_feedback(response, now, now, dcu::app::ParameterDialect::EncosLegacy);
    CHECK(!parameters::inspect(0).busy);
}
void command_axis(uint16_t control, bool enabled = true)
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
    channel.ExpectedRole = 1;
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
    configure_axis(0);
    CHECK(adapter::apply_configuration());
    select_owner(OWNER_PDO);
    motor_control_operational(true);
    command_axis(0, false);
    command_axis(0);
    command_axis(MC_AXIS_ARM);
    size_t read = 0;
    for (unsigned n = 0; n < 80; ++n) {
        run_cycle();
        while (read < sent.size()) {
            const auto [bus, request, axis] = sent[read++];
            if (axis != ENCOS_AXIS_COUNT || request.dlc != 2 || request.data[1] > 3) continue;
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
            auto *channel = object<mc_channel_config_t>(0x2010 + c);
            channel->Role = 2;
            channel->RawRateLimit = 8;
            channel->FrameFormat = 0;
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
        CHECK(adapter::apply_configuration());
        motor_control_operational(true);
        submit(false);
        sent.clear();
    }
};

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "config_source")) {
        auto *config = object<mc_board_config_t>(0x2001);
        config->MaxCommandLeaseMs = 13;
        config->FeedbackTimeoutMs = 27;
        motor_control_init();
        select_wire_profile(1);
        motor_control_operational(true);
        uint8_t data[784]{};
        mc_board_rx_t board{};
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        board.CycleSeq = 1;
        board.CommandLeaseMs = 13;
        std::memcpy(data, &board, sizeof(board));
        motor_control_output(data);
        run_cycle();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1);
        board.CycleSeq = 2;
        board.CommandLeaseMs = 14;
        std::memcpy(data, &board, sizeof(board));
        motor_control_output(data);
        run_cycle();
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq == 1);
        CHECK(object<mc_board_tx_t>(0x6000)->BoardStatus & 64U);
        return 0;
    }
    motor_control_init();
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
        CHECK(!adapter::apply_configuration());
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
        configure_axis(0);
        CHECK(adapter::apply_configuration());
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
        const auto dialect = dcu::app::ParameterDialect::EncosLegacy;
        CHECK(parameters::start_query(1, 7, 1, 1, 20, dialect, 1, 0));
        parameters::check_timeout(1, 119);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 0);
        parameters::check_timeout(1, 120);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 1);
        CHECK(axis_diagnostics(7)->LastServiceError == RESULT_TIMEOUT);
        parameters::check_timeout(1, 121);
        CHECK(service_cancellations[1] == 1);

        CHECK(parameters::start_query(1, 7, 1, 1, 200, dialect, 1, 0));
        parameters::mark_sent(1, 250);
        parameters::check_timeout(1, 300);
        CHECK(parameters::inspect(1).sent);
        parameters::check_timeout(1, 349);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 1);
        parameters::check_timeout(1, 350);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 2);

        CHECK(parameters::start_query(1, 7, 1, 1, 400, dialect, 1, 0));
        parameters::mark_sent(1, UINT32_MAX - 50U);
        parameters::check_timeout(1, 48);
        CHECK(parameters::inspect(1).busy);
        parameters::check_timeout(1, 49);
        CHECK(!parameters::inspect(1).busy && service_cancellations[1] == 3);

        configure_axis(7);
        auto *transaction = object<mc_transaction_t>(axis_object_base(7) + 7);
        transaction->Operation = 2;
        transaction->ParameterId = 3;
        transaction->RequestSeq = 1;
        parameters::Request request{};
        request.axis = 7;
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
        parameters::mark_sent(1, 550);
        dcu_can_frame_t acknowledgement{};
        acknowledgement.id = 1;
        acknowledgement.dlc = 3;
        acknowledgement.data[0] = 0x80;
        acknowledgement.data[1] = 4;
        acknowledgement.data[2] = 1;
        CHECK(parameters::consume(1, 7, acknowledgement, 649).acknowledged);
        CHECK(parameters::pending(1) && !parameters::inspect(1).sent);
        parameters::check_timeout(1, 748);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 3);
        parameters::mark_sent(1, 748);
        parameters::check_timeout(1, 847);
        CHECK(parameters::inspect(1).sent);
        parameters::check_timeout(1, 848);
        CHECK(transaction->DoneSeq == 1 && transaction->Result == RESULT_TIMEOUT);
        CHECK(service_cancellations[1] == 4);
        CHECK(service_cancellations[0] == 0 && service_cancellations[2] == 0);
        return 0;
    }
    if (!std::strcmp(argv[1], "transaction_cancellation")) {
        const auto dialect = dcu::app::ParameterDialect::EncosLegacy;
        configure_axis(7);
        auto *transaction = object<mc_transaction_t>(axis_object_base(7) + 7);
        transaction->Operation = 1;
        transaction->ParameterId = 0x101;
        transaction->RequestSeq = 9;
        parameters::Request request{};
        request.axis = 7;
        request.dialect = dialect;
        request.can_id = 1;
        request.sequence = 9;
        request.parameter = 0x101;
        request.opcode = 1;
        CHECK(parameters::commit(request, 0, true, false).accepted);
        parameters::abort_internal(1);
        CHECK(parameters::inspect(1).busy && service_cancellations[1] == 0);
        parameters::mark_sent(1, 2);
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
        CHECK(!parameters::consume(1, 7, late, 3).valid);
        CHECK(transaction->DoneSeq == 9 && transaction->Result == RESULT_IO);

        axis_diagnostics(0)->LastServiceError = 123;
        for (unsigned submitted = 0; submitted < 2; ++submitted) {
            CHECK(parameters::start_query(0, 0, 1, 1, 10, dialect, 1, 0));
            if (submitted) parameters::mark_sent(0, 10);
            parameters::abort_internal(0);
            CHECK(!parameters::inspect(0).busy && service_cancellations[0] == submitted + 1);
            CHECK(axis_diagnostics(0)->LastServiceError == 123);
        }
        transaction->RequestSeq = 10;
        request.sequence = 10;
        CHECK(parameters::commit(request, 10, true, false).accepted);
        CHECK(parameters::start_query(2, 14, 1, 1, 10, dialect, 1, 0));
        parameters::reset_session(RESULT_STATE);
        CHECK(transaction->DoneSeq == 10 && transaction->Result == RESULT_STATE);
        CHECK(service_cancellations[1] == 2 && service_cancellations[2] == 1);
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            CHECK(parameters::available(c, 10) && parameters::pending(c) == nullptr);
        return 0;
    }
    if (!std::strcmp(argv[1], "transaction_observation")) {
        const auto dialect = dcu::app::ParameterDialect::EncosLegacy;
        axis_diagnostics(0)->QueriedPosition = -11;
        axis_diagnostics(0)->QueriedVelocity = -22;
        for (unsigned query = 1; query <= 2; ++query) {
            CHECK(parameters::start_query(0, 0, query, 1, query, dialect, 1, 0));
            parameters::mark_sent(0, query);
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
    if (!std::strcmp(argv[1], "service_other_axis")) {
        const unsigned axis = 7;
        auto *route = object<mc_routing_t>(axis_object_base(axis));
        route->ModelProfileId = 0x8000;
        select_owner(OWNER_SDO);
        auto *maintenance = object<mc_maintenance_t>(axis_object_base(axis) + 8);
        maintenance->Opcode = 3;
        uint32_t sequence = 1;
        CHECK(MC_OD_Write(axis_object_base(axis) + 8, 4, sizeof(sequence),
                          reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
        test_ms = 1;
        run_cycle();
        CHECK(!sent.empty() && std::get<0>(sent.back()) == 1);
        dcu_can_frame_t response{};
        response.id = 0x7FF;
        response.dlc = 4;
        response.data[1] = 1;
        response.data[2] = 1;
        response.data[3] = 3;
        deliver(1, response);
        CHECK(maintenance->DoneSeq == 1 && maintenance->Result == RESULT_OK);
        CHECK(object<mc_maintenance_t>(0x3008)->DoneSeq == 0);
        return 0;
    }
    if (!std::strncmp(argv[1], "service_", 8)) {
        auto *route = object<mc_routing_t>(0x3000);
        route->ModelProfileId = 0x8000;
        route->CanId = 1;
        select_owner(OWNER_SDO);
        auto *transaction = object<mc_transaction_t>(0x3007);
        transaction->Operation = 2;
        uint32_t sequence = 1;
        if (!std::strcmp(argv[1], "service_extended")) {
            auto *kt = object<mc_kt_t>(0x3004);
            auto *snapshot = object<mc_query_t>(0x3005);
            kt->Group = 10;
            kt->CalibrationMode = 1;
            kt->Kt1Raw = 0; kt->Kt2Raw = 4095; kt->Kt3Raw = 0x123; kt->Kt4Raw = 0xABC;
            transaction->ParameterId = 15;
            CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            test_ms = 1;
            run_cycle();
            CHECK(parameters::inspect(0).sent && sent.size() == 1);
            auto response = std::get<1>(sent.back());
            CHECK(response.data[0] == 0xCA && response.data[1] == 16 && response.dlc == 8);
            response.data[0] = 0xEE;
            response.data[1] = 0x2A;
            deliver(0, response);
            CHECK(transaction->Result == RESULT_BUSY && transaction->DoneSeq == 0);
            ++test_ms;
            run_cycle();
            CHECK(parameters::inspect(0).sent && sent.size() == 2);
            CHECK(std::get<1>(sent.back()).data[0] == 0xEA && std::get<1>(sent.back()).data[1] == 36);
            kt->Group = 1;
            kt->Kt1Raw = kt->Kt2Raw = kt->Kt3Raw = kt->Kt4Raw = 99;
            deliver(0, response);
            CHECK(transaction->Result == RESULT_OK && transaction->DoneSeq == sequence);
            CHECK(kt->Group == 10 && kt->CalibrationMode == 1);
            CHECK(kt->Kt1Raw == 0 && kt->Kt2Raw == 4095 && kt->Kt3Raw == 0x123 && kt->Kt4Raw == 0xABC);
            CHECK(snapshot->SnapshotSeq == 1 && transaction->RawResponse[1] == 0x2A);
            transaction->Operation = 1;
            transaction->ParameterId = 0x127;
            ++sequence;
            CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            ++test_ms;
            run_cycle();
            response.dlc = 7;
            response.data[0] = 0xA0; response.data[1] = 39; response.data[2] = 2;
            response.data[3] = 0xFE; response.data[4] = 0xDC;
            response.data[5] = 0x12; response.data[6] = 0x34;
            deliver(0, response);
            CHECK(snapshot->SnapshotSeq == 1 && !(snapshot->ValidBits & 0x1C0));
            response.data[2] = 1;
            deliver(0, response);
            CHECK(transaction->Result == RESULT_OK && transaction->DoneSeq == sequence);
            CHECK(snapshot->SnapshotSeq == 2 && (snapshot->ValidBits & 0x1C0) == 0x1C0);
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
            route->CanId = 2;
            route->ModelProfileId = 0;
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
            for (uint8_t code : {5, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34, 35, 37, 38}) {
                begin_query(code);
                const bool pair = (code >= 23 && code <= 28) || (code >= 32 && code <= 34);
                const bool choice = code == 35 || code == 37;
                response.dlc = pair ? 6 : choice ? 3 : 4;
                response.data[1] = code;
                response.data[2] = choice ? 1 : 0xFE;
                response.data[3] = 0xDC;
                response.data[4] = 0x12;
                response.data[5] = 0x34;
                deliver(0, response);
                CHECK(transaction->DoneSeq == sequence && transaction->Result == RESULT_OK);
                if (pair) {
                    const uint16_t index = code <= 28 ? 0x3002 : 0x3003;
                    const uint8_t sub = 1 + 2 * (code <= 28 ? code - 23 : code - 32);
                    CHECK(read_sdo<uint16_t>(index, sub) == 0xFEDC);
                    CHECK(read_sdo<uint16_t>(index, sub + 1) == 0x1234);
                } else if (code == 35) {
                    CHECK(object<mc_kt_t>(0x3004)->CalibrationMode == 1);
                } else if (code == 37) {
                    CHECK(object<mc_query_t>(0x3005)->BrakeState == 1);
                } else {
                    const uint8_t sub = code == 5 ? 1 : code == 22 ? 3 : code == 31 ? 4 : 5;
                    CHECK(read_sdo<uint16_t>(0x3001, sub) == 0xFEDC);
                }
            }
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
        if (!std::strcmp(argv[1], "service_maintenance")) {
            auto *maintenance = object<mc_maintenance_t>(0x3008);
            maintenance->Opcode = 3;
            CHECK(MC_OD_Write(0x3008, 4, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            test_ms = 1;
            run_cycle();
            dcu_can_frame_t response{};
            response.id = 0x7FF;
            response.dlc = 4;
            response.data[1] = 1;
            response.data[2] = 1;
            response.data[3] = 3;
            deliver(0, response);
            CHECK(maintenance->DoneSeq == sequence && maintenance->Result == RESULT_OK);
            CHECK(!parameters::inspect(0).busy && !parameters::available(0, 500));
            CHECK(parameters::available(0, 501));
            transaction->ParameterId = 3;
            CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) ==
                  ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED);
            return 0;
        }
        if (!std::strcmp(argv[1], "service_rejected")) {
            const uint16_t parameters[] = {1, 16, 0x101};
            for (uint16_t parameter : parameters) {
                transaction->ParameterId = parameter;
                auto adjacent = parameters::inspect(1);
                CHECK(MC_OD_Write(0x3007, 3, sizeof(sequence),
                                  reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
                CHECK(transaction->Result == RESULT_UNSUPPORTED && transaction->DoneSeq == sequence);
                CHECK(!parameters::inspect(0).busy && parameters::pending(0) == nullptr);
                auto after = parameters::inspect(1);
                CHECK(adjacent.busy == after.busy && adjacent.can_id == after.can_id &&
                      adjacent.axis == after.axis && adjacent.sent == after.sent &&
                      adjacent.internal == after.internal);
                ++sequence;
            }
            return 0;
        }
        transaction->ParameterId = 3;
        object<mc_basic_t>(0x3001)->TorqueConstantRaw = 50;
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
        dcu_can_frame_t response{};
        response.id = 1;
        response.dlc = 3;
        response.data[0] = 0x80;
        response.data[1] = 4;
        response.data[2] = 1;
        deliver(0, response);
        CHECK(axis_diagnostics(0)->ProtocolErrorCount == 0);
        CHECK(parameters::pending(0) && parameters::pending(0)->dlc == 2 && !parameters::inspect(0).sent);
        test_ms = 2;
        run_cycle();
        response.dlc = 4;
        response.data[0] = 0xA0;
        response.data[1] = 22;
        bool mismatch = !std::strcmp(argv[1], "service_mismatch");
        response.data[2] = 0;
        response.data[3] = mismatch ? 51 : 50;
        deliver(0, response);
        CHECK(!parameters::inspect(0).busy && transaction->DoneSeq == sequence);
        CHECK(transaction->Result == (mismatch ? RESULT_MISMATCH : RESULT_OK));
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_queue")) {
        object<mc_channel_config_t>(0x2010)->Role = 2;
        CHECK(adapter::apply_configuration());
        dcu_can_frame_t frame{};
        for (unsigned i = 0; i < motor_cfg::raw_queue_capacity + 3; ++i) {
            frame.id = i;
            incoming[0].emplace_back(frame, 0);
        }
        run_cycle();
        run_cycle();
        Publication snapshot{};
        prepare_publication(snapshot);
        CHECK(snapshot.channels[0].raw_drops == 4);
        for (unsigned i = 0; i < motor_cfg::raw_queue_capacity - 1; ++i) {
            CHECK(take_raw_frame(0, frame) && frame.id == i);
        }
        CHECK(!take_raw_frame(0, frame));
        return 0;
    }
    if (!std::strcmp(argv[1], "same_ms")) {
        motor_control_prepare_input();
        const uint32_t sequence = object<mc_board_tx_t>(0x6000)->PublishSeq;
        motor_control_input(first);
        motor_control_input(second);
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
        motor_control_input(first);
        axis_feedback(0)->ActualPosition = 12345;
        motor_control_input(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        motor_control_prepare_input();
        CHECK(!(axis_feedback(0)->StatusWord & 16U));
        motor_control_input(second);
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
            std::memset(first, 0xA5, sizeof(first));
            motor_control_prepare_input();
            motor_control_input(first);
            mc_board_tx_t board;
            std::memcpy(&board, first, sizeof(board));
            CHECK(board.PublishSeq == object<mc_board_tx_t>(0x6000)->PublishSeq);
            for (unsigned i = test_input_size; i < sizeof(first); ++i)
                CHECK(first[i] == 0xA5);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "sdo_publication")) {
        start_axis();
        motor_control_input(first);
        const uint32_t sequence = object<mc_board_tx_t>(0x6000)->PublishSeq;
        deliver(0, type1_feedback());
        const auto count = read_sdo<uint32_t>(0x2400, 4);
        CHECK(count >= 2);
        CHECK(read_sdo<uint16_t>(0x6200, 1) & 16U);
        CHECK(read_sdo<uint16_t>(0x6200, 1) & 16U);
        CHECK(read_sdo<uint32_t>(0x2400, 4) == count);
        CHECK(object<mc_board_tx_t>(0x6000)->PublishSeq == sequence);
        motor_control_input(second);
        CHECK(!std::memcmp(first, second, test_input_size));
        motor_control_prepare_input();
        motor_control_input(first);
        mc_axis_tx_t snapshot{};
        std::memcpy(&snapshot, first + 64, sizeof(snapshot));
        CHECK(snapshot.StatusWord & 16U);
        CHECK(!(read_sdo<uint16_t>(0x6200, 1) & 16U));
        motor_control_input(second);
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
        configure_axis(0);
        CHECK(adapter::apply_configuration());
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
                const auto [bus, request, axis] = sent[read++];
                if (axis != ENCOS_AXIS_COUNT || request.dlc != 2 || request.data[1] > 3) continue;
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
        configure_axis(0);
        configure_axis(7);
        dcu::app::AxisController first_axis, second_axis;
        first_axis.initialize(0);
        second_axis.initialize(7);
        CHECK(configure_controller(first_axis, 0) && configure_controller(second_axis, 7));
        dcu::motor::Command intent{};
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        intent.control_bits = MC_AXIS_ARM;
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        first_axis.stop(3);
        dcu::app::AxisFeedback first_feedback{}, second_feedback{};
        dcu::app::AxisDiagnostics first_diagnostics{}, second_diagnostics{};
        first_axis.read_feedback(0, 20, first_feedback, first_diagnostics);
        second_axis.read_feedback(0, 20, second_feedback, second_diagnostics);
        CHECK(!(first_feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY));
        CHECK(second_feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(first_feedback.bridge_fault == 3 && !second_feedback.bridge_fault);
        CHECK(configure_controller(first_axis, 0, false));
        CHECK(!first_axis.configured() && second_axis.configured());
        CHECK(!configure_controller(first_axis, 0, true, 2));
        CHECK(!first_axis.configured());
        parameters::Request invalid{};
        invalid.axis = ENCOS_AXIS_COUNT;
        CHECK(!commit_parameter(invalid, 0, true).accepted);
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_phases")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Velocity;
        command.current_limit_dA = 50;
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
        dcu_can_frame_t frame{};
        const auto status = [&]() {
            axis.read_feedback(1, 20, feedback, diagnostics);
            return feedback.status_word;
        };
        CHECK(!axis.active() && !axis.control_sent() && !axis.encode_control(frame));
        axis.control_submitted(1);
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
        axis.consume_feedback(type1_feedback(), 1, 1, dcu::app::ParameterDialect::None);
        CHECK(!(status() & 8U)); // No first submission yet: unsolicited Type-1 cannot start motion.
        axis.check_timeouts(21, false, 20);
        CHECK(axis.active() && !axis.control_sent());
        axis.control_submitted(21);
        CHECK(axis.control_sent() && feedback.mode == dcu::motor::ControlMode::Position);
        axis.check_timeouts(40, false, 20);
        CHECK(axis.active());
        axis.consume_feedback(type1_feedback(), 40, 40, dcu::app::ParameterDialect::None);
        axis.control_submitted(40);
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
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
        for (unsigned variant = 0; variant < 2; ++variant) {
            CHECK(configure_controller(axis, 0));
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
                axis.control_submitted(UINT32_MAX - 5);
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
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
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
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
        axis.read_feedback(0, 20, feedback, diagnostics);
        CHECK(feedback.status_word & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(!axis.active() && !axis.control_sent());
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_config_snapshot")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
        CHECK(parameters::start_query(0, 0, 3, 1, 1, dcu::app::ParameterDialect::EncosLegacy, 1, 0));
        parameters::mark_sent(0, 1);
        dcu_can_frame_t response{};
        response.id = 1;
        response.dlc = 6;
        response.data[0] = 0xA0;
        response.data[1] = 3;
        response.data[2] = 0x3F;
        response.data[3] = 0x80;
        axis.consume_feedback(response, 1, 1, dcu::app::ParameterDialect::EncosLegacy);
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
        axis.read_feedback(1, 20, feedback, diagnostics);
        const auto torque = feedback.sample.torque_mNm;
        CHECK(torque > 0 && feedback.sample.current_cA == 100);
        object<mc_basic_t>(0x3001)->TorqueConstantRaw = 100;
        *object<mc_range_t>(0x3002) = {};
        object<mc_routing_t>(0x3000)->CanId = 2;
        axis.read_feedback(1, 20, feedback, diagnostics);
        CHECK(feedback.sample.torque_mNm == torque);
        CHECK(axis.matches(1) && !axis.matches(2));
        axis.read_feedback(2 + motor_cfg::query_current_ms, 20, feedback, diagnostics);
        CHECK(feedback.sample.torque_mNm == torque && !(feedback.sample.valid_fields & 8U));
        CHECK(!configure_controller(axis, 0));
        CHECK(!axis.configured());
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_runtime_authority")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Position;
        command.current_limit_dA = 50;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        query_observation(axis, 1, 0x3F800000);
        query_observation(axis, 2, 0);
        query_observation(axis, 3, 0);
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
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
        configure_axis(0);
        object<mc_routing_t>(0x3000)->PowerQueryPeriodMs = 100;
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
        dcu::motor::Command command{};
        command.mode = dcu::motor::ControlMode::Position;
        command.current_limit_dA = 50;
        CHECK(axis.accept(command, true, 1));
        command.control_bits = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 1));
        for (uint8_t code = 1; code <= 4; ++code) query_observation(axis, code, 0);
        command.control_bits |= MC_AXIS_START_MOTION;
        CHECK(axis.accept(command, true, 1));
        axis.control_submitted(1);
        axis.consume_feedback(type1_feedback(), 1, 1, dcu::app::ParameterDialect::None);
        dcu::app::AxisFeedback feedback{}, repeated{};
        dcu::app::AxisDiagnostics diagnostics{};
        diagnostics.tx_drop_count = 88;
        axis.record_tx_drop();
        const dcu::app::AxisController &view = axis;
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
        axis.consume_feedback(type1_feedback(), 22, 22, dcu::app::ParameterDialect::None);
        view.read_feedback(22, 20, repeated, diagnostics);
        CHECK((repeated.status_word & (16U | 1024U)) == 16U);
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_history")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(configure_controller(axis, 0));
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
        axis.control_submitted(1);
        dcu_can_frame_t invalid{};
        invalid.id = 1;
        invalid.dlc = 8;
        axis.consume_feedback(invalid, 1, 1, dcu::app::ParameterDialect::None);
        axis.stop(4);
        dcu::app::AxisFeedback feedback{};
        dcu::app::AxisDiagnostics diagnostics{};
        axis.read_feedback(1, 20, feedback, diagnostics);
        CHECK(diagnostics.tx_enqueue_count == 1 && diagnostics.rx_valid_count == 3);
        CHECK(diagnostics.protocol_error_count == 1 && diagnostics.active_fault_bits == (1U << 4));
        CHECK(configure_controller(axis, 0));
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
    if (!std::strcmp(argv[1], "raw_applied_policy")) {
        RawPdoFixture input(1);
        configure_axis(0);
        auto *channel = object<mc_channel_config_t>(0x2010);
        channel->Role = 3;
        input.start();
        object<mc_routing_t>(0x3000)->CanId = 2;
        channel->Role = channel->RawRateLimit = 0;
        input.sideband.BatchSeq = 1;
        input.sideband.CanId0 = 1;
        input.submit();
        CHECK(sent.empty() && object<mc_board_tx_t>(0x6000)->TxDropCount == 1);
        input.sideband.BatchSeq = 2;
        input.sideband.CanId0 = 2;
        input.submit();
        CHECK(sent.size() == 1 && std::get<1>(sent.back()).id == 2);
        input.sideband.BatchSeq = 3;
        input.sideband.CanId0 = 1;
        input.sideband.Channel0 = 1;
        input.submit();
        CHECK(sent.size() == 2 && std::get<0>(sent.back()) == 1);
        input.sideband.BatchSeq = 4;
        input.submit(true, 1);
        CHECK(sent.size() == 2 && object<mc_channel_tx_t>(0x6101)->TxDropCount == 1);
        run_cycle();
        CHECK(sent.size() == 2);

        motor_control_operational(false);
        channel->Role = 3;
        channel->RawRateLimit = 8;
        CHECK(adapter::apply_configuration());
        motor_control_operational(true);
        input.submit(false);
        input.sideband.Channel0 = 0;
        input.sideband.BatchSeq = 5;
        input.submit();
        CHECK(sent.size() == 3 && std::get<1>(sent.back()).id == 1);
        input.sideband.BatchSeq = 6;
        input.sideband.CanId0 = 2;
        input.submit();
        CHECK(sent.size() == 3);
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
    if (!std::strcmp(argv[1], "raw_service_hold")) {
        RawPdoFixture input(1);
        input.start();
        object<mc_routing_t>(0x3000)->ModelProfileId = 0x8000;
        auto *maintenance = object<mc_maintenance_t>(0x3008);
        maintenance->Opcode = 3;
        maintenance->RequestSeq = 1;
        parameters::Request request{};
        request.dialect = dcu::app::ParameterDialect::EncosLegacy;
        request.can_id = 1;
        request.sequence = 1;
        request.maintenance = true;
        request.opcode = 3;
        CHECK(parameters::commit(request, 0, true, false).accepted);
        parameters::mark_sent(0, 1);
        parameters::cancel(0, RESULT_IO);
        CHECK(!parameters::inspect(0).busy && !parameters::available(0, 500));
        test_ms = 500;
        input.sideband.BatchSeq = 1;
        input.submit();
        CHECK(sent.empty() && object<mc_board_tx_t>(0x6000)->TxDropCount == 1);
        test_ms = 501;
        CHECK(parameters::available(0, 501));
        input.sideband.BatchSeq = 2;
        input.submit();
        CHECK(sent.size() == 1);
        maintenance->RequestSeq = 2;
        request.sequence = 2;
        CHECK(parameters::commit(request, 501, true, false).accepted);
        parameters::mark_sent(0, 501);
        motor_control_operational(false);
        CHECK(parameters::available(0, 502));
        CHECK(maintenance->Result == RESULT_STATE && maintenance->DoneSeq == 2);
        motor_control_operational(true);
        test_ms = 502;
        input.submit(false);
        input.sideband.BatchSeq = 3;
        input.submit();
        CHECK(sent.size() == 2);
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_objects")) {
        dcu::app::RawCanGateway first_gateway, second_gateway;
        dcu_can_frame_t frame{};
        frame.id = 101;
        first_gateway.receive(0, frame);
        CHECK(!second_gateway.take(0, frame));
        CHECK(first_gateway.take(0, frame) && frame.id == 101);
        for (unsigned n = 0; n < motor_cfg::raw_queue_capacity + 1; ++n)
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
            return gateway.process(batch, policy, now, budgets);
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
            policy.buses[0].protected_count = 1;
            policy.buses[0].protected_ids[0] = 1;
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
            CHECK(gateway.process(batch, policy, 0, budgets).total == 0 && sent.size() == 1);
            CHECK(gateway.process(batch, policy, 2, budgets).total == 0 && sent.size() == 1);
            gateway.reset_session();
            CHECK(gateway.process(batch, policy, 2, budgets).total == 0 && sent.size() == 2);
            return 0;
        }
    }
    if (!std::strcmp(argv[1], "startup_gate")) {
        configure_axis(0);
        CHECK(adapter::apply_configuration());
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
