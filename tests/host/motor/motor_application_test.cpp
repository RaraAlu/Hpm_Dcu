#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <tuple>
#include <vector>
#include "motor_application.h"
#include "motor_application.hpp"
#include "axis_controller.hpp"
#include "raw_can_gateway.hpp"
#include "encos_parameter_transactions.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_ethercat.h"
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
using namespace dcu::app::motor_application;
using namespace dcu::ethercat::motor_dictionary;

static uint32_t test_ms, test_us, cancellations;
static uint16_t test_input_size;
static std::deque<std::pair<dcu_can_frame_t, uint32_t>> incoming[ENCOS_CHANNEL_COUNT];
static std::vector<std::tuple<unsigned, dcu_can_frame_t, unsigned>> sent;
static bool allow_send = true;
static uint16_t bus_states[ENCOS_CHANNEL_COUNT];
static unsigned service_cancellations[ENCOS_CHANNEL_COUNT];

uint32_t ethercat_canfd_now_ms(void) { return test_ms; }
uint32_t ethercat_canfd_now_us(void) { return test_us; }
uint32_t ethercat_canfd_rx_drops(unsigned) { return 0; }
uint32_t ethercat_canfd_completions(unsigned) { return 0; }
uint16_t ethercat_canfd_bus_state(uint32_t bus) { return bus_states[bus]; }
bool ethercat_canfd_receive(unsigned bus, dcu_can_frame_t *frame, uint32_t *stamp)
{
    if (incoming[bus].empty()) return false;
    *frame = incoming[bus].front().first;
    *stamp = incoming[bus].front().second;
    incoming[bus].pop_front();
    return true;
}
bool ethercat_canfd_pending(unsigned) { return false; }
bool ethercat_canfd_axis_pending(unsigned) { return false; }
unsigned ethercat_canfd_tx_free(unsigned) { return 8; }
uint32_t ethercat_canfd_tx_progress(unsigned) { return 0; }
void ethercat_canfd_cancel(unsigned) { ++cancellations; }
void ethercat_canfd_cancel_axis(unsigned) { ++cancellations; }
void ethercat_canfd_cancel_service(unsigned bus)
{
    CHECK(bus < ENCOS_CHANNEL_COUNT);
    CHECK(parameters::inspect(bus).busy);
    ++service_cancellations[bus];
    ++cancellations;
}
bool ethercat_canfd_send(unsigned bus, const dcu_can_frame_t *frame, unsigned axis)
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
    CHECK(apply_configuration());
    object<mc_owner_t>(0x2003)->Owner = OWNER_PDO;
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
    for (unsigned n = 0; n < 8 && !axis_diagnostics(0)->TxEnqueueCount; ++n) run_cycle();
    CHECK(axis_diagnostics(0)->TxEnqueueCount != 0);
    deliver(0, type1_feedback());
    motor_control_prepare_input();
    CHECK(axis_feedback(0)->StatusWord & 8U);
}

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
        CHECK(parameters::commit(7, false, 500, true, false));
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
        CHECK(parameters::commit(7, false, 0, true, false));
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
        CHECK(parameters::commit(7, false, 10, true, false));
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
        object<mc_owner_t>(0x2003)->Owner = OWNER_SDO;
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
        object<mc_owner_t>(0x2003)->Owner = OWNER_SDO;
        auto *transaction = object<mc_transaction_t>(0x3007);
        transaction->Operation = 2;
        uint32_t sequence = 1;
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
        CHECK(apply_configuration());
        dcu_can_frame_t frame{};
        for (unsigned i = 0; i < motor_cfg::raw_queue_capacity + 3; ++i) {
            frame.id = i;
            incoming[0].emplace_back(frame, 0);
        }
        run_cycle();
        run_cycle();
        CHECK(prepare_publication().raw_drops[0] == 4);
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
        CHECK(axis.ActualPosition == 12345 && axis.FeedbackAgeMs == 1);
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
        CHECK(first_axis.configure(true, 0) && second_axis.configure(true, 0));
        mc_axis_rx_t intent{};
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        intent.ControlWord = MC_AXIS_ARM;
        CHECK(first_axis.accept(intent, true, 0) && second_axis.accept(intent, true, 0));
        first_axis.stop(3);
        first_axis.publish(0, 20);
        second_axis.publish(0, 20);
        CHECK(!(axis_feedback(0)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY));
        CHECK(axis_feedback(7)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(axis_feedback(0)->BridgeFaultCode == 3 && !axis_feedback(7)->BridgeFaultCode);
        CHECK(first_axis.configure(false, 0));
        CHECK(!first_axis.configured() && second_axis.configured());
        CHECK(!first_axis.configure(true, 2));
        CHECK(!first_axis.configured());
        CHECK(!commit_parameter(ENCOS_AXIS_COUNT, false, 0, true));
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_rearm")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(axis.configure(true, 0));
        mc_axis_rx_t command{};
        CHECK(axis.accept(command, true, 0));
        command.ControlWord = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 0));
        axis.stop(0);
        CHECK(!axis.accept(command, true, 0));
        command.ControlWord = MC_AXIS_STOP;
        CHECK(axis.accept(command, true, 0));
        command.ControlWord = MC_AXIS_ARM;
        CHECK(!axis.accept(command, true, 0));
        command.ControlWord = 0;
        CHECK(axis.accept(command, false, 0));
        command.ControlWord = MC_AXIS_ARM;
        CHECK(!axis.accept(command, true, 0));
        command.ControlWord = 0;
        CHECK(axis.accept(command, true, 0));
        command.ControlWord = MC_AXIS_ARM;
        CHECK(axis.accept(command, true, 0));
        axis.publish(0, 20);
        CHECK(axis_feedback(0)->StatusWord & MC_STATUS_ARMED_QUERY_ONLY);
        CHECK(!axis.active() && !axis.control_sent());
        return 0;
    }
    if (!std::strcmp(argv[1], "axis_config_snapshot")) {
        configure_axis(0);
        dcu::app::AxisController axis;
        axis.initialize(0);
        CHECK(axis.configure(true, 0));
        CHECK(parameters::start_query(0, 0, 3, 1, 1, dcu::app::ParameterDialect::EncosLegacy, 1, 0));
        parameters::mark_sent(0, 1);
        dcu_can_frame_t response{};
        response.id = 1;
        response.dlc = 6;
        response.data[0] = 0xA0;
        response.data[1] = 3;
        response.data[2] = 0x3F;
        response.data[3] = 0x80;
        axis.consume_feedback(response, 1, 1);
        axis.publish(1, 20);
        const auto torque = axis_feedback(0)->EstimatedTorque;
        CHECK(torque > 0 && axis_feedback(0)->ActualCurrent == 100);
        object<mc_basic_t>(0x3001)->TorqueConstantRaw = 100;
        *object<mc_range_t>(0x3002) = {};
        object<mc_routing_t>(0x3000)->CanId = 2;
        axis.publish(1, 20);
        CHECK(axis_feedback(0)->EstimatedTorque == torque);
        CHECK(axis.matches(1) && !axis.matches(2));
        CHECK(!axis.configure(true, 0));
        CHECK(!axis.configured());
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
        dcu::app::AxisController axes[ENCOS_AXIS_COUNT];
        dcu::app::BusScheduler budgets[ENCOS_CHANNEL_COUNT];
        mc_channel_config_t channels[ENCOS_CHANNEL_COUNT]{};
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            budgets[c].reset_budget(0);
            channels[c].Role = 2;
            channels[c].RawRateLimit = 1;
        }
        dcu::app::RawCanCycle cycle{1, 1, true, nullptr};
        auto *raw = object<mc_raw_t>(0x7300);
        *raw = {};
        raw->ValidMask = 1;
        raw->SlotCount = 1;
        motor_pdo::RawSlot slot{};
        slot.id = 0x100;
        slot.dlc = 1;
        slot.data[0] = 42;
        motor_pdo::write_raw_slot(*raw, 0, slot);
        if (!std::strcmp(argv[1], "raw_costs")) {
            for (unsigned variant = 0; variant < 4; ++variant) {
                const bool fd_request = (variant & 1) != 0, fd_response = (variant & 2) != 0;
                slot.dlc = 8;
                slot.flags = fd_request ? 24 : 0;
                channels[0].FrameFormat = fd_response ? 1 : 0;
                motor_pdo::write_raw_slot(*raw, 0, slot);
                const unsigned cost = (fd_request ? 64U : 138U) + (fd_response ? 64U : 138U);
                budgets[0].reset_budget(2 * variant);
                budgets[0].charge(motor_cfg::can_budget_cap_us - cost + 1);
                gateway.stage_batch(2 * variant + 1, true);
                CHECK(gateway.process(cycle, 2 * variant, channels, axes, budgets).total == 1);
                CHECK(sent.size() == variant && budgets[0].can_afford(cost - 1));
                budgets[0].reset_budget(2 * variant);
                budgets[0].charge(motor_cfg::can_budget_cap_us - cost);
                gateway.stage_batch(2 * variant + 2, true);
                CHECK(gateway.process(cycle, 2 * variant, channels, axes, budgets).total == 0);
                CHECK(sent.size() == variant + 1 && !budgets[0].can_afford(1));
            }
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_service_hold")) {
            object<mc_routing_t>(0x3000)->ModelProfileId = 0x8000;
            auto *maintenance = object<mc_maintenance_t>(0x3008);
            maintenance->Opcode = 3;
            maintenance->RequestSeq = 1;
            CHECK(parameters::commit(0, true, 0, true, false));
            parameters::mark_sent(0, 1);
            parameters::cancel(0, RESULT_IO);
            CHECK(!parameters::inspect(0).busy && !parameters::available(0, 500));
            gateway.stage_batch(1, true);
            CHECK(gateway.process(cycle, 500, channels, axes, budgets).total == 1 && sent.empty());
            CHECK(parameters::available(0, 501));
            gateway.stage_batch(2, true);
            CHECK(gateway.process(cycle, 501, channels, axes, budgets).total == 0 && sent.size() == 1);
            maintenance->RequestSeq = 2;
            CHECK(parameters::commit(0, true, 501, true, false));
            parameters::mark_sent(0, 501);
            parameters::reset_session(RESULT_STATE);
            CHECK(parameters::available(0, 502));
            CHECK(maintenance->Result == RESULT_STATE && maintenance->DoneSeq == 2);
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_limits")) {
            raw->ValidMask = 3;
            raw->SlotCount = 2;
            slot.id = 0x101;
            motor_pdo::write_raw_slot(*raw, 1, slot);
            gateway.stage_batch(1, true);
            auto drops = gateway.process(cycle, 0, channels, axes, budgets);
            CHECK(sent.size() == 1 && drops.total == 1 && drops.channel[0] == 1);
            gateway.stage_batch(1, true);
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 0 && sent.size() == 1);
            gateway.stage_batch(2, true);
            CHECK(gateway.process(cycle, 1, channels, axes, budgets).total == 2 && sent.size() == 1);
            gateway.stage_batch(3, true);
            CHECK(gateway.process(cycle, 2, channels, axes, budgets).total == 1 && sent.size() == 2);
            return 0;
        }
        if (!std::strcmp(argv[1], "raw_filter")) {
            configure_axis(0);
            axes[0].initialize(0);
            CHECK(axes[0].configure(true, 0));
            slot.id = 1;
            motor_pdo::write_raw_slot(*raw, 0, slot);
            gateway.stage_batch(1, true);
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 1 && sent.empty());
            slot.id = 0x7FF;
            motor_pdo::write_raw_slot(*raw, 0, slot);
            gateway.stage_batch(2, true);
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 1 && sent.empty());
            slot.id = 0x100;
            motor_pdo::write_raw_slot(*raw, 0, slot);
            const unsigned cost = dcu::app::BusScheduler::frame_cost(false, 1) +
                                  dcu::app::BusScheduler::frame_cost(false, 8);
            budgets[0].charge(motor_cfg::can_budget_cap_us - cost);
            gateway.stage_batch(3, true);
            allow_send = false;
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 1 && sent.empty());
            CHECK(budgets[0].can_afford(cost));
            allow_send = true;
            gateway.stage_batch(4, true);
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 0 && sent.size() == 1);
            CHECK(!budgets[0].can_afford(cost));
            gateway.stage_batch(5, true);
            cycle.permitted = false;
            CHECK(gateway.process(cycle, 2, channels, axes, budgets).total == 0 && sent.size() == 1);
            cycle.permitted = true;
            CHECK(gateway.process(cycle, 2, channels, axes, budgets).total == 0 && sent.size() == 1);
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
            cycle.profile = 2;
            cycle.pdo = data;
            gateway.stage_full(true);
            CHECK(gateway.process(cycle, 0, channels, axes, budgets).total == 0 && sent.size() == 1);
            gateway.stage_full(true);
            CHECK(gateway.process(cycle, 2, channels, axes, budgets).total == 0 && sent.size() == 1);
            gateway.reset_session();
            gateway.stage_full(true);
            CHECK(gateway.process(cycle, 2, channels, axes, budgets).total == 0 && sent.size() == 2);
            return 0;
        }
    }
    if (!std::strcmp(argv[1], "startup_gate")) {
        configure_axis(0);
        CHECK(apply_configuration());
        object<mc_owner_t>(0x2003)->Owner = OWNER_PDO;
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
