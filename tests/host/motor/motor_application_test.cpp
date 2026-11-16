#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "motor_application.cpp"
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
static uint32_t test_ms, cancellations;
static uint16_t test_input_size;

uint32_t ethercat_canfd_now_ms(void) { return test_ms; }
uint32_t ethercat_canfd_now_us(void) { return test_ms * 1000U; }
uint32_t ethercat_canfd_rx_drops(unsigned) { return 0; }
uint32_t ethercat_canfd_completions(unsigned) { return 0; }
uint16_t ethercat_canfd_bus_state(uint32_t) { return 0; }
bool ethercat_canfd_receive(unsigned, dcu_can_frame_t *, uint32_t *) { return false; }
bool ethercat_canfd_pending(unsigned) { return false; }
bool ethercat_canfd_axis_pending(unsigned) { return false; }
unsigned ethercat_canfd_tx_free(unsigned) { return 8; }
uint32_t ethercat_canfd_tx_progress(unsigned) { return 0; }
void ethercat_canfd_cancel(unsigned) { ++cancellations; }
void ethercat_canfd_cancel_axis(unsigned) { ++cancellations; }
void ethercat_canfd_cancel_service(unsigned) { ++cancellations; }
bool ethercat_canfd_send(unsigned, const dcu_can_frame_t *, unsigned) { return true; }

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

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "config_source")) {
        auto *config = object<mc_board_config_t>(0x2001);
        config->MaxCommandLeaseMs = 13;
        config->FeedbackTimeoutMs = 27;
        motor_control_init();
        CHECK(state.config.MaxCommandLeaseMs == 13 && state.config.FeedbackTimeoutMs == 27);
        return 0;
    }
    motor_control_init();
    select_wire_profile(1);
    uint8_t first[1072], second[1072];
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
                motor_control_process();
                encos_frame_t response{};
                response.id = 1;
                response.dlc = 6;
                response.data[0] = 0xA0;
                response.data[1] = k;
                response.data[2] = 0x3F;
                response.data[3] = 0x80;
                consume_feedback(0, &response, test_ms, test_ms);
                CHECK(transaction->Result == RESULT_OK && transaction->DoneSeq == sequence);
                auto *feedback = axis_feedback(0);
                int32_t actual = k == 1 ? feedback->ActualPosition : k == 2 ? feedback->ActualVelocity :
                    k == 3 ? feedback->ActualCurrent : feedback->ActualPower;
                CHECK(actual == expected[k - 1]);
            }
            return 0;
        }
        if (!std::strcmp(argv[1], "service_maintenance")) {
            auto *maintenance = object<mc_maintenance_t>(0x3008);
            maintenance->Opcode = 3;
            CHECK(MC_OD_Write(0x3008, 4, sizeof(sequence), reinterpret_cast<uint16_t *>(&sequence), 0) == 0);
            test_ms = 1;
            motor_control_process();
            encos_frame_t response{};
            response.id = 0x7FF;
            response.dlc = 4;
            response.data[1] = 1;
            response.data[2] = 1;
            response.data[3] = 3;
            consume_feedback(0, &response, test_ms, test_ms);
            CHECK(maintenance->DoneSeq == sequence && maintenance->Result == RESULT_OK);
            CHECK(!parameters::inspect(0).busy && parameters::inspect(0).hold_until == 501);
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
                      adjacent.axis == after.axis && adjacent.started == after.started);
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
        motor_control_process();
        CHECK(parameters::inspect(0).sent);
        if (!std::strcmp(argv[1], "service_timeout")) {
            test_ms = 101;
            motor_control_process();
            CHECK(!parameters::inspect(0).busy && transaction->Result == RESULT_TIMEOUT);
            CHECK(transaction->DoneSeq == sequence);
            return 0;
        }
        encos_frame_t response{};
        response.id = 1;
        response.dlc = 3;
        response.data[0] = 0x80;
        response.data[1] = 4;
        response.data[2] = 1;
        consume_feedback(0, &response, test_ms, test_ms);
        CHECK(axis_diagnostics(0)->ProtocolErrorCount == 0);
        CHECK(parameters::pending(0) && parameters::pending(0)->dlc == 2 && !parameters::inspect(0).sent);
        test_ms = 2;
        motor_control_process();
        response.dlc = 4;
        response.data[0] = 0xA0;
        response.data[1] = 22;
        bool mismatch = !std::strcmp(argv[1], "service_mismatch");
        response.data[2] = 0;
        response.data[3] = mismatch ? 51 : 50;
        consume_feedback(0, &response, test_ms, test_ms);
        CHECK(!parameters::inspect(0).busy && transaction->DoneSeq == sequence);
        CHECK(transaction->Result == (mismatch ? RESULT_MISMATCH : RESULT_OK));
        return 0;
    }
    if (!std::strcmp(argv[1], "raw_queue")) {
        encos_frame_t frame{};
        for (unsigned i = 0; i < motor_cfg::raw_queue_capacity + 3; ++i) {
            frame.id = i;
            raw_push(0, &frame);
        }
        CHECK(state.raw_rx[0].drops == 4);
        for (unsigned i = 0; i < motor_cfg::raw_queue_capacity - 1; ++i) {
            CHECK(raw_pop(0, &frame) && frame.id == i);
        }
        CHECK(!raw_pop(0, &frame));
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
        test_ms = 100;
        state.axes[0].active = state.axes[0].feedback_seen = state.axes[0].new_feedback = true;
        state.axes[0].feedback_ms = 99;
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
        motor_control_operational(true);
        state.axes[0].active = state.axes[0].armed = true;
        state.pending = true;
        motor_control_cycle_fault();
        CHECK(!state.operational && !state.pending && !state.axes[0].active && !state.axes[0].armed);
        CHECK(cancellations != 0);
        motor_control_prepare_input();
        CHECK(object<mc_board_tx_t>(0x6000)->BoardStatus & 0x40U);
        CHECK(!(object<mc_board_tx_t>(0x6000)->BoardStatus & 2U));
        return 0;
    }
    if (!std::strcmp(argv[1], "invalid_stop")) {
        motor_control_operational(true);
        state.axes[0].active = state.axes[0].armed = true;
        uint8_t command[784] = {};
        mc_board_rx_t board{};
        board.BoardControl = 2U;
        board.InterfaceVersion = 0; /* Even invalid PDO must not suppress Stop. */
        std::memcpy(command, &board, sizeof(board));
        motor_control_output(command);
        CHECK(state.axes[0].active);
        test_ms = 1;
        motor_control_process();
        CHECK(!state.axes[0].active && !state.axes[0].armed && !state.pending);
        return 0;
    }
    return 2;
}
