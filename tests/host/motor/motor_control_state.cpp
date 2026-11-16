#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "motor_control.cpp"

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

UINT8 nAlStatus = STATE_PREOP;
TMC_ASSIGN gRxAssign{}, gTxAssign{};
static uint32_t test_ms, cancellations;

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

static void select_profile(unsigned profile)
{
    uint16_t indices[39];
    gRxAssign.u16SubIndex0 = mapping_indices(profile, false, indices);
    std::memcpy(gRxAssign.aEntries, indices, gRxAssign.u16SubIndex0 * 2U);
    gTxAssign.u16SubIndex0 = mapping_indices(profile, true, indices);
    std::memcpy(gTxAssign.aEntries, indices, gTxAssign.u16SubIndex0 * 2U);
    uint16_t in = 0, out = 0;
    CHECK(motor_control_mapping(&in, &out) == 0);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    motor_control_init();
    select_profile(1);
    uint8_t first[1072], second[1072];
    if (!std::strcmp(argv[1], "same_ms")) {
        motor_control_prepare_input();
        const uint32_t sequence = OD(board_tx, 0x6000)->PublishSeq;
        motor_control_input(first);
        motor_control_input(second);
        CHECK(!std::memcmp(first, second, input_size));
        CHECK(OD(board_tx, 0x6000)->PublishSeq == sequence);
        motor_control_prepare_input();
        CHECK(OD(board_tx, 0x6000)->PublishSeq == sequence + 1U);
        test_ms = 1;
        motor_control_prepare_input();
        CHECK(OD(board_tx, 0x6000)->PublishSeq == sequence + 2U);
        return 0;
    }
    if (!std::strcmp(argv[1], "immutable")) {
        test_ms = 100;
        axes[0].active = axes[0].feedback_seen = axes[0].new_feedback = true;
        axes[0].feedback_ms = 99;
        motor_control_prepare_input();
        CHECK(STATE(0)->StatusWord & 16U);
        CHECK(STATE(0)->FeedbackAgeMs == 1U);
        motor_control_input(first);
        STATE(0)->ActualPosition = 12345;
        motor_control_input(second);
        CHECK(!std::memcmp(first, second, input_size));
        motor_control_prepare_input();
        CHECK(!(STATE(0)->StatusWord & 16U));
        motor_control_input(second);
        CHECK(std::memcmp(first, second, input_size) != 0);
        mc_axis_tx_t axis;
        std::memcpy(&axis, second + 64, sizeof(axis));
        CHECK(axis.ActualPosition == 12345 && axis.FeedbackAgeMs == 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "profiles")) {
        for (unsigned profile = 0; profile < 3; ++profile) {
            select_profile(profile);
            std::memset(first, 0xA5, sizeof(first));
            motor_control_prepare_input();
            motor_control_input(first);
            mc_board_tx_t board;
            std::memcpy(&board, first, sizeof(board));
            CHECK(board.PublishSeq == OD(board_tx, 0x6000)->PublishSeq);
            for (unsigned i = input_size; i < sizeof(first); ++i)
                CHECK(first[i] == 0xA5);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "cycle_fault")) {
        motor_control_operational(true);
        axes[0].active = axes[0].armed = true;
        pending = true;
        motor_control_cycle_fault();
        CHECK(!operational && !pending && !axes[0].active && !axes[0].armed);
        CHECK(cancellations != 0);
        motor_control_prepare_input();
        CHECK(OD(board_tx, 0x6000)->BoardStatus & 0x40U);
        CHECK(!(OD(board_tx, 0x6000)->BoardStatus & 2U));
        return 0;
    }
    if (!std::strcmp(argv[1], "invalid_stop")) {
        motor_control_operational(true);
        axes[0].active = axes[0].armed = true;
        uint8_t command[784] = {};
        mc_board_rx_t board{};
        board.BoardControl = 2U;
        board.InterfaceVersion = 0; /* Even invalid PDO must not suppress Stop. */
        std::memcpy(command, &board, sizeof(board));
        motor_control_output(command);
        CHECK(axes[0].active);
        test_ms = 1;
        motor_control_process();
        CHECK(!axes[0].active && !axes[0].armed && !pending);
        return 0;
    }
    return 2;
}
