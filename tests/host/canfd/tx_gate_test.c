#include <stdio.h>
#include <stdlib.h>
#include "hpm_mcan_transport.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

MCAN_Type test_mcan[4];
GPIO_Type test_gpio;
static bool inhibited, during_stage, during_commit, deferred_fault, stage_failure;
static unsigned critical_depth, stage_calls, commits, cancel_calls, expected_tag;

static void fault_timer(void)
{
    if (critical_depth) {
        deferred_fault = true;
        return;
    }
    inhibited = true;
    dcu_can_cancel(0);
}

bool dcu_tx_inhibited(void) { return inhibited; }
uint32_t dcu_time_now_us(void) { return 0; }
uint32_t dcu_time_now_ms(void) { return 0; }
void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void)
{
    CHECK(critical_depth == 1);
    --critical_depth;
    if (deferred_fault)
        fault_timer();
}
void intc_m_disable_irq(unsigned irq) { CHECK(irq >= IRQn_MCAN0 && irq <= IRQn_MCAN3); }
void intc_m_enable_irq(unsigned irq) { CHECK(irq >= IRQn_MCAN0 && irq <= IRQn_MCAN3); }
void gpio_write_pin(GPIO_Type *base, unsigned index, unsigned pin, unsigned level)
{ (void)base; (void)index; (void)pin; (void)level; }
int mcan_get_protocol_status(MCAN_Type *base, mcan_protocol_status_t *status)
{ (void)base; memset(status, 0, sizeof(*status)); return 0; }
void mcan_cancel_tx_buf_send_request(MCAN_Type *base, unsigned index)
{ ++cancel_calls; base->TXBRP &= ~(1U << index); }
int mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index)
{
    (void)base;
    CHECK(critical_depth == 0 && frame->dlc == 8);
    ++stage_calls;
    *index = 2;
    if (during_stage)
        fault_timer();
    return stage_failure ? 1 : 0;
}
void mcan_send_add_request(MCAN_Type *base, uint32_t index)
{
    if (during_commit)
        fault_timer();
    CHECK(!inhibited && critical_depth == 1);
    CHECK(s_tx_axis[0][index] == expected_tag);
    ++commits;
    base->TXBRP |= 1U << index;
}

/* Also permits running this regression fixture against the pre-fix gateway. */
int mcan_transmit_via_txfifo_nonblocking(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index)
{
    int status = mcan_request_and_fill_txfifo(base, frame, index);
    if (!status)
        mcan_send_add_request(base, *index);
    return status;
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    memset(s_tx_axis, 255, sizeof(s_tx_axis));
    dcu_can_frame_t frame = {.id = 1, .dlc = 8};
    expected_tag = 3;
    if (!strcmp(argv[1], "staging_fault")) {
        during_stage = true;
        CHECK(!dcu_can_send(0, &frame, 3));
        CHECK(stage_calls == 1 && commits == 0 && s_tx_axis[0][2] == 255);
    } else if (!strcmp(argv[1], "commit_fault")) {
        during_commit = true;
        CHECK(dcu_can_send(0, &frame, 3));
        CHECK(commits == 1 && inhibited && cancel_calls == 8 && test_mcan[0].TXBRP == 0);
    } else if (!strcmp(argv[1], "already_inhibited")) {
        inhibited = true;
        CHECK(!dcu_can_send(0, &frame, 3));
        CHECK(stage_calls == 0 && commits == 0);
    } else if (!strcmp(argv[1], "stage_failure")) {
        stage_failure = true;
        CHECK(!dcu_can_send(0, &frame, 3));
        CHECK(commits == 0 && s_tx_axis[0][2] == 255);
    } else if (!strcmp(argv[1], "invalid")) {
        CHECK(!dcu_can_send(4, &frame, 3));
        CHECK(!dcu_can_send(0, NULL, 3));
        CHECK(stage_calls == 0 && commits == 0);
    } else {
        CHECK(!strcmp(argv[1], "normal"));
        CHECK(dcu_can_send(0, &frame, 3));
        CHECK(commits == 1 && stage_calls == 1 && test_mcan[0].TXBRP == 4);
    }
    CHECK(critical_depth == 0);
    return 0;
}
