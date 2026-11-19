#include "cih408_mit_once.hpp"
#include <algorithm>
#include <cassert>

using namespace cih408_mit_once;

namespace {
constexpr unsigned bus = 1;
constexpr uint16_t node = 1;
constexpr uint32_t seq = 37;
constexpr uint64_t token = 97;
constexpr uint64_t deadline = 601;

dcu_can_frame_t reply()
{
    // This mode-1 payload was observed after a 0x11 in the wheeled-project trace.
    dcu_can_frame_t f{node, 10, fd_brs,
        {0x80, 0x10, 0x00, 0x40, 0x7f, 0x94, 0x80, 0xbc, 0x71, 0xb3, 0x7c, 0, 0, 0x4e, 0x4e, 0x61}};
    return f;
}

void start(Exchange &exchange)
{
    dcu_can_frame_t f{};
    assert(exchange.dispatch(request(seq), true, true, 80, deadline, f));
    const Status expected{0x11};
    assert(f.id == node && f.flags == fd_brs && f.dlc == dlc16);
    assert(dcu_can_frame_valid(&f) && dcu_can_dlc_to_bytes(f.dlc, true) == expected.size());
    assert(std::equal(expected.begin(), expected.end(), f.data));
    exchange.submitted(token);
}

void round_trip()
{
    Exchange exchange(bus, node, 0);
    dcu_can_frame_t tx{};
    auto invalid = request(seq);
    invalid.back() = 1;
    assert(!exchange.dispatch(invalid, true, true, 80, deadline, tx));
    assert(!exchange.dispatch(request(seq), false, true, 80, deadline, tx));
    assert(!exchange.dispatch(request(seq), true, false, 80, deadline, tx));
    assert(!exchange.dispatch(request(0), true, true, 80, deadline, tx));
    TxPdo sm3{};
    Feedback feedback{};
    assert(!collect(sm3, seq, 3, true, feedback));

    start(exchange);
    assert(!exchange.dispatch(request(seq + 1), true, true, 90, deadline, tx));
    auto f = reply();
    assert(!exchange.rx(bus, f, 147));
    exchange.tx_event(bus + 1, {token, DCU_CAN_TX_COMPLETED}, 147);
    exchange.tx_event(bus, {token + 1, DCU_CAN_TX_COMPLETED}, 147);
    assert(!exchange.rx(bus, f, 148));
    exchange.tx_event(bus, {token, DCU_CAN_TX_COMPLETED}, 147);
    assert(!exchange.rx(bus, f, 146));
    assert(!exchange.rx(bus, f, 147));
    assert(!exchange.rx(bus + 1, f, 214));
    for (unsigned bad = 0; bad < 4; ++bad) {
        auto wrong = f;
        switch (bad) {
        case 0: ++wrong.id; break;
        case 1: wrong.flags |= DCU_CAN_FLAG_EXTENDED; break;
        case 2: --wrong.dlc; break;
        case 3: wrong.data[0] = 0xa2; break;
        }
        assert(!exchange.rx(bus, wrong, 214));
    }
    assert(!exchange.publish(true, 230, sm3));
    assert(exchange.rx(bus, f, 214));
    assert(!exchange.rx(bus, f, 215));
    assert(!exchange.publish(true, 213, sm3));
    assert(exchange.publish(true, 400, sm3));
    assert(!exchange.publish(true, 401, sm3));
    assert(!exchange.dispatch(request(seq), true, true, 405, deadline, tx));
    assert(!collect(sm3, seq + 1, 3, true, feedback));
    assert(!collect(sm3, seq, 1, true, feedback));
    assert(!collect(sm3, seq, 3, false, feedback));
    assert(collect(sm3, seq, 3, true, feedback));
    assert(std::equal(feedback.bytes.begin(), feedback.bytes.end(), f.data));
    assert(feedback.mode() == 1 && feedback.alarm() == 0 && !feedback.mit_active());
    feedback.bytes[1] = 0x21;
    assert(feedback.mode() == 2 && feedback.alarm() == 256 && !feedback.mit_active());
    feedback.bytes[1] = 0x20;
    assert(feedback.mit_active());
}

void failure_is_terminal()
{
    for (unsigned scenario = 0; scenario < 5; ++scenario) {
        Exchange exchange(bus, node, 0);
        dcu_can_frame_t tx{};
        TxPdo sm3{};
        if (scenario == 0) {
            assert(exchange.dispatch(request(seq), true, true, 80, deadline, tx));
            exchange.submitted(0);
        } else {
            start(exchange);
            exchange.tx_event(bus, {token, scenario == 1 ? DCU_CAN_TX_CANCELLED : DCU_CAN_TX_COMPLETED}, 147);
            if (scenario == 2) {
                exchange.poll(false, 170);
            } else if (scenario == 3) {
                assert(!exchange.rx(bus, reply(), deadline));
                exchange.poll(true, deadline);
            } else if (scenario == 4) {
                assert(exchange.rx(bus, reply(), 214));
                assert(!exchange.publish(true, deadline, sm3));
            }
        }
        assert(!exchange.rx(bus, reply(), 500));
        assert(!exchange.publish(true, 500, sm3));
        assert(!exchange.dispatch(request(seq + 1), true, true, 501, deadline, tx));
        assert(sm3 == TxPdo{});
    }
}
}

int main()
{
    round_trip();
    failure_is_terminal();
    Exchange explicit_counter(bus, node, 0x5a);
    dcu_can_frame_t f{};
    assert(explicit_counter.dispatch(request(seq), true, true, 80, deadline, f));
    assert(f.data[0] == 0x11 && f.data[15] == 0x5a);
    assert(std::all_of(f.data + 1, f.data + 15, [](uint8_t byte) { return byte == 0; }));
}
