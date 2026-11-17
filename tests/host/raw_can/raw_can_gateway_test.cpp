#include "raw_can_gateway.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); std::exit(1); \
} } while (false)

using namespace dcu::app;
struct SentFrame { unsigned bus; dcu_can_frame_t frame; };
static std::vector<SentFrame> sent;
static bool allow_send = true;

extern "C" bool ethercat_canfd_send(unsigned bus, const dcu_can_frame_t *frame, unsigned tag)
{
    CHECK(tag == ENCOS_AXIS_COUNT);
    if (!allow_send) return false;
    sent.push_back({bus, *frame});
    return true;
}

struct Fixture {
    RawCanGateway gateway;
    RawCanPolicy policy{};
    RawCanBatch batch{};
    BusScheduler budgets[ENCOS_CHANNEL_COUNT];

    Fixture()
    {
        policy.permitted = true;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            policy.buses[c].can_send = true;
            policy.buses[c].rate_limit = 8;
            budgets[c].reset_budget(0);
        }
        batch.valid = true;
        batch.count = 1;
        batch.slots[0].frame.id = 100;
        batch.slots[0].frame.dlc = 1;
        batch.slots[0].frame.data[0] = 42;
    }
    RawCanDrops process(uint32_t sequence, uint32_t now = 0)
    {
        batch.sequence = sequence;
        return gateway.process(batch, policy, now, budgets);
    }
};

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    Fixture f;
    if (!std::strcmp(argv[1], "sideband_sequence")) {
        CHECK(f.process(UINT32_MAX - 1).total == 0 && sent.size() == 1);
        CHECK(f.process(UINT32_MAX - 1).total == 0 && sent.size() == 1);
        CHECK(f.process(UINT32_MAX - 2).total == 0 && sent.size() == 1);
        CHECK(f.process(0).total == 0 && sent.size() == 2);
        CHECK(f.process(0x80000000U).total == 0 && sent.size() == 2);
        f.policy.permitted = false;
        CHECK(f.process(1).total == 0 && sent.size() == 2);
        f.policy.permitted = true;
        CHECK(f.process(1).total == 0 && sent.size() == 2);
        f.batch.valid = false;
        CHECK(f.process(2).total == 0 && sent.size() == 2);
        f.batch.valid = true;
        CHECK(f.process(2).total == 0 && sent.size() == 2);
        CHECK(f.process(3).total == 0 && sent.size() == 3);
        f.gateway.reset_session();
        CHECK(f.process(1).total == 0 && sent.size() == 4);
        return 0;
    }
    if (!std::strcmp(argv[1], "full_bus_sequence")) {
        f.batch.kind = RawCanBatch::Kind::FullBus;
        f.batch.slots[0].bus = 3;
        f.batch.slots[0].frame.id = 0x7FF;
        CHECK(f.process(65535).total == 0 && sent.size() == 1 && sent.back().bus == 0);
        CHECK(f.process(65535 + 65536U).total == 0 && sent.size() == 1);
        CHECK(f.process(0).total == 0 && sent.size() == 2);
        CHECK(f.process(65535).total == 0 && sent.size() == 3);
        f.batch.bus = 1;
        CHECK(f.process(65535).total == 0 && sent.size() == 4 && sent.back().bus == 1);
        f.policy.permitted = false;
        CHECK(f.process(0).total == 0 && sent.size() == 4);
        f.policy.permitted = true;
        CHECK(f.process(0).total == 0 && sent.size() == 5);
        f.policy.buses[1].can_send = false;
        CHECK(f.process(1).channel[1] == 1 && sent.size() == 5);
        f.policy.buses[1].can_send = true;
        CHECK(f.process(1).total == 0 && sent.size() == 5);
        f.batch.valid = false;
        CHECK(f.process(2).total == 0 && sent.size() == 5);
        f.batch.valid = true;
        CHECK(f.process(2).total == 0 && sent.size() == 5);
        f.gateway.reset_session();
        CHECK(f.process(2).total == 0 && sent.size() == 6);
        return 0;
    }
    if (!std::strcmp(argv[1], "admission_bounds")) {
        f.policy.buses[0].protected_count = 1;
        f.policy.buses[0].protected_ids[0] = 100;
        CHECK(f.process(1).channel[0] == 1 && sent.empty());
        f.batch.slots[0].bus = 1;
        CHECK(f.process(2).total == 0 && sent.size() == 1);
        f.batch.slots[0].bus = ENCOS_CHANNEL_COUNT;
        auto drops = f.process(3);
        CHECK(drops.total == 1);
        for (auto count : drops.channel) CHECK(count == 0);
        f.batch.slots[0].bus = 0;
        f.policy.buses[0].protected_count = ENCOS_MOTORS_PER_CHANNEL + 1;
        CHECK(f.process(4).total == 1 && sent.size() == 1);
        f.policy.buses[0].protected_count = 0;
        f.batch.count = config::motor_control::raw_batch_capacity + 1;
        CHECK(f.process(5).total == 0 && sent.size() == 1);
        f.batch.count = 1;
        CHECK(f.process(5).total == 0 && sent.size() == 1);
        f.batch.kind = static_cast<RawCanBatch::Kind>(255);
        CHECK(f.process(6).total == 0 && sent.size() == 1);
        f.batch.kind = RawCanBatch::Kind::FullBus;
        f.batch.bus = ENCOS_CHANNEL_COUNT;
        CHECK(f.process(6).total == 0 && sent.size() == 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "rate_and_reset")) {
        f.policy.buses[0].rate_limit = 1;
        const unsigned cost = BusScheduler::frame_cost(false, 1) + BusScheduler::frame_cost(false, 8);
        f.budgets[0].charge(config::motor_control::can_budget_cap_us - 2 * cost);
        allow_send = false;
        CHECK(f.process(1, UINT32_MAX).total == 1 && f.budgets[0].can_afford(2 * cost));
        allow_send = true;
        CHECK(f.process(2, UINT32_MAX).total == 0 && sent.size() == 1);
        f.gateway.receive(0, f.batch.slots[0].frame);
        f.gateway.reset_session();
        CHECK(f.process(2, 0).total == 1 && sent.size() == 1);
        CHECK(f.process(3, 1).total == 0 && sent.size() == 2 && !f.budgets[0].can_afford(1));
        dcu_can_frame_t received{};
        CHECK(f.gateway.take(0, received) && received.id == 100 && received.data[0] == 42);
        CHECK(!f.gateway.take(0, received));
        return 0;
    }
    return 2;
}
