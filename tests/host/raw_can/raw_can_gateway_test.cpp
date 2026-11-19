#include "raw_can_gateway.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expression); std::exit(1); \
} } while (false)

using namespace dcu::app;
namespace cfg = config::raw_can;
struct SentFrame { unsigned bus; dcu_can_frame_t frame; bool fd_response; };

struct Fixture {
    RawCanGateway gateway;
    RawCanPolicy policy{};
    RawCanBatch batch{};
    std::vector<SentFrame> sent;
    bool allow_send = true;

    Fixture()
    {
        policy.permitted = true;
        for (auto &bus : policy.buses) {
            bus.can_send = true;
            bus.rate_limit = 8;
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
        return gateway.process(batch, policy, now, {this, [](void *context, unsigned bus,
                 const dcu_can_frame_t &frame, bool fd_response) {
            auto &fixture = *static_cast<Fixture *>(context);
            if (!fixture.allow_send) return false;
            fixture.sent.push_back({bus, frame, fd_response});
            return true;
        }});
    }
};

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    Fixture f;
    const auto &sent = f.sent;
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
        f.policy.buses[0].protected_count = 1;
        f.policy.buses[0].protected_ids[0] = 0x7FF;
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
        f.batch.slots[0].bus = cfg::bus_capacity;
        auto drops = f.process(3);
        CHECK(drops.total == 1);
        for (auto count : drops.channel) CHECK(count == 0);
        f.batch.slots[0].bus = 0;
        f.policy.buses[0].protected_count = cfg::protected_id_capacity + 1;
        CHECK(f.process(4).total == 1 && sent.size() == 1);
        f.policy.buses[0].protected_count = 0;
        f.batch.count = cfg::batch_capacity + 1;
        CHECK(f.process(5).total == 0 && sent.size() == 1);
        f.batch.count = 1;
        CHECK(f.process(5).total == 0 && sent.size() == 1);
        f.batch.kind = static_cast<RawCanBatch::Kind>(255);
        CHECK(f.process(6).total == 0 && sent.size() == 1);
        f.batch.kind = RawCanBatch::Kind::FullBus;
        f.batch.bus = cfg::bus_capacity;
        CHECK(f.process(6).total == 0 && sent.size() == 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "rate_and_reset")) {
        f.policy.buses[0].rate_limit = 1;
        f.allow_send = false;
        CHECK(f.process(1, UINT32_MAX).total == 1 && sent.empty());
        f.allow_send = true;
        CHECK(f.process(2, UINT32_MAX).total == 0 && sent.size() == 1);
        f.gateway.receive(0, f.batch.slots[0].frame);
        f.gateway.reset_session();
        CHECK(f.process(2, 0).total == 1 && sent.size() == 1);
        CHECK(f.process(3, 1).total == 0 && sent.size() == 2);
        dcu_can_frame_t received{};
        CHECK(f.gateway.take(0, received) && received.id == 100 && received.data[0] == 42);
        CHECK(!f.gateway.take(0, received));
        return 0;
    }
    if (!std::strcmp(argv[1], "single_bus")) {
        f.policy = {};
        f.policy.permitted = true;
        f.policy.buses[2].can_send = true;
        f.policy.buses[2].rate_limit = 1;
        f.policy.buses[2].fd_response = true;
        CHECK(f.process(1).channel[0] == 1 && sent.empty());
        f.batch.slots[0].bus = 2;
        CHECK(f.process(2).total == 0 && sent.size() == 1);
        CHECK(sent[0].bus == 2 && sent[0].fd_response && sent[0].frame.data[0] == 42);
        f.batch.sequence = 3;
        CHECK(f.gateway.process(f.batch, f.policy, 2, {}).channel[2] == 1);
        CHECK(f.process(4, 2).total == 0 && sent.size() == 2);
        return 0;
    }
    return 2;
}
