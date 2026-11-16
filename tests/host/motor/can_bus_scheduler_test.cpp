#include "can_bus_scheduler.hpp"
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

using dcu::app::BusScheduler;
using dcu::app::ControlSet;

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    BusScheduler scheduler;
    ControlSet controls{3, 3, 1000};
    if (!std::strcmp(argv[1], "round_robin")) {
        CHECK(scheduler.next_control(controls, 100, 128, true) == -1);
        scheduler.reset_budget(0);
        CHECK(scheduler.next_control(controls, 100, 128, false) == -1);
        CHECK(scheduler.next_control(controls, 100, 128, true) == 0);
        CHECK(scheduler.next_control(controls, 100, 128, true) == 0);
        scheduler.control_submitted(0, 100, 1000, true, 128);
        controls.first_send_mask = 2;
        CHECK(scheduler.next_control(controls, 199, 128, true) == -1);
        CHECK(scheduler.next_control(controls, 200, 128, true) == 1);
        scheduler.control_submitted(1, 200, 1000, true, 128);
        controls.first_send_mask = 0;
        CHECK(scheduler.next_control(controls, 1099, 128, true) == -1);
        CHECK(scheduler.next_control(controls, 1100, 128, true) == 0);
    } else if (!std::strcmp(argv[1], "phase")) {
        scheduler.reset_budget(0);
        controls = {1, 0, 2000};
        scheduler.control_submitted(0, 100, 2000, true, 128);
        CHECK(scheduler.next_control(controls, 2150, 128, true) == 0);
        scheduler.control_submitted(0, 2150, 2000, false, 128);
        CHECK(scheduler.next_control(controls, 4099, 128, true) == -1);
        CHECK(scheduler.next_control(controls, 4100, 128, true) == 0);
        scheduler.control_submitted(0, 12000, 2000, false, 128);
        CHECK(scheduler.next_control(controls, 13999, 128, true) == -1);
        CHECK(scheduler.next_control(controls, 14000, 128, true) == 0);
    } else if (!std::strcmp(argv[1], "budget")) {
        scheduler.reset_budget(0);
        scheduler.charge(2390);
        CHECK(scheduler.can_afford(10) && !scheduler.can_afford(11));
        scheduler.refill(1);
        CHECK(scheduler.can_afford(810) && !scheduler.can_afford(811));
        scheduler.refill(2);
        CHECK(scheduler.can_afford(1610) && !scheduler.can_afford(1611));
        scheduler.refill(100);
        CHECK(scheduler.can_afford(2400) && !scheduler.can_afford(2401));
        scheduler.reset_budget(UINT32_MAX - 1);
        scheduler.charge(2400);
        scheduler.refill(1);
        CHECK(scheduler.can_afford(2400));
    } else if (!std::strcmp(argv[1], "query")) {
        CHECK(scheduler.query_window({}, 0, 120, 128));
        scheduler.reset_budget(0);
        controls = {1, 1, 2000};
        CHECK(!scheduler.query_window(controls, 1010, 120, 128));
        scheduler.control_submitted(0, 1000, 2000, true, 128);
        controls.first_send_mask = 0;
        CHECK(scheduler.query_window(controls, 1010, 120, 128));
        CHECK(!scheduler.query_window(controls, 2900, 120, 128));
        scheduler.charge(2100);
        CHECK(!scheduler.query_window(controls, 1010, 120, 128));
        BusScheduler loaded;
        loaded.reset_budget(0);
        CHECK(!loaded.query_window({0x7F, 0, 1000}, 50, 188, 276));
    } else if (!std::strcmp(argv[1], "progress")) {
        CHECK(!scheduler.stalled(10, 0, true, 20));
        CHECK(scheduler.stalled(20, 0, true, 20));
        CHECK(!scheduler.stalled(20, 1, true, 20));
        CHECK(!scheduler.stalled(39, 1, true, 20));
        CHECK(!scheduler.stalled(40, 1, false, 20));
        CHECK(!scheduler.stalled(UINT32_MAX - 3, 2, true, 20));
        CHECK(!scheduler.stalled(10, 2, true, 20));
        CHECK(scheduler.stalled(17, 2, true, 20));
    } else if (!std::strcmp(argv[1], "isolation")) {
        BusScheduler other;
        scheduler.reset_budget(0);
        other.reset_budget(0);
        scheduler.control_submitted(0, 100, 1000, true, 128);
        CHECK(scheduler.next_control(controls, 150, 128, true) == -1);
        CHECK(other.next_control(controls, 150, 128, true) == 0);
        CHECK(!scheduler.can_afford(2400) && other.can_afford(2400));
    } else if (!std::strcmp(argv[1], "frame_cost")) {
        CHECK(BusScheduler::frame_cost(true, 8) == 64);
        CHECK(BusScheduler::frame_cost(false, 8) == 138);
    } else return 2;
    return 0;
}
