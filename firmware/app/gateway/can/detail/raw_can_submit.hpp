#ifndef DCU_RAW_CAN_SUBMIT_HPP
#define DCU_RAW_CAN_SUBMIT_HPP

#include "can/can_bus_scheduler.hpp"
#include "can_transport.h"
#include "can/raw_can_gateway.hpp"

namespace dcu::app::gateway_application::detail {

static_assert(DCU_DEVICE_CAN_BUS_COUNT <= config::raw_can::bus_capacity);
static_assert(DCU_MOTOR_DEVICES_PER_BUS + 1 <= config::raw_can::protected_id_capacity);

inline RawCanSubmit raw_submit(BusScheduler (&schedule)[DCU_DEVICE_CAN_BUS_COUNT])
{
    return {schedule, [](void *context, unsigned bus, const dcu_can_frame_t &frame, bool fd_response) {
        if (bus >= DCU_DEVICE_CAN_BUS_COUNT || frame.dlc > DCU_CAN_CLASSIC_MAX_PAYLOAD) return false;
        auto &budget = static_cast<BusScheduler *>(context)[bus];
        const unsigned cost = BusScheduler::frame_cost((frame.flags & 8u) != 0, frame.dlc) +
                              BusScheduler::frame_cost(fd_response, 8);
        if (!budget.can_afford(cost) || !dcu_can_submit(bus, &frame)) return false;
        budget.charge(cost);
        return true;
    }};
}

}
#endif
