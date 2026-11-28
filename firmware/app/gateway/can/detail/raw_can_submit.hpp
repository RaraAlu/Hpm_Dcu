#ifndef DCU_RAW_CAN_SUBMIT_HPP
#define DCU_RAW_CAN_SUBMIT_HPP

#include "can/can_bus_scheduler.hpp"
#include "can_transport.h"
#include "can/raw_can_gateway.hpp"
#include "dcu_sideband.hpp"

namespace dcu::app::gateway_application::detail {

static_assert(DCU_DEVICE_CAN_BUS_COUNT <= config::raw_can::bus_capacity);
static_assert(DCU_MOTOR_DEVICES_PER_BUS + 1 <= config::raw_can::protected_id_capacity);

inline RawCanSubmit raw_event_submit()
{
    return {nullptr, [](void *, unsigned bus, const dcu_can_frame_t &frame, bool) {
        return dcu_can_submit(bus, DCU_CAN_DEVICE_RAW, &frame) != 0;
    }};
}

template<bool paced = false>
inline RawCanSubmit raw_submit(BusScheduler (&schedule)[DCU_DEVICE_CAN_BUS_COUNT])
{
    return {schedule, [](void *context, unsigned bus, const dcu_can_frame_t &frame, bool fd_response) {
        if (bus >= DCU_DEVICE_CAN_BUS_COUNT || !dcu_can_frame_valid(&frame) ||
            frame.dlc > DCU_CAN_CLASSIC_MAX_PAYLOAD) return false;
        if constexpr (paced) {
            if (!dcu_profile_pdo::sideband_tx_allowed(bus, frame.id, frame.dlc, frame.flags)) return false;
        }
        const bool fd = (frame.flags & DCU_CAN_FLAG_FD) != 0;
        auto &budget = static_cast<BusScheduler *>(context)[bus];
        const unsigned cost = BusScheduler::frame_cost(fd, dcu_can_dlc_to_bytes(frame.dlc, fd)) +
                              BusScheduler::frame_cost(fd_response, 8);
        if ((paced && !dcu_can_tx_serial_ready(bus)) || !budget.can_afford(cost) ||
            !dcu_can_submit(bus, DCU_CAN_DEVICE_RAW, &frame)) return false;
        budget.charge(cost);
        return true;
    }};
}

}
#endif
