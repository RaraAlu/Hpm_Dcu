#pragma once

#include "can_frame.h"

namespace dcu {
// Bound once by the production composition root. All callbacks run on the gateway task.
struct CyclicCanObserver {
    unsigned bus;
    void (*start)(uint32_t now);
    void (*stop)();
    void (*receive)(const dcu_can_frame_t &frame, uint32_t observed_ms);
    uint16_t (*channel_status)(uint32_t now);
};
}
