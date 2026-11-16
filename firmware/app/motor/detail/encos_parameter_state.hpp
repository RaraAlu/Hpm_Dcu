#ifndef DCU_ENCOS_PARAMETER_STATE_HPP
#define DCU_ENCOS_PARAMETER_STATE_HPP

#include "encos_motor.h"
#include <cstdint>

namespace dcu::app::parameters::detail {

enum class Phase : uint8_t { Idle,
                             Queued,
                             AwaitingReply };
enum class Purpose : uint8_t { Query,
                               Write,
                               Verify,
                               Maintenance };
struct service_t {
    Phase phase;
    Purpose purpose;
    bool internal;
    uint8_t axis, query, group, uuid_mask;
    uint16_t parameter;
    uint32_t seq, started, hold_until, uuid[3];
    encos_frame_t request;
    uint8_t expected[6], expected_size;
};

}
#endif
