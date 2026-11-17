#ifndef DCU_TEST_ENCOS_MOTION_REFERENCE_HPP
#define DCU_TEST_ENCOS_MOTION_REFERENCE_HPP

#include "encos_motor.h"

namespace legacy_reference {
bool encode(const mc_axis_rx_t *, const mc_routing_t *, const encos_profile_t *, encos_frame_t *);
bool feedback(const encos_frame_t *, const encos_profile_t *, mc_axis_tx_t *);
}
#endif
