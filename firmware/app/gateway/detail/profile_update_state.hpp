#ifndef DCU_PROFILE_UPDATE_STATE_HPP
#define DCU_PROFILE_UPDATE_STATE_HPP

#include "../profile_update.hpp"

namespace dcu::app::profile_update::detail {
struct Candidate {
    uint64_t boot_id;
    uint32_t profile_id, base_revision, revision;
    Policy policy;
};
}

#endif
