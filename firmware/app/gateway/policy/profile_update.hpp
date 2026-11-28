#ifndef DCU_PROFILE_UPDATE_HPP
#define DCU_PROFILE_UPDATE_HPP

#include <cstdint>

namespace dcu::app::profile_update {

struct Policy {
    uint16_t max_command_lease_ms{}, feedback_timeout_ms{};
    uint32_t readiness_ttl_ms{};
    bool operator==(const Policy &other) const
    {
        return max_command_lease_ms == other.max_command_lease_ms &&
               feedback_timeout_ms == other.feedback_timeout_ms &&
               readiness_ttl_ms == other.readiness_ttl_ms;
    }
};

struct Candidate {
    uint32_t profile_id, base_revision, revision;
    Policy policy;
};

enum class Status : uint16_t {
    Idle, Staged, Applied, Invalid, Stale, Unchanged, Aborted,
};

bool valid(const Policy &);
// A malformed replacement (nullptr) invalidates any previously staged revision.
bool stage(const Candidate *candidate);
bool commit(uint32_t revision);
void discard();
Status status();
uint32_t staged_revision();

}
#endif
