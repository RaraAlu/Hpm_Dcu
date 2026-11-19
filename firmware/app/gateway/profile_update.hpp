#ifndef DCU_PROFILE_UPDATE_HPP
#define DCU_PROFILE_UPDATE_HPP

#include <cstddef>
#include <cstdint>

namespace dcu::app::profile_update {

inline constexpr uint16_t object_index = 0x2100;
inline constexpr unsigned candidate_bytes = 36;

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

enum class Status : uint16_t {
    Idle, Staged, Applied, Invalid, Stale, Unchanged, Aborted,
};

bool valid(const Policy &);
bool stage(const void *data, size_t bytes);
bool commit(uint32_t revision);
void discard();
Status status();
uint32_t staged_revision();

}
#endif
