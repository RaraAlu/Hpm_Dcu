#include "profile_update.hpp"
#include "gateway_application.hpp"
#include "realtime_port.h"

namespace dcu::app::profile_update {
namespace application = gateway_application;

namespace {
Candidate pending{};
Status result = Status::Idle;
uint32_t committed{};
uint32_t staged_at_ms{};

bool matches_active(const Candidate &candidate)
{
    const auto active = application::readiness();
    return active.boot_id && candidate.boot_id == active.boot_id &&
           candidate.profile_id == active.profile_id && candidate.base_revision == active.revision &&
           candidate.revision > active.revision;
}

void expire()
{
    constexpr uint32_t stage_ttl_ms = 5000;
    if (pending.revision && dcu_time_now_ms() - staged_at_ms >= stage_ttl_ms)
        discard();
}
}

bool valid(const Policy &policy)
{
    return policy.max_command_lease_ms >= 2 && policy.max_command_lease_ms <= 10 &&
           policy.feedback_timeout_ms >= 5 && policy.feedback_timeout_ms <= 20 &&
           policy.readiness_ttl_ms >= 100 && policy.readiness_ttl_ms <= 10000;
}

bool stage(const Candidate *candidate)
{
    pending = {};
    result = Status::Invalid;
    if (!candidate || !valid(candidate->policy)) return false;
    const auto &next = *candidate;
    if (!matches_active(next)) {
        result = Status::Stale;
        return false;
    }
    if (next.policy == application::profile_policy()) {
        result = Status::Unchanged;
        return true;
    }
    pending = next;
    staged_at_ms = dcu_time_now_ms();
    result = Status::Staged;
    return true;
}

bool commit(uint32_t revision)
{
    expire();
    if (revision && revision == committed && !pending.revision &&
        application::readiness().revision == revision) {
        result = Status::Applied;
        return true;
    }
    if (!pending.revision || revision != pending.revision || !matches_active(pending)) {
        result = Status::Stale;
        pending = {};
        return false;
    }
    const auto candidate = pending;
    pending = {};
    if (!application::apply_profile_policy(candidate.profile_id, candidate.base_revision,
                                           candidate.revision, candidate.policy)) {
        result = Status::Aborted;
        return false;
    }
    committed = revision;
    result = Status::Applied;
    return true;
}

void discard()
{
    if (!pending.revision) return;
    pending = {};
    result = Status::Aborted;
}

Status status() { expire(); return result; }
uint32_t staged_revision() { expire(); return pending.revision; }

}
