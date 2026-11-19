#include "profile_update.hpp"
#include "detail/profile_update_state.hpp"
#include "gateway_application.hpp"
#include "realtime_port.h"

namespace dcu::app::profile_update {
namespace application = gateway_application;

namespace {
using detail::Candidate;
Candidate pending{};
Status result = Status::Idle;
uint32_t committed{};
uint32_t staged_at_ms{};

uint32_t load_le(const uint8_t *p, unsigned n)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < n; ++i) value |= uint32_t{p[i]} << (8U * i);
    return value;
}

uint32_t crc32(const uint8_t *p, unsigned n)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return ~crc;
}

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

bool stage(const void *data, size_t bytes)
{
    pending = {};
    result = Status::Invalid;
    if (!data || bytes != candidate_bytes) return false;
    const auto *p = static_cast<const uint8_t *>(data);
    if (load_le(p, 2) != 1 || load_le(p + 2, 2) != candidate_bytes ||
        crc32(p, candidate_bytes - 4) != load_le(p + 32, 4)) return false;
    const Candidate next{uint64_t{load_le(p + 20, 4)} << 32 | load_le(p + 16, 4),
                         load_le(p + 4, 4), load_le(p + 8, 4), load_le(p + 12, 4),
                         {static_cast<uint16_t>(load_le(p + 24, 2)),
                          static_cast<uint16_t>(load_le(p + 26, 2)), load_le(p + 28, 4)}};
    if (!valid(next.policy)) return false;
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
