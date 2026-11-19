#pragma once

#include "one_cycle_pdo.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>

namespace one_cycle {
// The FRMW samples SM2 frame arrival, not the RK send call. The target is the
// early, working part of the previous board trace, not a shift of the ESC edge.
class ReferenceFollower {
public:
    static constexpr int target_sof_ns = 37000;
    static constexpr int acquire_limit_ns = 2000, tracking_limit_ns = 500;
    int step_ns() const { return step_; }
    bool locked() const { return good_ >= 8; }

    bool observe(uint64_t &dc_tick, uint32_t seq, uint32_t sof, bool acquisition)
    {
        constexpr int period = static_cast<int>(period_ns);
        int64_t error = int32_t(sof - uint32_t(dc_tick + target_sof_ns));
        // Ignore a free-running pre-configuration clock until IgH has set the DC offset.
        if (std::abs(error) > 8 * period) return false;
        if (have_last_) {
            const auto elapsed = uint32_t(sof - last_sof_);
            const auto expected = uint64_t(seq - last_seq_) * period;
            if (!elapsed || std::abs(int64_t(elapsed) - int64_t(expected)) > period / 3) {
                if (acquisition && elapsed) {
                    have_last_ = false;
                    used_ = good_ = 0;
                    integral_ = step_ = 0;
                }
                return false;
            }
        } else {
            if (!acquisition) return false;
            // Choose the nearest hardware cycle once during acquisition. The
            // logical DC grid never moves by a cycle during scored exchanges.
            const auto whole = (error + (error >= 0 ? period / 2 : -period / 2)) / period;
            dc_tick = uint64_t(int64_t(dc_tick) + whole * period);
            error -= whole * period;
        }
        if (std::abs(error) >= period / 2) return false;
        have_last_ = true;
        last_sof_ = sof;
        last_seq_ = seq;
        errors_[used_++ % errors_.size()] = int(error);
        if (used_ < errors_.size()) return true;
        auto ordered = errors_;
        std::sort(ordered.begin(), ordered.end());
        const auto median = ordered[ordered.size() / 2];
        good_ = std::abs(median) <= 4000 ? std::min(8U, good_ + 1) : 0;
        const int limit = acquisition ? acquire_limit_ns : tracking_limit_ns;
        if (std::abs(median) < 8000)
            integral_ = std::clamp(integral_ + median, -512 * 300, 512 * 300);
        // Positive phase error means late at the ESC: shorten a future RK period.
        step_ = -std::clamp(median / 16 + integral_ / 512, -limit, limit);
        return true;
    }
private:
    std::array<int, 5> errors_{};
    uint32_t last_seq_{}, last_sof_{};
    unsigned used_{}, good_{};
    int integral_{}, step_{};
    bool have_last_{};
};
}
