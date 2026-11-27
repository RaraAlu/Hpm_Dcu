#ifndef DCU_HEAD_HEALTH_HPP
#define DCU_HEAD_HEALTH_HPP

#include "can_frame.h"
#include "cyclic_can_observer.hpp"

namespace dcu::head {

struct Health {
    bool monitoring{};
    uint8_t heartbeat_seen{}, offline{}, device_errors{};
    uint16_t feedback_seen{}, stale{};
    uint32_t last_error[4]{};
};

const CyclicCanObserver *profile_observer();
const Health &health(uint32_t now);

class Monitor {
public:

    constexpr void start(uint32_t now)
    {
        health_ = {};
        accept_after_ms_ = now;
        health_.monitoring = true;
        for (auto &last : heartbeat_ms_) last = now;
        for (auto &last : feedback_ms_) last = now;
    }
    constexpr void stop() { health_.monitoring = false; }
    
    constexpr void receive(const dcu_can_frame_t &f, uint32_t observed)
    {
        if (!health_.monitoring || f.flags || static_cast<int32_t>(observed - accept_after_ms_) < 0) return;
        if (f.id >= 0x700 && f.id <= 0x703) {
            const unsigned board = f.id - 0x700, bit = 1u << board;
            if (f.dlc != 4 || f.data[0] != board + 1 || f.data[2] != 0xa5 || f.data[3] != 0x5a ||
                ((health_.heartbeat_seen & bit) && f.data[1] == counters_[board])) return;
            heartbeat_ms_[board] = observed;
            counters_[board] = f.data[1];
            health_.heartbeat_seen |= bit;
            health_.offline &= ~bit;
        } else if (f.id >= 0x781) {
            for (unsigned i = 0; i < sizeof(groups) / sizeof(groups[0]); ++i) {
                if (groups[i].id != f.id || groups[i].dlc != f.dlc) continue;
                feedback_ms_[i] = observed;
                health_.feedback_seen |= 1u << i;
                health_.stale &= ~(1u << i);
                break;
            }
        } else if (f.dlc == 8) {
            if (f.id == 0x621) record_error(f.data[1] >> 4, f.id, f.data[0], 0);
            if ((f.id == 0x627 || f.id == 0x628) && f.data[0] >= 1 && f.data[0] <= 7)
                record_error(f.id == 0x627 ? 2 : 4, f.id, f.data[6], f.data[7]);
        }
    }


    constexpr const Health &sample(uint32_t now, unsigned heartbeat_timeout_ms, unsigned feedback_periods)
    {
        if (health_.monitoring) {
            // Reject queued pre-session frames; keep the time fence local across clock wrap.
            if (now - accept_after_ms_ > 60000) accept_after_ms_ = now - 60000;
            for (unsigned b = 0; b < 4; ++b)
                if (now - heartbeat_ms_[b] >= heartbeat_timeout_ms) health_.offline |= 1u << b;
            for (unsigned i = 0; i < sizeof(groups) / sizeof(groups[0]); ++i)
                if (now - feedback_ms_[i] >= groups[i].period_ms * feedback_periods) health_.stale |= 1u << i;
        }
        return health_;
    }
private:
    // Keep the board-by-board bit order of CoE 0x2104.
    struct Group { uint16_t id; uint8_t dlc, period_ms; };
    static constexpr Group groups[]{
        {0x781,8,120},
        {0x785,8,120},
        {0x791,8,120},
        {0x782,8,200},
        {0x786,8,200},
        {0x792,4,200},
        {0x7a3,8,200},
        {0x7a7,8,200},
        {0x789,8,120},
        {0x78d,8,120},
        {0x794,8,120},
        {0x78a,8,200},
        {0x78e,8,200},
        {0x795,4,200},
        {0x798,8,200},
        {0x79f,8,200}
    };


    constexpr void record_error(unsigned board, uint32_t id, uint8_t code, uint8_t stage)
    {
        if (board >= 1 && board <= 4 && (code || stage)) {
            health_.device_errors |= 1u << (board - 1);
            health_.last_error[board - 1] = (id << 16) | (static_cast<uint32_t>(stage) << 8) | code;
        }
    }
    Health health_{};
    uint32_t accept_after_ms_{};
    uint8_t counters_[4]{};
    uint32_t heartbeat_ms_[4]{}, feedback_ms_[16]{};
};

}
#endif
