#pragma once

#include "can_transport.h"
#include <algorithm>
#include <array>
#include <cstdint>

namespace bjcx_mit_once {

using RxPdo = std::array<uint8_t, 32>;
using TxPdo = std::array<uint8_t, 48>;
using Status = std::array<uint8_t, 16>;
constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
constexpr uint8_t dlc16 = 10;
constexpr uint32_t request_tag = 0x3054494d; // MIT0
constexpr uint32_t feedback_tag = 0x4b434142; // BACK

inline void put_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(value >> (8 * i));
}

inline uint32_t get_u32(const uint8_t *p)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i) value |= uint32_t{p[i]} << (8 * i);
    return value;
}

inline RxPdo request(uint32_t seq)
{
    RxPdo pdo{};
    put_u32(pdo.data(), request_tag);
    put_u32(pdo.data() + 4, seq);
    return pdo;
}

struct Feedback {
    Status bytes{};
    uint8_t mode() const { return bytes[1] >> 4; }
    uint16_t alarm() const { return ((bytes[1] & 0xfU) << 8) | bytes[2]; }
    bool mit_active() const { return mode() == 2 && alarm() == 0; }
};

inline bool collect(const TxPdo &pdo, uint32_t seq, unsigned wkc, bool op, Feedback &out)
{
    if (!seq || !op || wkc != 3 || get_u32(pdo.data()) != feedback_tag ||
        get_u32(pdo.data() + 4) != seq || pdo[8] != 0x80)
        return false;
    std::copy_n(pdo.begin() + 8, out.bytes.size(), out.bytes.begin());
    return true;
}

class Exchange {
public:
    Exchange(unsigned bus, uint16_t id, uint8_t counter) : bus_(bus), id_(id), counter_(counter) {}

    // Only an explicitly armed, quiet diagnostic session may dispatch. There is no rearm or retry.
    bool dispatch(const RxPdo &pdo, bool armed, bool op, uint64_t now, uint64_t sm3_deadline,
                  dcu_can_frame_t &frame)
    {
        const auto seq = get_u32(pdo.data() + 4);
        if (stage_ != Stage::idle || !armed || !op || !seq || pdo != request(seq) ||
            bus_ >= DCU_CAN_BUS_COUNT || !id_ || id_ > 0x7ff || now >= sm3_deadline)
            return false;
        seq_ = seq;
        dispatch_us_ = now;
        deadline_us_ = sm3_deadline;
        stage_ = Stage::dispatched;
        frame = {};
        frame.id = id_;
        frame.dlc = dlc16;
        frame.flags = fd_brs;
        frame.data[0] = 0x11;
        frame.data[15] = counter_;
        return true;
    }

    void submitted(dcu_can_tx_id_t tx_id)
    {
        if (stage_ != Stage::dispatched) return;
        tx_id_ = tx_id;
        stage_ = tx_id ? Stage::await_tx : Stage::failed;
    }

    void tx_event(unsigned bus, dcu_can_tx_event_t event, uint64_t eof_us)
    {
        if (stage_ != Stage::await_tx || bus != bus_ || event.id != tx_id_) return;
        if (event.status != DCU_CAN_TX_COMPLETED || eof_us < dispatch_us_ || eof_us >= deadline_us_) {
            stop();
            return;
        }
        tx_eof_us_ = eof_us;
        stage_ = Stage::await_rx;
    }

    bool rx(unsigned bus, const dcu_can_frame_t &frame, uint64_t eof_us)
    {
        // The default 0x80 reply does not echo the request counter.
        if (stage_ != Stage::await_rx || bus != bus_ || eof_us <= tx_eof_us_ ||
            eof_us >= deadline_us_ || frame.id != id_ || frame.flags != fd_brs ||
            frame.dlc != dlc16 || frame.data[0] != 0x80)
            return false;
        put_u32(pdo_.data(), feedback_tag);
        put_u32(pdo_.data() + 4, seq_);
        std::copy_n(frame.data, Status{}.size(), pdo_.begin() + 8);
        rx_eof_us_ = eof_us;
        stage_ = Stage::ready;
        return true;
    }

    void poll(bool op, uint64_t now)
    {
        if (!op || (stage_ != Stage::idle && stage_ != Stage::published && now >= deadline_us_)) stop();
    }

    // Called with the observed SM3 write-completion time in the host replay.
    bool publish(bool op, uint64_t written_us, TxPdo &image)
    {
        poll(op, written_us);
        if (stage_ != Stage::ready || written_us < rx_eof_us_) return false;
        image = pdo_;
        stage_ = Stage::published;
        return true;
    }

    void stop() { stage_ = Stage::failed; }

private:
    enum class Stage { idle, dispatched, await_tx, await_rx, ready, published, failed };
    const unsigned bus_;
    const uint16_t id_;
    const uint8_t counter_;
    Stage stage_{Stage::idle};
    uint32_t seq_{};
    dcu_can_tx_id_t tx_id_{};
    uint64_t dispatch_us_{}, deadline_us_{}, tx_eof_us_{}, rx_eof_us_{};
    TxPdo pdo_{};
};

} // namespace bjcx_mit_once
