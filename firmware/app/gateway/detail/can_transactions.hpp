#ifndef DCU_GATEWAY_CAN_TRANSACTIONS_HPP
#define DCU_GATEWAY_CAN_TRANSACTIONS_HPP

#include "can_bus_scheduler.hpp"
#include "gateway_application.hpp"
#include "motor_instance.hpp"

namespace dcu::app::gateway_application::detail {

inline constexpr uint32_t legacy_broadcast_id = 0x7ff;

// One query lane per physical bus. Drivers still build and interpret their frames.
class CanTransactions {
public:
    bool busy(unsigned bus) const;
    bool any_busy() const;
    bool legacy_busy(unsigned bus) const;
    bool awaiting_tx(unsigned bus) const;
    bool raw_send_allowed(unsigned bus, uint32_t now_ms) const;
    uint8_t reply_format(unsigned bus, uint32_t id, uint8_t fallback) const;
    bool consume_broadcast_reply(unsigned bus, const dcu_can_frame_t &, uint8_t bytes);
    bool consume_reply(unsigned bus, unsigned device, motor::MotorInstance &,
                       const dcu_can_frame_t &, uint8_t bytes, uint32_t stamp_ms, uint32_t now_ms);
    bool finish_tx(unsigned bus, const dcu_can_tx_event_t &);
    void expire_poll(unsigned bus, uint32_t now_ms);
    void expire_legacy(unsigned bus, uint32_t now_ms);
    void cancel(unsigned bus, uint16_t result);
    void reset_session(uint16_t result);
    bool start_query(unsigned bus, unsigned device, motor::MotorInstance &, BusScheduler &,
                     ControlSet, bool pdo_owner, bool fd, uint32_t now_ms, uint32_t now_us);
    void submit_queued(unsigned bus, BusScheduler &, ControlSet, bool fd, uint32_t now_ms, uint32_t now_us);
    ParameterReadResult read_parameter(unsigned device, const motor::MotorInstance &, const ParameterRead &,
                                       uint8_t format, uint32_t now_ms, bool permitted, bool channel_active);

private:
    void abort_poll(unsigned bus);
    bool submit_poll(unsigned bus, unsigned device, motor::MotorInstance &, BusScheduler &, ControlSet,
                     bool fd, uint32_t now_ms, uint32_t now_us);
    struct PendingPoll {
        dcu_can_tx_id_t tx_id{};
        uint32_t submitted_ms{};
        unsigned device{};
        uint16_t timeout_ms{};
        bool transmitted{};
    } polls_[DCU_DEVICE_CAN_BUS_COUNT]{};
};

}
#endif
