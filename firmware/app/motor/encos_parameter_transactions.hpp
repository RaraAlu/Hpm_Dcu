#ifndef DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP
#define DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP

#include <cstdint>
#include "can_frame.h"
#include "motor_driver_binding.hpp"

namespace dcu::app::parameters {

struct Status {
    bool busy, sent, internal;
    unsigned axis;
    uint32_t can_id;
};
enum class Field : uint8_t { None, Position, Velocity, Current, Power };
struct Observation {
    bool valid{};
    bool update_actual{};
    bool acknowledged{};
    Field field{};
    int32_t value{};
};

// Task-owned transactions; callers receive immutable status, never mutable state.
Status inspect(unsigned bus);
// New requests and raw traffic must also respect the device's settling hold.
bool available(unsigned bus, uint32_t now_ms);
const dcu_can_frame_t *pending(unsigned bus);
void mark_sent(unsigned bus, uint32_t now_ms);
// Queued and submitted phases each retain the existing timeout policy.
void check_timeout(unsigned bus, uint32_t now_ms);
// Request tagged CAN cancellation before finishing; do not wait for hardware.
void cancel(unsigned bus, uint16_t result);
void reset_session(uint16_t result);
// Startup may preempt polling, but must never abort a user transaction.
void abort_internal(unsigned bus);
bool start_query(unsigned bus, unsigned axis, uint8_t query, uint8_t group,
                 uint32_t now_ms, ParameterDialect dialect, uint16_t can_id, uint8_t format);
bool commit(unsigned axis, bool maintenance, uint32_t now_ms,
            bool permitted, bool channel_active);
bool consume_maintenance(unsigned bus, const dcu_can_frame_t &frame);
Observation consume(unsigned bus, unsigned axis, const dcu_can_frame_t &frame,
                    uint32_t now_ms);

}
#endif
