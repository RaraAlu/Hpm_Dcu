#ifndef DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP
#define DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP

#include "can_frame.h"
#include "motor_driver_binding.hpp"
#include <cstdint>

namespace dcu::app::parameters {

struct Status {
    bool busy, sent, internal;
    unsigned axis;
    uint32_t can_id;
};


enum class Field : uint8_t { None,
                             Position,
                             Velocity,
                             Current,
                             Power };

struct Observation {
    bool valid{};
    bool update_actual{};
    bool acknowledged{};
    Field field{};
    int32_t value{};
};


Status inspect(unsigned bus);


bool available(unsigned bus, uint32_t now_ms);


const dcu_can_frame_t *pending(unsigned bus);


void mark_sent(unsigned bus, uint32_t now_ms);


void check_timeout(unsigned bus, uint32_t now_ms);


void cancel(unsigned bus, uint16_t result);


void reset_session(uint16_t result);


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
