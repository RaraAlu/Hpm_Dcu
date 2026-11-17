#ifndef DCU_MOTOR_PDO_STATE_HPP
#define DCU_MOTOR_PDO_STATE_HPP

#include <cstdint>
#include "motor_pdo_layout.hpp"
#include "motor_application.hpp"

namespace dcu::ethercat::motor_pdo::detail {

struct PublicationState {
    uint8_t image[pdo_layout::max_input_bytes];
    uint32_t raw_batch;
    uint16_t input_size;
    uint8_t round_robin, diagnostic_page;
    app::motor_application::Publication snapshot;
};

}
#endif
