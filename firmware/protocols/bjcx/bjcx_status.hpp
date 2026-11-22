#ifndef DCU_BJCX_STATUS_HPP
#define DCU_BJCX_STATUS_HPP

#include "motor_driver.hpp"

namespace dcu::bjcx {

inline constexpr uint8_t status_reply_bytes = 16;

motor::CanFrame status_request(uint16_t can_id) noexcept;
// Reported mode is the raw status nibble, not a power-stage confirmation.
// Rejected frames leave both outputs unchanged.
bool decode_status_reply(uint16_t can_id, const motor::CanFrame &, motor::Feedback &,
                         uint8_t *reported_mode = nullptr) noexcept;

}
#endif
