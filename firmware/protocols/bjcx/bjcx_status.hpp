#ifndef DCU_BJCX_STATUS_HPP
#define DCU_BJCX_STATUS_HPP

#include "motor_driver.hpp"

namespace dcu::bjcx {

inline constexpr uint8_t status_reply_bytes = 16;

// Use the configured CAN ID. Rejected frames leave feedback unchanged.
motor::CanFrame status_request(uint16_t can_id) noexcept;
bool decode_status_reply(uint16_t can_id, const motor::CanFrame &, motor::Feedback &) noexcept;

}
#endif
