#include "bjcx_control.hpp"
#include <cmath>
#include <cstring>
#include <limits>

namespace dcu::bjcx {
namespace {
constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

bool valid_joint_id(uint16_t id)
{
    return id != 0 && id <= 0x3ff;
}

void put_be16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
}

void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    put_be16(out, static_cast<uint16_t>(bits >> 16));
    put_be16(out + 2, static_cast<uint16_t>(bits));
}
}

bool encode_enable(uint16_t can_id, bool enabled, motor::CanFrame &out) noexcept
{
    if (!valid_joint_id(can_id)) return false;
    out = {can_id, 2, fd_brs, {0x10, static_cast<uint8_t>(enabled)}};
    return true;
}

bool matches_enable_reply(uint16_t can_id, bool enabled, const motor::CanFrame &frame) noexcept
{
    return valid_joint_id(can_id) && frame.id == can_id && frame.dlc == 2 &&
           frame.flags == fd_brs && frame.data[0] == 0x90 && frame.data[1] == enabled;
}

bool encode_mit(uint16_t can_id, const MitWireCommand &command, motor::CanFrame &out) noexcept
{
    if (!valid_joint_id(can_id) || !std::isfinite(command.position_deg) || !std::isfinite(command.velocity_rpm))
        return false;

    motor::CanFrame frame{can_id, 10, fd_brs, {0x11}};
    put_be16(frame.data + 1, command.kp_deci);
    put_be16(frame.data + 3, command.kd_deci);
    put_float(frame.data + 5, command.position_deg);
    put_float(frame.data + 9, command.velocity_rpm);
    put_be16(frame.data + 13, static_cast<uint16_t>(command.feedforward_raw));
    frame.data[15] = command.counter;
    out = frame;
    return true;
}

}
