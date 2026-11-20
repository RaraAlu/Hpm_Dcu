#include "bjcx_status.hpp"
#include <cmath>
#include <cstring>
#include <limits>

namespace dcu::bjcx {
namespace {
constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
constexpr double pi = 3.14159265358979323846;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

bool decode_scaled(const uint8_t *data, double scale, int32_t &value)
{
    const uint32_t bits = (uint32_t{data[0]} << 24) | (uint32_t{data[1]} << 16) |
                          (uint32_t{data[2]} << 8) | data[3];
    float wire;
    std::memcpy(&wire, &bits, sizeof(wire));
    const double converted = double(wire) * scale;
    if (!std::isfinite(converted) || converted < INT32_MIN || converted > INT32_MAX) return false;
    value = static_cast<int32_t>(::round(converted));
    return true;
}
}

motor::CanFrame status_request(uint16_t can_id) noexcept
{
    motor::CanFrame frame{};
    frame.id = can_id;
    frame.flags = fd_brs;
    frame.dlc = 1;
    frame.data[0] = 0x17;
    return frame;
}

bool decode_status_reply(uint16_t can_id, const motor::CanFrame &frame, motor::Feedback &out) noexcept
{
    if (frame.id != can_id || frame.flags != fd_brs ||
        frame.dlc != 10 || frame.data[0] != 0x80)
        return false;
    motor::Feedback sample{};
    const auto *data = frame.data;
    // Invalid position/velocity must not hide an independent motor alarm.
    sample.motor_fault = (uint32_t{data[1] & 0x0fU} << 8) | data[2];
    if (data[1] >> 4 <= 5) {
        if (decode_scaled(data + 3, pi * 1e6 / 180, sample.position_urad)) sample.valid_fields |= motor::Position;
        if (decode_scaled(data + 7, pi * 1e6 / 30, sample.velocity_urad_s)) sample.valid_fields |= motor::Velocity;
    }
    const int32_t current = (int32_t{data[11]} << 8) | data[12];
    sample.current_cA = current < 32768 ? current : current - 65536;
    sample.motor_temperature_deciC = (int(data[13]) - 50) * 10;
    sample.mos_temperature_deciC = (int(data[14]) - 50) * 10;
    sample.valid_fields |= motor::Current | motor::MotorTemperature | motor::MosTemperature;
    out = sample;
    return true;
}

}
