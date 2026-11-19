#include "encos_motor.hpp"
#include "motor_limits.hpp"
#include "detail/encos_math.hpp"
#include <limits.h>
#include <math.h>
#include <string.h>

using dcu::encos::detail::pi;
static void put16(uint8_t *p, uint16_t v) { p[0]=v>>8; p[1]=v; }
static void put32(uint8_t *p, uint32_t v) { p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
static uint32_t float_bits(float f) { uint32_t u; memcpy(&u,&f,4); return u; }
bool encos_profile_valid(const encos_profile_t *p)
{
    const double *ranges[] = {p->kp, p->kd, p->pos, p->vel, p->torque, p->current};
    for (const auto *range : ranges)
        if (!isfinite(range[0]) || !isfinite(range[1]) || !isfinite(range[1] - range[0])) return false;
    return p->kp[0]<p->kp[1] && p->kd[0]<p->kd[1] && p->pos[0]<0 && p->pos[1]>0
        && p->vel[0]<0 && p->vel[1]>0 && p->torque[0]<0 && p->torque[1]>0
        && p->current[0]<0 && p->current[1]>0 && isfinite(p->kt) && p->kt >= 0
        && p->pos[0] * 1e6 >= INT32_MIN && p->pos[1] * 1e6 <= INT32_MAX
        && p->vel[0] * 1e6 >= INT32_MIN && p->vel[1] * 1e6 <= INT32_MAX;
}
static bool quantize(double x, const double r[2], unsigned bits, uint32_t *v)
{
    if(!isfinite(x) || r[0]>=r[1] || x<r[0] || x>r[1]) return false;
    *v=(uint32_t)((x-r[0])*((1u<<bits)-1)/(r[1]-r[0])); return true;
}
static double decode(uint32_t v,const double r[2],unsigned bits) { return r[0]+v*(r[1]-r[0])/((1u<<bits)-1); }
namespace dcu::encos {
bool encode_motion(const motor::Command &c, const motor::Routing &r,
                   const encos_profile_t &p, motor::CanFrame &frame) noexcept
{
    using motor::ControlMode;
    const auto mode = static_cast<uint8_t>(c.mode);
    const double pos = c.position_urad / 1e6, vel = c.velocity_urad_s / 1e6;
    constexpr uint16_t controls = motor::Arm | motor::Stop | motor::ResetFault | motor::StartMotion;
    if (mode < 1 || mode > 5 || c.reserved_flags || (c.control_bits & ~controls) || r.can_id < 1 || r.can_id > 8)
        return false;
    if (c.current_cA < INT16_MIN || c.current_cA > INT16_MAX)
        return false;
    if (!motor::within_limits(c, r))
        return false;
    if (c.mode != ControlMode::Position && c.mode != ControlMode::Impedance && c.position_urad)
        return false;
    if (c.mode != ControlMode::Position && c.mode != ControlMode::Velocity &&
        c.mode != ControlMode::Impedance && c.velocity_urad_s)
        return false;
    if (c.mode != ControlMode::Current && c.current_cA)
        return false;
    if (c.mode != ControlMode::Impedance && c.mode != ControlMode::Torque && c.torque_mNm)
        return false;
    if (c.mode != ControlMode::Impedance && (c.kp_centi || c.kd_milli))
        return false;
    if (c.mode == ControlMode::Position || c.mode == ControlMode::Velocity) {
        if (!c.current_limit_dA)
            return false;
    } else if (c.current_limit_dA) {
        return false;
    }

    motor::CanFrame candidate{};
    candidate.id = r.can_id;
    switch (c.mode) {
    case ControlMode::Position: {
        const double rpm10 = vel * 60 / (2 * pi) * 10;
        if (rpm10 < 0 || rpm10 > 32767 || c.current_limit_dA > 4095)
            return false;
        const uint64_t bits = (UINT64_C(1) << 61) |
            (static_cast<uint64_t>(float_bits(static_cast<float>(pos * 180 / pi))) << 29) |
            (static_cast<uint64_t>(static_cast<uint32_t>(rpm10)) << 14) |
            (static_cast<uint64_t>(c.current_limit_dA) << 2) | 1;
        candidate.dlc = 8;
        for (unsigned i = 0; i < 8; ++i)
            candidate.data[i] = static_cast<uint8_t>(bits >> (56 - 8 * i));
        break;
    }
    case ControlMode::Velocity:
        candidate.dlc = 7;
        candidate.data[0] = 0x41;
        put32(candidate.data + 1, float_bits(static_cast<float>(vel * 60 / (2 * pi))));
        put16(candidate.data + 5, c.current_limit_dA);
        break;
    case ControlMode::Current:
    case ControlMode::Torque: {
        const int32_t value = c.mode == ControlMode::Current ? c.current_cA : c.torque_mNm / 10;
        if (value < INT16_MIN || value > INT16_MAX)
            return false;
        candidate.dlc = 3;
        candidate.data[0] = c.mode == ControlMode::Current ? 0x61 : 0x65;
        put16(candidate.data + 1, static_cast<uint16_t>(value));
        break;
    }
    case ControlMode::Impedance: {
        uint32_t kp, kd, x, v, t;
        if (!quantize(c.kp_centi / 100., p.kp, 12, &kp) || !quantize(c.kd_milli / 1000., p.kd, 9, &kd) ||
            !quantize(pos, p.pos, 16, &x) || !quantize(vel, p.vel, 12, &v) ||
            !quantize(c.torque_mNm / 1000., p.torque, 12, &t))
            return false;
        const uint64_t bits = static_cast<uint64_t>(kp) << 49 | static_cast<uint64_t>(kd) << 40 |
            static_cast<uint64_t>(x) << 24 | static_cast<uint64_t>(v) << 12 | t;
        candidate.dlc = 8;
        for (unsigned i = 0; i < 8; ++i)
            candidate.data[i] = static_cast<uint8_t>(bits >> (56 - 8 * i));
        break;
    }
    default:
        return false;
    }
    frame = candidate;
    return true;
}

bool decode_feedback(const motor::CanFrame &frame, const encos_profile_t &p,
                     motor::Feedback &feedback) noexcept
{
    if (frame.dlc != 8 || (frame.data[0] >> 5) != 1 || (frame.data[0] & 31) > 7)
        return false;
    const uint8_t *d = frame.data;
    const double position = decode(encos_be16(d + 1), p.pos, 16) * 1e6;
    const double velocity = decode(static_cast<uint32_t>(d[3]) << 4 | d[4] >> 4, p.vel, 12) * 1e6;
    const double current = decode(static_cast<uint32_t>(d[4] & 15) << 8 | d[5], p.current, 12) * 100;
    if (!(position >= INT32_MIN && position <= INT32_MAX) ||
        !(velocity >= INT32_MIN && velocity <= INT32_MAX) || !(current >= INT16_MIN && current <= INT16_MAX))
        return false;
    motor::Feedback candidate{};
    candidate.position_urad = static_cast<int32_t>(position);
    candidate.velocity_urad_s = static_cast<int32_t>(velocity);
    candidate.current_cA = static_cast<int16_t>(current);
    // Torque uses the already truncated current, as required by the wire conversion.
    const double torque = candidate.current_cA * p.kt * 10;
    const bool valid_torque = torque >= INT32_MIN && torque <= INT32_MAX;
    candidate.torque_mNm = valid_torque ? static_cast<int32_t>(torque) : 0;
    candidate.motor_temperature_deciC = (static_cast<int>(d[6]) - 50) * 5;
    candidate.mos_temperature_deciC = (static_cast<int>(d[7]) - 50) * 5;
    candidate.motor_fault = d[0] & 31;
    candidate.valid_fields = motor::Position | motor::Velocity | motor::Current |
        motor::MotorTemperature | motor::MosTemperature;
    if (p.kt > 0 && valid_torque)
        candidate.valid_fields |= motor::Torque;
    feedback = candidate;
    return true;
}
}
