#ifndef DCU_SIDEBAND_HPP
#define DCU_SIDEBAND_HPP

#include <cstddef>
#include <cstdint>
#include <dcu_profile_pdo.hpp>

namespace dcu_profile_pdo {

inline constexpr std::size_t classic_slot_bytes = 12;

constexpr bool fits_classic_slot(uint32_t id, unsigned dlc, unsigned flags = 0)
{ return id <= 0x7ff && dlc <= 8 && flags == 0; }

constexpr bool sideband_tx_allowed(unsigned bus, uint32_t id, uint8_t dlc, uint8_t flags)
{
    if (sideband_bus < 0) return true; // Profiles without a dedicated schedule keep generic Raw.
    if (bus != static_cast<unsigned>(sideband_bus) || flags) return false;
    for (const auto &slot : sideband_slots)
        if (slot.bound && slot.frame.can_id == id && slot.frame.dlc == dlc) return true;
    for (const auto &frame : sideband_services)
        if (frame.can_id == id && frame.dlc == dlc) return true;
    return false;
}

// RK passes the mapped 0x7300 block, not the start of SM2. Phase zero starts a new pose.
constexpr bool write_sideband_slot(uint8_t *tail, std::size_t bytes, unsigned phase,
                                   const uint8_t *payload, std::size_t length)
{
    if (sideband_bus < 0 || !tail || bytes != classic_slot_bytes) return false;
    for (std::size_t i = 0; i < bytes; ++i) tail[i] = 0;
    if (phase >= sideband_slots.size()) return false;
    if (!payload) return length == 0;
    const auto &slot = sideband_slots[phase];
    if (!slot.bound || length != slot.frame.dlc || !fits_classic_slot(slot.frame.can_id, slot.frame.dlc)) return false;
    tail[0] = static_cast<uint8_t>(slot.frame.can_id);
    tail[1] = static_cast<uint8_t>(slot.frame.can_id >> 8);
    tail[2] = slot.frame.dlc;
    tail[3] = 1;
    for (std::size_t i = 0; i < length; ++i) tail[4 + i] = payload[i];
    return true;
}

}
#endif
