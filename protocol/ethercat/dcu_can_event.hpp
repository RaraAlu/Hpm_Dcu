#pragma once
#include <cstdint>
namespace dcu_can_event {
inline constexpr unsigned header_bytes = 24, data_bytes = 64, bus_count = 4;
inline constexpr uint8_t extended = 2, fd = 8, brs = 16;
constexpr unsigned bytes(unsigned mask) { return header_bytes + data_bytes * __builtin_popcount(mask); }
constexpr unsigned offset(unsigned mask, unsigned bus) { return bytes(mask & ((1U << bus) - 1U)); }
inline uint32_t read32(const uint8_t *p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline void write32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i != 4; ++i) p[i] = uint8_t(v >> (8U * i));
}
struct Route { uint32_t id; uint8_t flags, dlc; };
enum class Entry { Empty, Frame, Invalid };
inline Entry decode(const uint8_t *header, unsigned bus, Route &out)
{
    const auto *p = header + 4 + 5 * bus;
    const uint32_t word = read32(p);
    if (!p[4] && !word) return Entry::Empty;
    if ((p[4] & 0xf0) != 0x80) return Entry::Invalid;
    out = {word & 0x1fffffffU, uint8_t(((word >> 31) ? extended : 0) |
        ((word & (1U << 30)) ? fd : 0) | ((word & (1U << 29)) ? brs : 0)), uint8_t(p[4] & 15)};
    if (!(out.flags & extended) && out.id > 0x7ff) return Entry::Invalid;
    return !(out.flags & fd) && (out.dlc > 8 || (out.flags & brs)) ? Entry::Invalid : Entry::Frame;
}
inline void encode(uint8_t *header, unsigned bus, Route route)
{
    auto *p = header + 4 + 5 * bus;
    write32(p, route.id | ((route.flags & extended) ? 1U << 31 : 0) |
        ((route.flags & fd) ? 1U << 30 : 0) | ((route.flags & brs) ? 1U << 29 : 0));
    p[4] = 0x80 | route.dlc;
}
static_assert(bytes(1) == 88 && bytes(5) == 152 && bytes(7) == 216 && bytes(15) == 280);
}
