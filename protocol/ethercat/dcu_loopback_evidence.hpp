#ifndef DCU_LOOPBACK_EVIDENCE_HPP
#define DCU_LOOPBACK_EVIDENCE_HPP

#include <cstdint>
#include <cstring>

namespace dcu::loopback_evidence {
// T1R2 bytes 5..32 retain the alternating per-slot counters. The trailer
// describes the command identified by AcceptedCycleSeq in the same SM3 image.
constexpr unsigned offset = 33, size = 15;
struct Masks { uint32_t requested{}, tx_complete{}, rx_matched{}; };

inline void store(uint8_t *sideband, Masks masks)
{
    auto *p = sideband + offset;
    if (!masks.requested) { std::memset(p, 0, size); return; }
    std::memcpy(p, "E01", 3);
    const uint32_t words[]{masks.requested, masks.tx_complete, masks.rx_matched};
    std::memcpy(p + 3, words, sizeof(words)); // HPM and RK PDOs are little-endian.
}

inline Masks read(const uint8_t *sideband)
{
    if (std::memcmp(sideband, "T1R2", 4) || std::memcmp(sideband + offset, "E01", 3)) return {};
    uint32_t words[3];
    std::memcpy(words, sideband + offset + 3, sizeof(words));
    return {words[0], words[1], words[2]};
}
}
#endif
