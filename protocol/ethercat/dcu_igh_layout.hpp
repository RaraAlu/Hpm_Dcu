#ifndef DCU_IGH_LAYOUT_HPP
#define DCU_IGH_LAYOUT_HPP

#include <cstring>
#include <ecrt.h>
#include <dcu_profile_pdo.hpp>

namespace dcu_profile_pdo {
inline bool read_exact(ec_master_t *master, uint16_t slave, uint16_t index, uint8_t sub,
                       uint8_t *dst, size_t length)
{
    size_t actual{};
    uint32_t abort{};
    return !ecrt_master_sdo_upload(master, slave, index, sub, dst, length, &actual, &abort)
        && !abort && actual == length;
}

// Check both the live assignments and IgH's scanned mapping before registering any outputs.
inline bool verify(ec_master_t *master, uint16_t slave, unsigned mode)
{
    uint8_t live[sizeof(digest)]{};
    if (mode >= sizeof(modes) / sizeof(modes[0]) ||
        !read_exact(master, slave, 0x2102, 1, live, sizeof(live)) || std::memcmp(live, digest, sizeof(live)))
        return false;
    for (unsigned sm = 2; sm <= 3; ++sm) {
        const auto &im = sm == 2 ? modes[mode].rx : modes[mode].tx;
        ec_sync_info_t sync{};
        const uint16_t assignment = 0x1c10 + sm;
        if (!read_exact(master, slave, assignment, 0, live, 1) || live[0] != im.count ||
            ecrt_master_get_sync_manager(master, slave, sm, &sync) || sync.n_pdos != im.count ||
            sync.dir != (sm == 2 ? EC_DIR_OUTPUT : EC_DIR_INPUT)) return false;
        unsigned n = 0;
        for (unsigned p = 0; p < im.count; ++p) {
            const auto &b = im.blocks[p];
            ec_pdo_info_t pdo{};
            if (!read_exact(master, slave, assignment, p + 1, live, 2) ||
                (uint32_t(live[0]) | uint32_t(live[1]) << 8) != b.mapping ||
                ecrt_master_get_pdo(master, slave, sm, p, &pdo) || pdo.index != b.mapping ||
                pdo.n_entries != b.fields) return false;
            for (unsigned j = 0; j < b.fields; ++j) {
                ec_pdo_entry_info_t e{};
                if (ecrt_master_get_pdo_entry(master, slave, sm, p, j, &e) ||
                    ((uint32_t(e.index) << 16) | (uint32_t(e.subindex) << 8) | e.bit_length) != im.entries[n++])
                    return false;
            }
        }
    }
    return true;
}

inline int register_image(ec_slave_config_t *config, ec_domain_t *domain, unsigned sm, const Image &image)
{
    int base = -1;
    for (unsigned p = 0; p < image.count; ++p) {
        unsigned bit = 8;
        const int address = ecrt_slave_config_reg_pdo_entry_pos(config, sm, p, 0, domain, &bit);
        if (!p) base = address;
        if (address < 0 || bit || address - base != image.blocks[p].offset) return -1;
    }
    return base;
}
}
#endif
