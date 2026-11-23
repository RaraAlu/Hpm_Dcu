#ifndef DCU_IGH_SELECTION_HPP
#define DCU_IGH_SELECTION_HPP

#include <dcu_igh_layout.hpp>
#include <chrono>
#include <thread>
#include <vector>

namespace dcu_profile_pdo {
struct SelectedImage {
    std::vector<Block> blocks;
    std::vector<uint32_t> entries;
    unsigned bytes{};
    Image image() const { return {blocks.data(), entries.data(), unsigned(blocks.size()), bytes}; }
};
struct Selection {
    uint32_t channels{}, axes{};
    SelectedImage rx, tx;

    bool build(uint32_t buses, uint32_t slots)
    {
        *this = {};
        if (!buses || (buses & ~channel_mask) || (slots & ~axis_mask)) return false;
        for (unsigned a = 0; a < 28; ++a)
            if ((slots & (1U << a)) && !(buses & (1U << (a / 7)))) return false;
        channels = buses;
        axes = slots;
        for (unsigned dir = 0; dir < 2; ++dir) {
            auto &dst = dir ? tx : rx;
            const auto &src = dir ? modes[1].tx : modes[1].rx;
            unsigned field = 0;
            for (unsigned i = 0; i < src.count; ++i) {
                Block b = src.blocks[i];
                unsigned n = b.mapping - (dir ? 0x1a00 : 0x1600);
                bool include = true;
                if (n >= 1 && n <= 4) include = buses & (1U << (n - 1));
                if (n >= 0x20 && n < 0x3c) include = slots & (1U << (n - 0x20));
                if (include) {
                    b.offset = dst.bytes;
                    dst.bytes += b.bytes;
                    dst.blocks.push_back(b);
                    dst.entries.insert(dst.entries.end(), src.entries + field, src.entries + field + b.fields);
                }
                field += b.fields;
            }
        }
        return true;
    }
};

inline bool sdo_value(ec_master_t *m, unsigned slave, uint16_t ix, unsigned sub, uint32_t &value, unsigned n)
{
    uint8_t b[4]{};
    if (n > sizeof(b) || !read_exact(m, slave, ix, sub, b, n)) return false;
    value = uint32_t(b[0]) | uint32_t(b[1]) << 8 | uint32_t(b[2]) << 16 | uint32_t(b[3]) << 24;
    return true;
}
inline void put32(uint8_t *dst, uint32_t v)
{ for (unsigned i = 0; i < 4; ++i) dst[i] = static_cast<uint8_t>(v >> (8*i)); }
inline bool write_bytes(ec_master_t *m, unsigned slave, uint16_t ix, unsigned sub, uint8_t *b, size_t n)
{
    uint32_t abort{};
    return !ecrt_master_sdo_download(m, slave, ix, sub, b, n, &abort) && !abort;
}
inline bool write_value(ec_master_t *m, unsigned slave, uint16_t ix, unsigned sub, uint32_t v, unsigned n)
{
    uint8_t bytes[4];
    put32(bytes, v);
    return n <= sizeof(bytes) && write_bytes(m, slave, ix, sub, bytes, n);
}
inline bool catalogue_matches(ec_master_t *m, unsigned slave)
{
    uint8_t hash[sizeof(digest)]{};
    uint32_t id{};
    return read_exact(m, slave, 0x2102, 1, hash, sizeof(hash)) && !std::memcmp(hash, digest, sizeof(hash)) &&
        sdo_value(m, slave, 0x2100, 4, id, 4) && id == profile_id;
}
inline bool live_image(ec_master_t *m, unsigned slave, unsigned sm, const SelectedImage &im)
{
    const unsigned ix = 0x1c10 + sm;
    uint32_t v{};
    if (!sdo_value(m, slave, ix, 0, v, 1) || v != im.blocks.size()) return false;
    unsigned field = 0;
    for (unsigned i = 0; i < im.blocks.size(); ++i) {
        const auto &b = im.blocks[i];
        if (!sdo_value(m, slave, ix, i + 1, v, 2) || v != b.mapping ||
            !sdo_value(m, slave, b.mapping, 0, v, 1) || v != b.fields) return false;
        for (unsigned j = 0; j < b.fields; ++j)
            if (!sdo_value(m, slave, b.mapping, j + 1, v, 4) || v != im.entries[field++]) return false;
    }
    return true;
}
inline bool assign_image(ec_master_t *m, unsigned slave, uint16_t ix, const SelectedImage &im)
{
    if (!write_value(m, slave, ix, 0, 0, 1)) return false;
    for (unsigned i = 0; i < im.blocks.size(); ++i)
        if (!write_value(m, slave, ix, i + 1, im.blocks[i].mapping, 2)) return false;
    return write_value(m, slave, ix, 0, im.blocks.size(), 1);
}

// Management thread only. Caller owns the master, has stopped outputs and is in PREOP.
// No implicit rollback or retry: the master's old domain cannot be reused after this call.
inline const char *apply_selection(ec_master_t *m, unsigned slave, const Selection &s, uint32_t &generation)
{
    ec_slave_info_t info{};
    uint8_t request[88]{}, boot[8]{};
    uint32_t rev{}, state{}, gen{};
    if (!s.channels || ecrt_master_get_slave(m, slave, &info) || info.al_state != 2) return "not in PREOP";
    if (!catalogue_matches(m, slave) || !read_exact(m, slave, 0x2103, 5, request, 8) ||
        !sdo_value(m, slave, 0x2100, 5, rev, 4) || !sdo_value(m, slave, 0x2103, 2, gen, 4) ||
        gen == UINT32_MAX || !sdo_value(m, slave, 0x2101, 3, state, 1)) return "catalogue or context";
    if (state == 5 && !write_value(m, slave, 0x2101, 2, 1, 1)) return "fault reset";
    put32(request + 8, rev); put32(request + 12, gen);
    put32(request + 16, s.channels); put32(request + 20, s.axes);
    std::memcpy(request + 24, digest, 32);
    std::memcpy(request + 56, profile_digest, 32);
    if (!write_bytes(m, slave, 0x2103, 1, request, sizeof(request))) return "selection rejected";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    do {
        if (!sdo_value(m, slave, 0x2101, 3, state, 1) || (state != 4 && state != 1)) return "drain fault";
        if (state == 1) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    if (state != 1 || !write_value(m, slave, 0x2101, 2, 1, 1)) return "drain/reset";
    if (!assign_image(m, slave, 0x1c12, s.rx) || !assign_image(m, slave, 0x1c13, s.tx) ||
        !live_image(m, slave, 2, s.rx) || !live_image(m, slave, 3, s.tx)) return "live PDO mapping";
    if (!write_value(m, slave, 0x2103, 6, gen + 1, 4)) return "commit rejected";
    uint32_t buses{}, axes{};
    if (!read_exact(m, slave, 0x2103, 5, boot, 8) || std::memcmp(boot, request, 8) ||
        !sdo_value(m, slave, 0x2103, 2, generation, 4) || generation != gen + 1 ||
        !sdo_value(m, slave, 0x2103, 3, buses, 4) || buses != s.channels ||
        !sdo_value(m, slave, 0x2103, 4, axes, 4) || axes != s.axes) return "commit readback";
    return nullptr;
}

// Explicitly rebuild the local IgH domain, even when the SII scan predates the selection.
inline int register_selected(ec_slave_config_t *sc, ec_domain_t *domain, unsigned sm, const SelectedImage &s)
{
    if ((sm != 2 && sm != 3) || ecrt_slave_config_sync_manager(sc, sm, sm == 2 ? EC_DIR_OUTPUT : EC_DIR_INPUT,
            sm == 2 ? EC_WD_ENABLE : EC_WD_DISABLE)) return -1;
    ecrt_slave_config_pdo_assign_clear(sc, sm);
    unsigned field = 0;
    for (const auto &b : s.blocks) {
        if (ecrt_slave_config_pdo_assign_add(sc, sm, b.mapping)) return -1;
        ecrt_slave_config_pdo_mapping_clear(sc, b.mapping);
        for (unsigned j = 0; j < b.fields; ++j) {
            const uint32_t w = s.entries[field++];
            if (ecrt_slave_config_pdo_mapping_add(sc, b.mapping, w >> 16, uint8_t(w >> 8), uint8_t(w))) return -1;
        }
    }
    return register_image(sc, domain, sm, s.image());
}
}
#endif
