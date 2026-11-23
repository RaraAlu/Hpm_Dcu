#include "dcu_igh_layout.hpp"
#include <cassert>
#include <initializer_list>

using namespace dcu_profile_pdo;
enum Fault { None, Identity, ShortRead, Assignment, Direction, Entry, Offset, Bit };
static Fault fault;
static const Image &image(unsigned sm) { return sm == 2 ? modes[1].rx : modes[1].tx; }

int ecrt_master_sdo_upload(ec_master_t *, uint16_t, uint16_t index, uint8_t sub, uint8_t *dst,
                           size_t length, size_t *actual, uint32_t *abort)
{
    *actual = length - (fault == ShortRead);
    *abort = 0;
    if (index == 0x2102) {
        assert(sub == 1 && length == sizeof(digest));
        std::memcpy(dst, digest, length);
        if (fault == Identity) dst[0] ^= 1;
    } else {
        const auto &im = image(index - 0x1c10);
        if (sub) {
            const auto mapping = im.blocks[sub - 1].mapping;
            dst[0] = mapping ^ (fault == Assignment);
            dst[1] = mapping >> 8;
        } else dst[0] = im.count;
    }
    return 0;
}
int ecrt_master_get_sync_manager(ec_master_t *, uint16_t, uint8_t sm, ec_sync_info_t *s)
{
    s->n_pdos = image(sm).count;
    s->dir = (sm == 2) != (fault == Direction) ? EC_DIR_OUTPUT : EC_DIR_INPUT;
    return 0;
}
int ecrt_master_get_pdo(ec_master_t *, uint16_t, uint8_t sm, uint16_t p, ec_pdo_info_t *info)
{
    const auto &b = image(sm).blocks[p];
    *info = {b.mapping, b.fields};
    return 0;
}
int ecrt_master_get_pdo_entry(ec_master_t *, uint16_t, uint8_t sm, uint16_t p, uint16_t j, ec_pdo_entry_info_t *info)
{
    const auto &im = image(sm);
    unsigned n = j;
    for (unsigned i = 0; i < p; ++i) n += im.blocks[i].fields;
    const uint32_t w = im.entries[n] ^ (fault == Entry ? 0x100U : 0U);
    *info = {uint16_t(w >> 16), uint8_t(w >> 8), uint8_t(w)};
    return 0;
}
int ecrt_slave_config_reg_pdo_entry_pos(ec_slave_config_t *, uint8_t sm, unsigned p, unsigned,
                                        ec_domain_t *, unsigned *bit)
{
    *bit = fault == Bit;
    return 23 + image(sm).blocks[p].offset + (fault == Offset && p == 1);
}
int main()
{
    assert(verify(nullptr, 0, 1) && !verify(nullptr, 0, 4));
    for (auto f : {Identity, ShortRead, Assignment, Direction, Entry}) {
        fault = f;
        assert(!verify(nullptr, 0, 1));
    }
    for (unsigned sm : {2, 3}) {
        fault = None;
        assert(register_image(nullptr, nullptr, sm, image(sm)) == 23);
        fault = Offset;
        assert(register_image(nullptr, nullptr, sm, image(sm)) < 0);
        fault = Bit;
        assert(register_image(nullptr, nullptr, sm, image(sm)) < 0);
    }
}
