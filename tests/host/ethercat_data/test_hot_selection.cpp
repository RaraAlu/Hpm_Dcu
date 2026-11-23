#include <dcu_igh_selection.hpp>
#include <cassert>
#include <map>

using namespace dcu_profile_pdo;
static std::map<std::pair<unsigned,unsigned>, std::vector<uint8_t>> dictionary;
static bool corrupt_entry, committed, in_preop = true;
static unsigned writes;
static void scalar(unsigned ix, unsigned sub, uint32_t v, unsigned n)
{
    uint8_t b[4]; put32(b, v);
    dictionary[{ix,sub}] = {b,b+n};
}
int ecrt_master_get_slave(ec_master_t *, uint16_t, ec_slave_info_t *s) { s->al_state = in_preop ? 2 : 8; return 0; }
int ecrt_master_sdo_upload(ec_master_t *, uint16_t, uint16_t ix, uint8_t sub, uint8_t *p,
                           size_t len, size_t *actual, uint32_t *abort)
{
    const auto &v = dictionary[{ix,sub}];
    if (v.size() != len) return -1;
    *actual = len; *abort = 0; std::memcpy(p,v.data(),len);
    if (corrupt_entry && ix == 0x1a2e && sub == 1) p[1] ^= 1;
    return 0;
}
int ecrt_master_sdo_download(ec_master_t *, uint16_t, uint16_t ix, uint8_t sub, uint8_t *p,
                             size_t n, uint32_t *abort)
{
    ++writes; *abort = 0;
    if (ix == 0x2103 && sub == 1) {
        assert(n == 88 && !std::memcmp(p+24,digest,32) && !std::memcmp(p+56,profile_digest,32));
        dictionary[{ix,3}] = {p+16,p+20};
        dictionary[{ix,4}] = {p+20,p+24};
        scalar(0x2101,3,1,1);
    } else if (ix == 0x2103 && sub == 6) {
        assert(n == 4 && p[0] == 1);
        committed = true;
        scalar(ix,2,1,4);
    } else dictionary[{ix,sub}] = {p,p+n};
    return 0;
}
int main()
{
    Selection s;
    assert(!s.build(12,axis_mask | 1) && !s.build(4,axis_mask) && s.build(12,axis_mask));
    assert(s.rx.bytes == 216 && s.tx.bytes == 256);
    assert(s.build(12,axis_mask ^ (1U << 15)) && s.rx.bytes == 192 && s.tx.bytes == 224);
    assert(offset(s.rx.image(),0x720f) == -1 && offset(s.rx.image(),0x7210) == 72);
    dictionary[{0x2102,1}] = {digest,digest+32};
    dictionary[{0x2103,5}] = std::vector<uint8_t>(8,1);
    scalar(0x2100,4,profile_id,4); scalar(0x2100,5,profile_revision,4);
    scalar(0x2101,3,0,1); scalar(0x2103,2,0,4);
    for (const auto *im : {&s.rx,&s.tx}) {
        unsigned f = 0;
        for (const auto &b : im->blocks) {
            scalar(b.mapping,0,b.fields,1);
            for (unsigned j = 0; j < b.fields; ++j) scalar(b.mapping,j+1,im->entries[f++],4);
        }
    }
    uint32_t gen{};
    in_preop = false;
    assert(apply_selection(nullptr,0,s,gen) && !writes);
    in_preop = true; corrupt_entry = true;
    assert(apply_selection(nullptr,0,s,gen) && !committed);
    corrupt_entry = false;
    assert(!apply_selection(nullptr,0,s,gen) && committed && gen == 1);
}
