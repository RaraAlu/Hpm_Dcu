#include <dcu_igh_selection.hpp>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <memory>

namespace wire = dcu_profile_pdo;

int main(int argc, char **argv)
{
    if (argc == 2 && !std::strcmp(argv[1], "--catalogue-digest")) {
        for (auto byte : wire::profile_digest) std::printf("%02x", byte);
        std::puts("");
        return 0;
    }
    const bool apply = argc >= 3 && std::strcmp(argv[1], "--apply") == 0;
    const bool check = argc >= 3 && std::strcmp(argv[1], "--check") == 0;
    if (!apply && !check && !(argc == 2 && !std::strcmp(argv[1], "--inspect"))) {
        std::fprintf(stderr, "usage: %s --inspect | --catalogue-digest | --{apply,check} all|CHANNEL_MASK [PROFILE_UNIT ...]\n", argv[0]);
        return 2;
    }
    uint32_t buses{}, mask{};
    if (apply || check) {
        if (!std::strcmp(argv[2], "all") && argc == 3) {
            buses = wire::channel_mask; mask = wire::axis_mask;
        } else {
            char *end{};
            errno = 0;
            const auto parsed = std::strtoul(argv[2], &end, 0);
            if (errno || end == argv[2] || *end || parsed > 15) return 2;
            buses = parsed;
            for (int i = 3; i < argc; ++i) {
                uint32_t bit = 0;
                for (const auto &u : wire::units)
                    if (u.name && !std::strcmp(u.name, argv[i])) bit = 1U << u.slot;
                if (!bit || (mask & bit)) { std::fprintf(stderr, "Unknown/duplicate unit: %s\n", argv[i]); return 2; }
                mask |= bit;
            }
        }
    }
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> owner(ecrt_request_master(0), ecrt_release_master);
    auto *m = owner.get();
    ec_slave_info_t info{};
    if (!m || ecrt_master_get_slave(m, 0, &info) || info.al_state != 2 ||
        info.vendor_id != 0x10000 || info.product_code != 2 || !wire::catalogue_matches(m, 0)) {
        std::fprintf(stderr, "Expected idle HPM in PREOP with the selected catalogue\n"); return 1;
    }
    uint32_t gen{}, current_buses{}, current_axes{};
    uint8_t boot[8];
    if (!wire::sdo_value(m, 0, 0x2103, 2, gen, 4) || !wire::sdo_value(m, 0, 0x2103, 3, current_buses, 4) ||
        !wire::sdo_value(m, 0, 0x2103, 4, current_axes, 4)) return 1;
    if (wire::read_exact(m, 0, 0x2103, 5, boot, sizeof(boot))) {
        std::printf("HPM boot=0x");
        for (unsigned i = 8; i; --i) std::printf("%02x", boot[i-1]);
    } else {
        std::printf("HPM boot=unavailable");
    }
    std::printf(" generation=%u buses=0x%x axes=0x%x AL=PREOP\n", gen, current_buses, current_axes);
    if (check && (buses != current_buses || mask != current_axes)) {
        std::fprintf(stderr, "Live selection does not match the requested Profile\n"); return 1;
    }
    wire::Selection s;
    if (!s.build(apply ? buses : current_buses, apply ? mask : current_axes)) return 2;
    if (apply) {
        if (auto error = wire::apply_selection(m, 0, s, gen)) {
            std::fprintf(stderr, "PREOP transaction stopped at %s; no cyclic outputs started\n", error);
            return 1;
        }
    }
    if (!wire::live_image(m, 0, 2, s.rx) || !wire::live_image(m, 0, 3, s.tx)) return 1;
    auto *sc = ecrt_master_slave_config(m, 0, 0, info.vendor_id, info.product_code);
    auto *domain = ecrt_master_create_domain(m);
    if (!sc || !domain || wire::register_selected(sc, domain, 2, s.rx) != 0 ||
        wire::register_selected(sc, domain, 3, s.tx) != static_cast<int>(s.rx.bytes) ||
        ecrt_domain_size(domain) != s.rx.bytes + s.tx.bytes) {
        std::fprintf(stderr, "IgH domain offsets/size do not match live assignments\n"); return 1;
    }
    if (ecrt_master_get_slave(m, 0, &info) || info.al_state != 2) return 1;
    std::printf("%s generation=%u buses=0x%x axes=0x%x SM2=%uB SM3=%uB domain=%zuB AL=PREOP; no OP/PDO outputs\n",
                apply ? "applied" : "verified", gen, s.channels, s.axes, s.rx.bytes, s.tx.bytes, ecrt_domain_size(domain));
    for (const auto &u : wire::units)
        if (u.name && (s.axes & (1U << u.slot))) std::printf("slot %u: %s\n", u.slot, u.name);
    return 0;
}
