#ifndef DCU_BJCX_MODELS_H
#define DCU_BJCX_MODELS_H

#include <cstdint>
#include <string_view>

namespace dcu::motor::bjcx_config {

inline constexpr uint16_t coe_encoding_id = 0x8001U; // CiH408 impedance PDO

struct Model {
    std::string_view alias{};
    std::string_view part_number{};
    uint16_t reduction_ratio{};
    double rated_speed_rpm{};
    double torque_constant_Nm_per_A{};
    double rated_torque_Nm{};
    double peak_torque_Nm{};
};

// HRA quick selection guide, p. 2, and manufacturer torque-coefficient data.
// These are output-side product specs, not CiH408 wire scales or safety limits.
inline constexpr Model models[]{
    [] {
        Model model{};
        model.alias = "40h_101";
        model.part_number = "HRA40H-101";
        model.reduction_ratio = 101;
        model.rated_speed_rpm = 54;
        model.torque_constant_Nm_per_A = 2.16;
        model.rated_torque_Nm = 2.3;
        model.peak_torque_Nm = 6.3;
        return model;
    }(),
    [] {
        Model model{};
        model.alias = "55h_100";
        model.part_number = "HRA55H-100";
        model.reduction_ratio = 100;
        model.rated_speed_rpm = 37;
        model.torque_constant_Nm_per_A = 5.97;
        model.rated_torque_Nm = 5.4;
        model.peak_torque_Nm = 35;
        return model;
    }(),
};

constexpr const Model *find_model(std::string_view key) noexcept
{
    if (key.empty()) return nullptr;
    const Model *match = nullptr;
    for (const auto &model : models) {
        if (key != model.alias && key != model.part_number) continue;
        if (match) return nullptr;
        match = &model;
    }
    return match;
}

}
#endif
