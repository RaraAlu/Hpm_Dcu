#ifndef DCU_ENCOS_MODELS_H
#define DCU_ENCOS_MODELS_H

#include <cstdint>
#include <string_view>

namespace dcu::motor::encos_config {

// CoE compatibility ID for the encoding, not a physical model ID.
inline constexpr uint16_t coe_encoding_id = 0x8000U;

struct Datasheet {
    double torque_constant_Nm_per_A{};
    double peak_torque_Nm{};
    double peak_phase_current_A{};
    double peak_speed_rpm{};
};

struct MitEncoding {
    double max_speed_rad_s{};
    double max_torque_Nm{};
    double max_current_A{};
    double torque_constant_Nm_per_A{};

    constexpr bool verified() const noexcept
    {
        return max_speed_rad_s > 0 && max_torque_Nm > 0 && max_current_A > 0 && torque_constant_Nm_per_A > 0;
    }
};

struct Model {
    std::string_view alias{};
    std::string_view part_number{};
    Datasheet datasheet{};
    MitEncoding mit{}; // Leave empty until the wire ranges have been verified.
};

// ENCOS datasheets: 4315 V3.16EAP, 7216/7220 Gen 2 V1.6EAP, 9025 V1.4EAP.
inline constexpr Model models[]{
    [] {
        Model model{};
        model.alias = "4315";
        model.part_number = "EC-A4315-P2-36";
        model.datasheet.torque_constant_Nm_per_A = 2.80;
        model.datasheet.peak_torque_Nm = 75;
        model.datasheet.peak_phase_current_A = 30;
        model.datasheet.peak_speed_rpm = 117;
        return model;
    }(),
    [] {
        Model model{};
        model.alias = "7216";
        model.part_number = "EC-A7216-P2-22H";
        model.datasheet.torque_constant_Nm_per_A = 1.90;
        model.datasheet.peak_torque_Nm = 140;
        model.datasheet.peak_phase_current_A = 100;
        model.datasheet.peak_speed_rpm = 260;
        // Read back over CAN FD+BRS on can2; these are not motion limits.
        model.mit.max_speed_rad_s = 30;
        model.mit.max_torque_Nm = 140;
        model.mit.max_current_A = 100;
        model.mit.torque_constant_Nm_per_A = 1.90;
        return model;
    }(),
    [] {
        Model model{};
        model.alias = "7220";
        model.part_number = "EC-A7220-P2-22H";
        model.datasheet.torque_constant_Nm_per_A = 1.90;
        model.datasheet.peak_torque_Nm = 160;
        model.datasheet.peak_phase_current_A = 100;
        model.datasheet.peak_speed_rpm = 264;
        return model;
    }(),
    [] {
        Model model{};
        model.alias = "9025";
        model.part_number = "EC-A9025-P2-23.625H";
        // V1.4EAP marks Kt, peak torque and peak speed as estimates.
        model.datasheet.torque_constant_Nm_per_A = 2.00;
        model.datasheet.peak_torque_Nm = 320;
        model.datasheet.peak_phase_current_A = 160;
        model.datasheet.peak_speed_rpm = 235;
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
