#ifndef DCU_MOTOR_APPLICATION_IMPL_HPP
#define DCU_MOTOR_APPLICATION_IMPL_HPP

#include "axis_controller.hpp"
#include "can_bus_scheduler.hpp"
#include "motor_application.hpp"
#include "raw_can_gateway.hpp"
#include <cstdint>

namespace dcu::app::motor_application::detail {

// MainLoop and SDO callbacks share one task; only the C/C++ entry points own this object.
class MotorApplication {
public:
    void initialize(const Configuration &defaults);
    ProcessResult process(const CycleCommand *candidate);
    void operational(bool operational);
    void cycle_fault();
    bool configuration_writable() const;
    bool owner_change_allowed() const;
    bool apply_configuration(const Configuration &configuration);
    bool select_mode(OperatingMode mode);
    OperatingMode mode() const { return state_.mode; }
    bool set_owner(Owner owner);
    Owner owner() const { return state_.owner; }
    CommandResult commit_sdo_command(unsigned axis, const SdoCommand &command, uint32_t now_ms);
    parameters::CommitOutcome commit_parameter(const parameters::Request &request, uint32_t now_ms, bool preop);
    bool read_axis(unsigned axis, AxisFeedback &feedback, AxisDiagnostics &diagnostics) const;
    void prepare_publication(Publication &snapshot);
    bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

private:
    unsigned frame_us(unsigned bus, unsigned bytes) const;
    ControlSet controls_on_bus(unsigned bus) const;
    bool any_active() const;
    bool channel_active(unsigned bus) const;
    bool command_permitted(uint32_t now_ms) const;
    void stop_all(uint16_t fault);
    void check_timeouts(uint32_t now_ms);
    bool accept_candidate(const CycleCommand &candidate, uint32_t now_ms, ProcessResult &result);
    bool process_raw(const CycleCommand &candidate, uint32_t now_ms);
    void consume_feedback(unsigned bus, const dcu_can_frame_t &frame,
                          uint32_t timestamp_ms, uint32_t now_ms);

    AxisController axes_[ENCOS_AXIS_COUNT]{};
    RawCanGateway raw_{};
    BusScheduler schedule_[ENCOS_CHANNEL_COUNT]{};
    struct Session {
        ChannelConfiguration channels[ENCOS_CHANNEL_COUNT]{};
        BoardConfiguration config{};
        BoardCommand board_command{};
        ChannelCommand channel_command[ENCOS_CHANNEL_COUNT]{};
        OperatingMode mode{};
        Owner owner{};
        bool operational{}, session_armed{}, resume_pdo_owner{};
        bool seen_cycle{}, config_ready{};
        uint32_t last_command_ms{}, last_cycle{}, tx_total{}, last_process_us{};
        uint8_t query_slot[ENCOS_CHANNEL_COUNT]{};
        uint16_t board_fault{};
        uint32_t channel_tx_drop[ENCOS_CHANNEL_COUNT]{};
        uint32_t publication_sequence{}, sdo_accepted_sequence[ENCOS_AXIS_COUNT]{};
    } state_{};
};

}
#endif
