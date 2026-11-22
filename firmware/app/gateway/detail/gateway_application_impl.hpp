#ifndef DCU_GATEWAY_APPLICATION_IMPL_HPP
#define DCU_GATEWAY_APPLICATION_IMPL_HPP

#include "motor_instance.hpp"
#include "can/can_bus_scheduler.hpp"
#include "gateway_application.hpp"
#include "gateway_configuration.hpp"
#include "can/detail/can_transactions.hpp"
#include "startup/detail/production_startup.hpp"
#include "can/raw_can_gateway.hpp"
#include <cstdint>

namespace dcu::app::gateway_application::detail {

// MainLoop and SDO callbacks share one task; only the C/C++ entry points own this object.
class GatewayApplication {
public:
    bool initialize(const Configuration &profile);
    ChannelConfiguration channel_configuration(unsigned bus) const {
        return bus < DCU_DEVICE_CAN_BUS_COUNT ? state_.channels[bus] : ChannelConfiguration{};
    }
    bool boot_startup(const dcu_gateway_profile_t &, uint64_t boot_id);
    bool startup_enabled() const { return startup_.enforced(); }
    dcu_gateway_readiness_t readiness() const { return startup_.status(); }
    profile_update::Policy profile_policy() const;
    bool apply_profile_policy(uint32_t profile, uint32_t base, uint32_t revision, const profile_update::Policy &);
    ProcessResult process(const CycleCommand *candidate);
    void operational(bool operational);
    void cycle_fault();
    bool preop_access_allowed() const;
    bool owner_change_allowed() const;
    bool select_mode(OperatingMode mode);
    OperatingMode mode() const { return state_.mode; }
    bool set_owner(Owner owner);
    Owner owner() const { return state_.owner; }
    CommandResult commit_sdo_command(unsigned device, const SdoCommand &command, uint32_t now_ms);
    bool parameter_service_busy(unsigned bus) const { return transactions_.legacy_busy(bus); }
    ParameterReadResult read_parameter(unsigned device, const ParameterRead &, uint32_t now_ms, bool preop);
    bool read_device(unsigned device, motor::MotorFeedback &feedback, motor::MotorDiagnostics &diagnostics) const;
    void prepare_publication(Publication &snapshot);
    void set_neutral_window(uint32_t period_ns, uint32_t cutoff_us);
    void neutral_feedback_deadline() { stop_neutral_run(true); }
    bool neutral_feedback_pending() const;
    bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

private:
    unsigned frame_us(unsigned bus, unsigned bytes) const;
    ControlSet controls_on_bus(unsigned bus) const;
    bool any_active() const;
    bool channel_active(unsigned bus) const;
    bool command_permitted(uint32_t now_ms) const;
    void stop_all(uint16_t fault);
    void stop_neutral_run(bool fault);
    bool neutral_window_open(uint32_t now_us) const;
    void service_neutral_run(unsigned bus, uint32_t now_ms, bool drained);
    void check_timeouts(uint32_t now_ms);
    void process_bus(unsigned bus, uint32_t now_ms, uint32_t now_us);
    void step_startup(uint32_t now_ms);
    void revoke_business();
    bool accept_candidate(const CycleCommand &candidate, uint32_t now_ms, ProcessResult &result);
    bool process_raw(const CycleCommand &candidate, uint32_t now_ms);
    void collect_tx_events(unsigned bus);
    bool consume_feedback(unsigned bus, const dcu_can_frame_t &frame,
                          const dcu_can_rx_stamp_t &stamp, uint32_t now_ms, uint32_t now_us);

    motor::MotorInstance devices_[DCU_MOTOR_DEVICE_CAPACITY]{};
    unsigned device_count_{};
    uint8_t devices_on_bus_[DCU_DEVICE_CAN_BUS_COUNT][DCU_MOTOR_DEVICES_PER_BUS]{};
    uint8_t bus_device_count_[DCU_DEVICE_CAN_BUS_COUNT]{};
    RawCanGateway raw_{};
    BusScheduler schedule_[DCU_DEVICE_CAN_BUS_COUNT]{};
    CanTransactions transactions_{};
    ProductionStartup startup_{};
    struct NeutralRun {
        uint32_t limit{}, submitted{}, period_ns{}, cutoff_us{};
        bool active{}, awaiting{}, closed{};
    } neutral_run_{};
    struct Session {
        ChannelConfiguration channels[DCU_DEVICE_CAN_BUS_COUNT]{};
        BoardConfiguration config{};
        BoardCommand board_command{};
        ChannelCommand channel_command[DCU_DEVICE_CAN_BUS_COUNT]{};
        OperatingMode mode{};
        Owner owner{};
        bool operational{}, session_armed{}, resume_pdo_owner{};
        bool seen_cycle{}, config_ready{};
        uint32_t last_command_ms{}, last_cycle{}, tx_total{}, last_process_us{};
        uint8_t query_cursor[DCU_DEVICE_CAN_BUS_COUNT]{};
        uint16_t board_fault{};
        uint32_t channel_tx_drop[DCU_DEVICE_CAN_BUS_COUNT]{};
        uint32_t legacy_rx_drop[DCU_DEVICE_CAN_BUS_COUNT]{};
        uint32_t publication_sequence{}, sdo_accepted_sequence[DCU_MOTOR_DEVICE_CAPACITY]{};
    } state_{};
};

}
#endif
