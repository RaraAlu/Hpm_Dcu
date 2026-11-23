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
    RawState raw_state() const { return layout_.state; }
    bool request_layout(OperatingMode target, uint32_t profile, uint32_t revision,
                        const uint8_t (&digest)[DCU_GATEWAY_DIGEST_BYTES]);
    bool map_layout(OperatingMode mode);
    bool reset_layout();
    bool request_topology();
    bool topology_pending() const { return layout_.topology; }
    bool commit_topology(const dcu_production_profile_t &, uint32_t buses, uint32_t devices);
    void layout_fault();
    uint32_t raw_drops() const;
    uint32_t pure_raw_limits() const;
    bool set_pure_raw_limits(uint32_t packed);
    bool stage_raw_motor_owner(uint8_t buses);
    uint8_t raw_motor_owner() const { return layout_.motor_owner; }
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
    void set_cyclic_feedback_window(uint32_t period_us, uint32_t deadline_us);
    uint32_t cyclic_feedback_required() const { return cyclic_.required; }
    uint32_t cyclic_feedback_missing() const { return cyclic_.required & ~cyclic_.matched; }
    void fault_cyclic_feedback(uint32_t missing);
    bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

private:
    bool drain_raw();
    bool drain_layout();
    bool begin_layout(OperatingMode, uint32_t, uint32_t, const uint8_t (&)[DCU_GATEWAY_DIGEST_BYTES], bool);
    bool finish_layout(OperatingMode, bool topology);
    bool raw_channel(unsigned bus) const;
    bool raw_motor_bus_idle(uint8_t buses) const;
    void checkpoint_raw_losses();
    uint32_t channel_losses(unsigned bus) const;
    void clear_layout_business();
    void advance_layout(uint32_t now);
    void activate_layout();
    void reject_raw(const CycleCommand &candidate);
    struct RawLayout {
        RawState state{};
        OperatingMode target{};
        bool selected{}, started{}, reset{}, switching{}, running{}, topology{};
        uint32_t drain_ms{}, drops{}, baseline{}, revision{};
        uint64_t session{};
        uint8_t limits[DCU_DEVICE_CAN_BUS_COUNT]{};
        uint8_t pending_motor_owner{}, motor_owner{};
        uint32_t loss_origin[DCU_DEVICE_CAN_BUS_COUNT]{};
    } layout_{};
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
    bool process_raw(const CycleCommand &candidate, uint32_t now_ms, uint8_t reserved_buses = 0);
    void collect_tx_events(unsigned bus);
    void service_cyclic_feedback(uint32_t now_ms, uint32_t now_us);
    bool consume_feedback(unsigned bus, const dcu_can_frame_t &frame,
                          const dcu_can_rx_stamp_t &stamp, uint32_t now_ms, uint32_t now_us);

    motor::MotorInstance devices_[DCU_MOTOR_DEVICE_CAPACITY]{};
    unsigned device_count_{};
    uint8_t devices_on_bus_[DCU_DEVICE_CAN_BUS_COUNT][DCU_MOTOR_DEVICES_PER_BUS]{};
    uint8_t bus_device_count_[DCU_DEVICE_CAN_BUS_COUNT]{};
    ChannelConfiguration catalogue_channels_[DCU_DEVICE_CAN_BUS_COUNT]{};
    RawCanGateway raw_{};
    BusScheduler schedule_[DCU_DEVICE_CAN_BUS_COUNT]{};
    CanTransactions transactions_{};
    ProductionStartup startup_{};
    uint32_t readiness_devices_{};
    uint32_t timestamped_encos_{};
    struct CyclicFeedback {
        uint32_t period_us{}, deadline_us{}, required{}, submitted{}, matched{};
        uint8_t buses{};
        dcu_can_eof_t tx_eof[DCU_MOTOR_DEVICE_CAPACITY]{};
    } cyclic_{};
    struct NeutralRun {
        uint32_t limit[DCU_MOTOR_DEVICE_CAPACITY]{}, submitted[DCU_MOTOR_DEVICE_CAPACITY]{};
        uint32_t period_ns{}, cutoff_us{}, active{}, awaiting{}, closed{}, sent_this_cycle{};
        unsigned bus{DCU_DEVICE_CAN_BUS_COUNT};
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
