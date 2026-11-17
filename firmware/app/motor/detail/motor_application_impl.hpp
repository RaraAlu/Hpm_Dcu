#ifndef DCU_MOTOR_APPLICATION_IMPL_HPP
#define DCU_MOTOR_APPLICATION_IMPL_HPP

#include "axis_controller.hpp"
#include "can_bus_scheduler.hpp"
#include "motor_application.hpp"
#include "motor_pdo_layout.hpp"
#include "raw_can_gateway.hpp"
#include <cstdint>

namespace dcu::app::motor_application::detail {

// MainLoop and SDO callbacks share one task; only the C/C++ entry points own this object.
class MotorApplication {
public:
    void initialize();
    void process();
    void operational(bool operational);
    void cycle_fault();
    void output(const void *data);
    bool configuration_writable() const;
    bool owner_change_allowed() const;
    bool apply_configuration();
    bool select_profile(unsigned profile);
    void owner_changed();
    void commit_sdo_command(unsigned axis, mc_sdo_command_t &command, uint32_t now_ms);
    bool commit_parameter(unsigned axis, bool maintenance, uint32_t now_ms, bool preop);
    PdoPublication prepare_publication();
    bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

private:
    unsigned frame_us(unsigned bus, unsigned bytes) const;
    ControlSet controls_on_bus(unsigned bus) const;
    bool any_active() const;
    bool channel_active(unsigned bus) const;
    bool command_permitted(uint32_t now_ms) const;
    void stop_all(uint16_t fault);
    void check_timeouts(uint32_t now_ms);
    void accept_candidate(uint32_t now_ms);
    void consume_feedback(unsigned bus, const dcu_can_frame_t &frame,
                          uint32_t timestamp_ms, uint32_t now_ms);
    void publish(uint32_t now_ms);

    AxisController axes_[ENCOS_AXIS_COUNT]{};
    RawCanGateway raw_{};
    BusScheduler schedule_[ENCOS_CHANNEL_COUNT]{};
    struct Session {
        mc_channel_config_t channels[ENCOS_CHANNEL_COUNT]{};
        mc_board_config_t config{};
        mc_board_rx_t board_command{};
        mc_channel_rx_t channel_command[ENCOS_CHANNEL_COUNT]{};
        uint8_t candidate[dcu::ethercat::pdo_layout::max_output_bytes]{};
        uint16_t output_size{};
        bool operational{}, pending{}, session_armed{}, resume_pdo_owner{};
        bool seen_cycle{}, config_ready{};
        uint32_t last_command_ms{}, last_cycle{}, tx_total{}, last_process_us{};
        uint8_t query_slot[ENCOS_CHANNEL_COUNT]{};
        uint16_t board_fault{};
        uint32_t channel_tx_drop[ENCOS_CHANNEL_COUNT]{};
    } state_{};
};

}
#endif
