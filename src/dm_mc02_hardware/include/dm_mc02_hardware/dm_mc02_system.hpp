#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "dm_mc02_hardware/arm_link.hpp"
#include "dm_mc02_hardware/serial_port.hpp"
#include <memory>
#include <vector>
#include <string>

namespace dm_mc02_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class DmMc02System : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Hardware info
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_efforts_;
    std::vector<double> hw_commands_positions_;

    // Config
    std::string device_;
    dm_mc02::ArmLinkConfig link_cfg_;

    // Runtime
    std::unique_ptr<dm_mc02::SerialPort> port_;
    std::unique_ptr<dm_mc02::ArmLink> link_;
    rclcpp::Clock::SharedPtr clock_;
    uint32_t last_log_ms_;

    void load_config_from_params();
    void send_stop_and_disable();
};

}  // namespace dm_mc02_hardware

