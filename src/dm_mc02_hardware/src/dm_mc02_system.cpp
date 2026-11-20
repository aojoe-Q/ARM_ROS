#include "dm_mc02_hardware/dm_mc02_system.hpp"
#include <algorithm>
#include <utility>
#include <limits>
#include <cmath>
#include <stdexcept>

namespace dm_mc02_hardware {

CallbackReturn DmMc02System::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Expect 7 joints: joint1-6 (arm) + joint8 (gripper)
    if (info_.joints.size() != 7) {
        RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Expected 7 joints, got %zu", info_.joints.size());
        return CallbackReturn::ERROR;
    }

    hw_states_positions_.resize(7, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(7, 0.0);
    hw_states_efforts_.resize(7, 0.0);
    hw_commands_positions_.resize(7, std::numeric_limits<double>::quiet_NaN());

    // Read device path from hardware_parameters
    auto it = info_.hardware_parameters.find("device");
    device_ = (it != info_.hardware_parameters.end()) ? it->second : "/dev/ttyACM0";

    const char* names[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint8"};
    for (size_t i = 0; i < 7; ++i) {
        if (info_.joints[i].name != names[i]) {
            RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Unexpected joint order at %zu", i);
            return CallbackReturn::ERROR;
        }
    }
    try {
        load_config_from_params();
        const auto error = link_cfg_.validation_error();
        if (!error.empty()) throw std::invalid_argument(error);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Invalid configuration: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

void DmMc02System::load_config_from_params() {
    auto get_param = [&](const std::string& key, double default_val) -> double {
        auto it = info_.hardware_parameters.find(key);
        if (it != info_.hardware_parameters.end()) {
            size_t used = 0;
            const double value = std::stod(it->second, &used);
            if (used != it->second.size() || !std::isfinite(value))
                throw std::invalid_argument("invalid parameter: " + key);
            return value;
        }
        return default_val;
    };

// Per-joint config (joints 0-5)
    const dm_mc02::ArmLinkConfig defaults;
    for (size_t i = 0; i < 6; ++i) {
        std::string prefix = "joint" + std::to_string(i + 1) + "_";
        link_cfg_.joints[i].sign = get_param(prefix + "sign", 1.0);
        link_cfg_.joints[i].offset = get_param(prefix + "offset", 0.0);
        link_cfg_.joints[i].mcu_min = get_param(prefix + "mcu_min", defaults.joints[i].mcu_min);
        link_cfg_.joints[i].mcu_max = get_param(prefix + "mcu_max", defaults.joints[i].mcu_max);
        link_cfg_.joints[i].max_step_rad = get_param(prefix + "max_step_rad", 0.01);
    }

    // Grip config
    link_cfg_.grip.q_close_m = get_param("grip_q_close_m", -0.000523);
    link_cfg_.grip.q_open_m = get_param("grip_q_open_m", 0.009530);
    link_cfg_.grip.calibrated = get_param("grip_calibrated", 0.0) > 0.5;
    link_cfg_.grip.mcu_closed = get_param("grip_mcu_closed", defaults.grip.mcu_closed);
    link_cfg_.grip.mcu_open = get_param("grip_mcu_open", defaults.grip.mcu_open);
    link_cfg_.grip.mcu_min = get_param("grip_mcu_min", -12.0);
    link_cfg_.grip.mcu_max = get_param("grip_mcu_max", 0.0);
    link_cfg_.grip.max_step = get_param("grip_max_step", 0.01);
    link_cfg_.grip.command_enable = (get_param("grip_command_enable", 0.0) > 0.5);

    // Global config
    const double valid_ms = get_param("valid_time_ms", 100.0);
    const double timeout_ms = get_param("state_timeout_ms", 100.0);
    if (valid_ms < 40 || valid_ms > 300 || timeout_ms < 1 || timeout_ms > 100)
        throw std::invalid_argument("invalid timing parameters");
    link_cfg_.valid_time_ms = static_cast<uint16_t>(valid_ms);
    link_cfg_.state_timeout_ms = static_cast<uint32_t>(timeout_ms);
    link_cfg_.allow_enable = (get_param("allow_enable", 0.0) > 0.5);
    link_cfg_.mapping_calibrated = get_param("mapping_calibrated", 0.0) > 0.5;
}

std::vector<hardware_interface::StateInterface> DmMc02System::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < 7; ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DmMc02System::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
    }
    return command_interfaces;
}

CallbackReturn DmMc02System::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("DmMc02System"), "Configuring hardware, device=%s", device_.c_str());

    port_ = std::make_unique<dm_mc02::SerialPort>();
    if (!port_->open(device_)) {
        RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Failed to open %s", device_.c_str());
        return CallbackReturn::ERROR;
    }

    link_ = std::make_unique<dm_mc02::ArmLink>(link_cfg_);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    last_log_ms_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("DmMc02System"), "Hardware configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DmMc02System::on_cleanup(const rclcpp_lifecycle::State&) {
    send_stop_and_disable();
    if (port_) port_->close();
    link_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DmMc02System::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("DmMc02System"), "Activating (telemetry only until controller claims)");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DmMc02System::on_deactivate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("DmMc02System"), "Deactivating");
    send_stop_and_disable();
    if (link_) link_ = std::make_unique<dm_mc02::ArmLink>(link_cfg_);
    std::fill(hw_commands_positions_.begin(), hw_commands_positions_.end(),
              std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
}

return_type DmMc02System::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Only allow position interfaces
    for (const auto& iface : start_interfaces) {
        const bool ours = std::any_of(info_.joints.begin(), info_.joints.end(),
            [&](const auto& joint) { return iface.rfind(joint.name + "/", 0) == 0; });
        if (!ours) continue;  // ros2_control can pass interfaces owned by another hardware component.
        if (iface.substr(iface.find('/') + 1) != "position" ||
            !link_cfg_.allow_enable || !link_cfg_.mapping_calibrated ||
            (iface == "joint8/position" && !link_cfg_.grip.command_enable)) {
            RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Command interface disabled: %s", iface.c_str());
            return return_type::ERROR;
        }
    }
    return return_type::OK;
}

return_type DmMc02System::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    // Update claimed flags
    for (const auto& iface : stop_interfaces) {
        for (size_t i = 0; i < 7; ++i) {
            if (iface == info_.joints[i].name + "/position") {
                link_->set_claimed(i, false);
                hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    for (const auto& iface : start_interfaces) {
        for (size_t i = 0; i < 7; ++i) {
            if (iface == info_.joints[i].name + "/position") {
                link_->set_claimed(i, true);
                hw_commands_positions_[i] = hw_states_positions_[i];
            }
        }
    }
    return return_type::OK;
}

return_type DmMc02System::read(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!port_ || !link_) return return_type::ERROR;

    uint8_t rx_buf[4096];
    size_t n = port_->read(rx_buf, sizeof(rx_buf));
    if (n == static_cast<size_t>(-1)) {
        RCLCPP_ERROR(rclcpp::get_logger("DmMc02System"), "Serial port read I/O error");
        return return_type::ERROR;
    }

    uint32_t now_ms = static_cast<uint32_t>(clock_->now().nanoseconds() / 1000000);
    link_->receive(now_ms, rx_buf, n);

    // Update state interfaces (joints 0-5, then grip at 6)
    for (size_t i = 0; i < 6; ++i) {
        link_->get_joint_state(i, hw_states_positions_[i], hw_states_velocities_[i], hw_states_efforts_[i]);
    }
    link_->get_grip_state(hw_states_positions_[6], hw_states_velocities_[6], hw_states_efforts_[6]);

    // Periodic status log
    auto status = link_->status();
    if ((now_ms - last_log_ms_) > 5000) {
        RCLCPP_INFO(rclcpp::get_logger("DmMc02System"),
            "State=%d mode=%d sys=%d tx=%u rx=%u crc_err=%u ack_err=%u fault=0x%04x flags=0x%02x feedback=0x%02x ack=0x%02x grip_motor=%.6f reason=%s",
            static_cast<int>(status.state), status.arm_mode, status.system_state,
            status.frames_sent, status.frames_received, status.crc_errors, status.ack_errors,
            status.fault_code, status.flags, status.feedback_valid, status.last_ack_result,
            dm_mc02::urad_to_rad(status.grip_motor_urad), status.reason.c_str());
        last_log_ms_ = now_ms;
    }

    return return_type::OK;
}

return_type DmMc02System::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!port_ || !link_) return return_type::ERROR;

    // Push commands to link (joints 0-5, then grip at 6)
    for (size_t i = 0; i < 6; ++i) {
        link_->set_joint_command(i, hw_commands_positions_[i]);
    }
    link_->set_grip_command(hw_commands_positions_[6]);

    uint32_t now_ms = static_cast<uint32_t>(clock_->now().nanoseconds() / 1000000);
    auto generated = link_->step(now_ms, nullptr, 0);

    // Generate only after this cycle's controller commands have been copied.
    auto write_frame = [&](const std::vector<uint8_t>& frame) -> bool {
        if (frame.empty()) return true;
        const size_t written = port_->write(frame);
        if (written != frame.size()) {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DmMc02System"), *clock_, 1000, "Short/failed write");
            return false;
        }
        return true;
    };
    if (!write_frame(generated)) {
        send_stop_and_disable();
        return return_type::ERROR;
    }

    // Check for fault
    auto status = link_->status();
    if (status.state == dm_mc02::ArmState::FAULT) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DmMc02System"), *clock_, 1000,
            "PC control stopped: %s (ACK=0x%02x, MCU fault=0x%04x). Deactivate all command controllers before retry.",
            status.reason.c_str(), status.last_ack_result, status.fault_code);
        // Keep telemetry and controller switching available for explicit recovery.
    }

    return return_type::OK;
}

void DmMc02System::send_stop_and_disable() {
    if (!port_ || !port_->is_open()) return;

    dm_mc02::FrameEncoder enc;
    const uint32_t now_ms = clock_ ? static_cast<uint32_t>(clock_->now().nanoseconds() / 1000000) : 0U;
    enc.update_host_time(now_ms);

    auto stop_frame = enc.encode_stop();
    port_->write(stop_frame);

    auto disable_frame = enc.encode_enable(0);
    port_->write(disable_frame);
}

}  // namespace dm_mc02_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dm_mc02_hardware::DmMc02System, hardware_interface::SystemInterface)














