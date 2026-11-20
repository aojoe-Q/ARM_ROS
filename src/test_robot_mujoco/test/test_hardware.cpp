#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  require(argc == 2, "Expected model path");
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto hardware = loader.createSharedInstance("test_robot_mujoco/MujocoSystem");
  hardware_interface::HardwareInfo info;
  info.name = "MujocoTest";
  info.type = "system";
  info.hardware_parameters = {{"model_path", argv[1]}, {"gravity_compensation", "true"},
    {"publish_states", "false"}};
  for (const auto & name : {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint8"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    hardware_interface::InterfaceInfo position, velocity, effort;
    position.name = "position";
    position.initial_value = "0";
    velocity.name = "velocity";
    effort.name = "effort";
    joint.command_interfaces = {position};
    joint.state_interfaces = {position, velocity, effort};
    info.joints.push_back(joint);
  }
  require(hardware->on_init(info) == hardware_interface::CallbackReturn::SUCCESS, "Initialization failed");
  auto commands = hardware->export_command_interfaces();
  auto states = hardware->export_state_interfaces();
  require(commands.size() == 7 && states.size() == 21, "Incorrect interfaces");
  rclcpp_lifecycle::State lifecycle;
  require(hardware->on_activate(lifecycle) == hardware_interface::CallbackReturn::SUCCESS, "Activation failed");
  const double targets[] = {0.2, 0.3, 0.2, 0.1, 0.1, 0.1, 0.01};
  const auto period = rclcpp::Duration::from_seconds(0.01);
  for (int step = 0; step < 600; ++step) {
    const rclcpp::Time time(static_cast<int64_t>(step) * 10000000);
    for (size_t i = 0; i < 7; ++i) {commands[i].set_value(targets[i] * std::min(step / 200.0, 1.0));}
    require(hardware->write(time, period) == hardware_interface::return_type::OK, "Write failed");
    require(hardware->read(time, period) == hardware_interface::return_type::OK, "Read failed");
  }
  for (size_t i = 0; i < 7; ++i) {
    const double error = std::abs(states[3 * i].get_value() - targets[i]);
    std::cout << info.joints[i].name << " tracking error: " << error << '\n';
    require(error < (i == 6 ? 0.001 : 0.01), "Tracking error too large");
  }
  double max_effort = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    const double effort = states[3 * i + 2].get_value();
    require(std::isfinite(effort), "Nonfinite effort feedback");
    max_effort = std::max(max_effort, std::abs(effort));
    std::cout << info.joints[i].name << " drive effort: " << effort << std::endl;
  }
  require(max_effort > 0.1, "Compensated arm must report support effort");
  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  require(hardware->write(rclcpp::Time(0), period) == hardware_interface::return_type::ERROR,
    "Nonfinite command must be rejected");
  require(hardware->on_deactivate(lifecycle) == hardware_interface::CallbackReturn::SUCCESS, "Deactivation failed");
  std::cout << "Hardware plugin integration passed\n";
}
