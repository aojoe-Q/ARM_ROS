#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace test_robot_mujoco
{
class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      char error[2048] = {};
      model_.reset(mj_loadXML(info.hardware_parameters.at("model_path").c_str(), nullptr, error, sizeof(error)));
      if (!model_) {throw std::runtime_error(error);}
      data_.reset(mj_makeData(model_.get()));
      if (!data_) {throw std::runtime_error("Could not allocate MuJoCo data");}
      gravity_compensation_ = info.hardware_parameters.at("gravity_compensation") == "true";
      const auto count = info.joints.size();
      positions_.resize(count);
      velocities_.resize(count);
      efforts_.resize(count);
      commands_.resize(count);
      for (size_t i = 0; i < count; ++i) {
        const auto & joint = info.joints[i];
        if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "position" ||
            (joint.state_interfaces.size() != 2 && joint.state_interfaces.size() != 3)) {
          throw std::runtime_error("Expected position command and position/velocity states: " + joint.name);
        }
        bool has_position = false, has_velocity = false;
        for (const auto & state : joint.state_interfaces) {
          if (state.name == "position") {
            has_position = true;
            positions_[i] = state.initial_value.empty() ? 0.0 : std::stod(state.initial_value);
          }
          has_velocity |= state.name == "velocity";
          if (state.name != "position" && state.name != "velocity" && state.name != "effort") {
            throw std::runtime_error("Unsupported state interface: " + state.name);
          }
        }
        if (!has_position || !has_velocity || !std::isfinite(positions_[i])) {
          throw std::runtime_error("Invalid state interfaces: " + joint.name);
        }
        const int id = mj_name2id(model_.get(), mjOBJ_JOINT, joint.name.c_str());
        const int actuator = mj_name2id(model_.get(), mjOBJ_ACTUATOR, (joint.name + "_position").c_str());
        if (id < 0 || actuator < 0 ||
            (model_->jnt_type[id] != mjJNT_HINGE && model_->jnt_type[id] != mjJNT_SLIDE)) {
          throw std::runtime_error("Missing scalar joint/actuator: " + joint.name);
        }
        qpos_.push_back(model_->jnt_qposadr[id]);
        dofs_.push_back(model_->jnt_dofadr[id]);
        actuators_.push_back(actuator);
        commands_[i] = positions_[i];
        data_->qpos[qpos_[i]] = positions_[i];
        data_->ctrl[actuator] = positions_[i];
      }
      // Initialize linear mimic constraints at their equilibrium positions.
      for (int i = 0; i < model_->neq; ++i) {
        if (model_->eq_type[i] == mjEQ_JOINT && model_->eq_obj2id[i] >= 0) {
          const auto * coefficients = model_->eq_data + i * mjNEQDATA;
          data_->qpos[model_->jnt_qposadr[model_->eq_obj1id[i]]] = coefficients[0] +
            coefficients[1] * data_->qpos[model_->jnt_qposadr[model_->eq_obj2id[i]]];
        }
      }
      mj_forward(model_.get(), data_.get());
      const auto publish = info.hardware_parameters.find("publish_states");
      if (publish == info.hardware_parameters.end() || publish->second != "false") {
        node_ = std::make_shared<rclcpp::Node>("mujoco_physics");
        publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/mujoco/joint_states", 10);
        targets_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/mujoco/joint_targets", 10);
      }
      for (int i = 0; i < model_->njnt; ++i) {
        state_.name.emplace_back(mj_id2name(model_.get(), mjOBJ_JOINT, i));
      }
      state_.position.resize(model_->njnt);
      state_.velocity.resize(model_->njnt);
      state_.effort.resize(model_->njnt);
      for (const auto & joint : info.joints) {targets_.name.push_back(joint.name);}
      targets_.position.resize(count);
      targets_.effort.resize(count);
      RCLCPP_INFO(rclcpp::get_logger("mujoco_physics"), "MuJoCo %s loaded: %d joints, %d actuators",
        mj_versionString(), static_cast<int>(model_->njnt), static_cast<int>(model_->nu));
      return hardware_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger("mujoco_physics"), "%s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i = 0; i < positions_.size(); ++i) {
      result.emplace_back(info_.joints[i].name, "position", &positions_[i]);
      result.emplace_back(info_.joints[i].name, "velocity", &velocities_[i]);
      for (const auto & state : info_.joints[i].state_interfaces) {
        if (state.name == "effort") {
          result.emplace_back(info_.joints[i].name, "effort", &efforts_[i]);
        }
      }
    }
    return result;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i = 0; i < commands_.size(); ++i) {
      result.emplace_back(info_.joints[i].name, "position", &commands_[i]);
    }
    return result;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    commands_ = positions_;
    remainder_ = 0;
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    if (active_) {
      // Bound catch-up work after a debugger pause or a stalled host.
      remainder_ += std::clamp(period.seconds(), 0.0, 0.1);
      while (remainder_ + 1e-12 >= model_->opt.timestep) {
        if (gravity_compensation_) {
          // Ideal feed-forward compensation, in addition to bounded PD actuators.
          mj_forward(model_.get(), data_.get());
          mju_copy(data_->qfrc_applied, data_->qfrc_bias, model_->nv);
        }
        mj_step(model_.get(), data_.get());
        remainder_ -= model_->opt.timestep;
      }
    }
    // mj_step integrates positions after evaluating forces. Re-evaluate at the
    // reported state so positions, velocities and forces share one sample.
    mj_forward(model_.get(), data_.get());
    if (gravity_compensation_) {
      mju_copy(data_->qfrc_applied, data_->qfrc_bias, model_->nv);
      mj_forward(model_.get(), data_.get());
    }
    for (size_t i = 0; i < positions_.size(); ++i) {
      positions_[i] = data_->qpos[qpos_[i]];
      velocities_[i] = data_->qvel[dofs_[i]];
      efforts_[i] = data_->qfrc_actuator[dofs_[i]] + data_->qfrc_applied[dofs_[i]];
      if (!std::isfinite(positions_[i]) || !std::isfinite(velocities_[i])) {
        return hardware_interface::return_type::ERROR;
      }
    }
    for (int i = 0; i < mjNWARNING; ++i) {
      if (data_->warning[i].number && (i == mjWARN_BADQPOS || i == mjWARN_BADQVEL || i == mjWARN_BADQACC)) {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_physics"), "MuJoCo reported unstable dynamics; stopping hardware");
        return hardware_interface::return_type::ERROR;
      }
    }
    state_.header.stamp = time;
    for (int i = 0; i < model_->njnt; ++i) {
      state_.position[i] = data_->qpos[model_->jnt_qposadr[i]];
      state_.velocity[i] = data_->qvel[model_->jnt_dofadr[i]];
      const int dof = model_->jnt_dofadr[i];
      state_.effort[i] = data_->qfrc_actuator[dof] + data_->qfrc_applied[dof];
    }
    targets_.header.stamp = time;
    for (size_t i = 0; i < commands_.size(); ++i) {
      // Report the clamped target actually used by the physics engine.
      targets_.position[i] = data_->ctrl[actuators_[i]];
      targets_.effort[i] = data_->qfrc_actuator[dofs_[i]];
    }
    if (publisher_) {
      publisher_->publish(state_);
      targets_publisher_->publish(targets_);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {return hardware_interface::return_type::OK;}
    for (size_t i = 0; i < commands_.size(); ++i) {
      if (!std::isfinite(commands_[i])) {return hardware_interface::return_type::ERROR;}
      const int id = actuators_[i];
      data_->ctrl[id] = std::clamp(commands_[i], model_->actuator_ctrlrange[2 * id],
        model_->actuator_ctrlrange[2 * id + 1]);
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
  std::vector<double> positions_, velocities_, commands_, efforts_;
  std::vector<int> qpos_, dofs_, actuators_;
  bool active_ = false, gravity_compensation_ = true;
  double remainder_ = 0;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr targets_publisher_;
  sensor_msgs::msg::JointState state_, targets_;
};
}  // namespace test_robot_mujoco
PLUGINLIB_EXPORT_CLASS(test_robot_mujoco::MujocoSystem, hardware_interface::SystemInterface)
