// Copyright 2023 Ar-Ray-code.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "feetech_sts_hardware/feetech_sts_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace feetech_sts_hardware
{
constexpr const char * kFeetechStsHardware = "FeetechStsHardware";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

CallbackReturn FeetechStsHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kFeetechStsHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "baud_rate: %d", baud_rate);

  // open port
  port_handler_ = std::make_shared<h6x_serial_interface::PortHandler>(usb_port);
  packet_handler_ = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler_);

  port_handler_->configure(baud_rate);
  if (!port_handler_->open()) {
    return CallbackReturn::ERROR;
  }

  set_control_mode(ControlMode::Position, true);
  set_joint_params();
  // TODO: enable torque
  enable_torque(false);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechStsHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kFeetechStsHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechStsHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kFeetechStsHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn FeetechStsHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kFeetechStsHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  enable_torque(false);

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kFeetechStsHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type FeetechStsHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  for (uint i = 0; i < joints_.size(); i++) {
    float current_pos = -1;
    // float current_spd = feetech_sts_interface::STS3032::data2angle(packet_handler_->readSpd(joint_ids_[i]));
    while (current_pos <= 0) {
      current_pos = feetech_sts_interface::STS3032::data2angle(packet_handler_->readPos(joint_ids_[i]));
    }
    joints_[i].state.position = current_pos / 180.0 * M_PI;
    // print
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "joint_id %d: %f", i, joints_[i].state.position);
    // joints_[i].state.velocity = current_spd / 180.0 * M_PI;
  }


  return return_type::OK;
}

return_type FeetechStsHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Velocity control
  // TODO: Test PWM mode
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != j.prev_command.velocity; })) {
    set_control_mode(ControlMode::Velocity);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }

  // Position control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.position != j.prev_command.position; })) {
    set_control_mode(ControlMode::Position);
    if(mode_changed_)
    {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // if all command values are unchanged, then remain in existing control mode and set corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    default: // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kFeetechStsHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }

}

return_type FeetechStsHardware::enable_torque(const bool enabled)
{
  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      packet_handler_->setTorque(joint_ids_[i], enabled);
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      packet_handler_->setTorque(joint_ids_[i], enabled);
    }
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type FeetechStsHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  mode_changed_ = false;

  // if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
  //   bool torque_enabled = torque_enabled_;
  //   if (torque_enabled) {
  //     enable_torque(false);
  //   }

  //   // TODO: set velocity control mode
  //   for (uint i = 0; i < joint_ids_.size(); ++i) {
  //     packet_handler_->setPWMMode(joint_ids_[i]);
  //   }

  //   RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "Velocity control");
  //   if(control_mode_ != ControlMode::Velocity)
  //   {
  //     mode_changed_ = true;
  //     control_mode_ = ControlMode::Velocity;
  //   }

  //   if (torque_enabled) {
  //     enable_torque(true);
  //   }
  //   return return_type::OK;
  // }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    // TODO: set position control mode (default)
    // for (uint i = 0; i < joint_ids_.size(); ++i) {
    // }
    RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "Position control");
    if(control_mode_ != ControlMode::Position)
    {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kFeetechStsHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type FeetechStsHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn FeetechStsHardware::set_joint_positions()
{
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].command.position = joints_[i].command.position;

    // write position
    int position = feetech_sts_interface::STS3032::angle2data(joints_[i].command.position * 180.0 / M_PI);
    packet_handler_->writePos(ids[i], position, 50, 1000);
  }
  return CallbackReturn::SUCCESS;
}

// TODO: test writeSpd
CallbackReturn FeetechStsHardware::set_joint_velocities()
{
  // std::vector<int32_t> commands(info_.joints.size(), 0);
  // std::vector<uint8_t> ids(info_.joints.size(), 0);

  // RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "set_joint_velocities");

  // std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  // for (uint i = 0; i < ids.size(); i++) {
  //   int spd = feetech_sts_interface::STS3032::angle2data(joints_[i].command.velocity * 180.0 / M_PI);
  //   packet_handler_->writeSpd(ids[i], spd);
  //   joints_[i].prev_command.velocity = joints_[i].command.velocity;
  // }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsHardware::set_joint_params()
{
  RCLCPP_INFO(rclcpp::get_logger(kFeetechStsHardware), "set_joint_params");

  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        auto target_id = joint_ids_[i];

        int target_angle = feetech_sts_interface::STS3032::angle2data(value);
        packet_handler_->writePos(target_id, target_angle, 50, 1000);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_sts_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_sts_hardware::FeetechStsHardware, hardware_interface::SystemInterface)
