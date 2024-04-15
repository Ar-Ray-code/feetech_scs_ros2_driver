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

#ifndef FEETECH_STS_HARDWARE__FEETECH_CSC_HARDWARE_HPP_
#define FEETECH_STS_HARDWARE__FEETECH_CSC_HARDWARE_HPP_

#include <feetech_sts_interface/feetech_sts_interface.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>

#include "feetech_sts_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace feetech_sts_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode {
  Position,
  Velocity,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class FeetechStsHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FeetechStsHardware)

  FEETECH_STS_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  FEETECH_STS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FEETECH_STS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FEETECH_STS_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FEETECH_STS_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  FEETECH_STS_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FEETECH_STS_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();

  std::shared_ptr<h6x_serial_interface::PortHandler> port_handler_;
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
  bool use_dummy_{false};
};
}  // namespace feetech_sts_hardware

#endif  // FEETECH_STS_HARDWARE__FEETECH_CSC_HARDWARE_HPP_
