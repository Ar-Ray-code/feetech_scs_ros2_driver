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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <h6x_serial_interface/h6x_serial_interface.hpp>

namespace feetech_scs_interface
{
class PacketHandler
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;
  const PortHandler::SharedPtr port_handler_;
  const rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Logger logger_;

public:
  PacketHandler() = delete;
  explicit PacketHandler(
    PortHandler::SharedPtr,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr = nullptr,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr = nullptr);

  void ping();

private:
  const rclcpp::Logger getLogger() noexcept;
};
}  // namespace feetech_scs_interface
