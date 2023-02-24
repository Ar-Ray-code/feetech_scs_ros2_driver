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

#include "feetech_scs_interface/packet_handler.hpp"


namespace feetech_scs_interface
{

PacketHandler::PacketHandler(
  PortHandler::SharedPtr port_handler,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_if,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_if)
: port_handler_(port_handler),
  clock_(clock_if ? clock_if->get_clock() : std::make_shared<rclcpp::Clock>()),
  logger_(logging_if ? logging_if->get_logger() : rclcpp::get_logger("PacketHandler"))
{
}

void PacketHandler::ping()
{
  u_char write_buf[6];
  write_buf[0] = 0xFF;
  write_buf[1] = 0xFF;

  write_buf[2] = 1;  // Motor ID
  write_buf[3] = 2;  // Message length

  write_buf[4] = 0x01;  // Function ID
  write_buf[5] = 0xFB;

  std::string sent = "";
  for (size_t i = 0; i < sizeof(write_buf); ++i) {
    sent += std::to_string(write_buf[i]) + " ";
  }
  RCLCPP_INFO(
    this->getLogger(),
    "Write[%zu]: %s", sizeof(write_buf), sent.c_str());

  const ssize_t write_ret = this->port_handler_->writePort(
    reinterpret_cast<char *>(write_buf), sizeof(write_buf));
  if (write_ret == -1) {
    return;
  }


  using namespace std::chrono_literals;  // NOLINT
  const auto started = this->clock_->now();
  while (this->port_handler_->getBytesAvailable() == 0) {
    if (this->clock_->now() - started > 1s) {
      RCLCPP_ERROR(this->getLogger(), "timeout");
      return;
    }
  }

  char read_buf[128];
  const ssize_t read_ret = this->port_handler_->readPort(read_buf, sizeof(read_buf));
  if (read_ret == -1) {
    return;
  }
  RCLCPP_INFO(this->getLogger(), "Recv[%zu]: %s", read_ret, read_buf);
}

const rclcpp::Logger PacketHandler::getLogger() noexcept
{
  return this->logger_;
}
}  // namespace feetech_scs_interface
