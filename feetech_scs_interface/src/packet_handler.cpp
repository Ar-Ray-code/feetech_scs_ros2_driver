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
#include "feetech_scs_interface/INST.h"
#include "feetech_scs_interface/SCSCL.h"

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

u_char PacketHandler::calcChecksum(const u_char * const buf, const size_t len) noexcept
{
  u_char checksum = 0;
  for (size_t i = 2; i < len - 1; ++i) {
    checksum += buf[i];
  }
  return ~checksum;
}

void PacketHandler::host2SCS(u_char * const data_l, u_char * const data_h, const u_short data)
{
  bool END = 0; // SCS
  if (END) {
    *data_l = (data >> 8);
    *data_h = (data & 0xff);
  } else {
    *data_h = (data >> 8);
    *data_l = (data & 0xff);
  }
}

int PacketHandler::SCS2host(const u_char data_l, const u_char data_h)
{
  bool END = 0; // SCS
  if (END) {
    return int((data_l << 8) + data_h);
  } else {
    return int((data_h << 8) + data_l);
  }
}


int16_t PacketHandler::readBuf(
  const u_char id, const u_char mem_adder)
{
  const int length = 2;
  const int write_buf_size = length + 6;
  u_char write_buf[write_buf_size];
  write_buf[0] = 0xFF;
  write_buf[1] = 0xFF;

  write_buf[2] = id;
  write_buf[3] = 2 + length;  // Message length
  write_buf[4] = INST_READ;
  write_buf[5] = mem_adder;
  write_buf[6] = length;
  write_buf[7] = calcChecksum(write_buf, write_buf_size);

  std::string sent = "";
  for (long int i = 0; i < write_buf_size; i++) {
    sent += std::to_string(write_buf[i]) + " ";
  }

  const ssize_t write_ret = port_handler_->writePort(
    reinterpret_cast<char *>(write_buf), write_buf_size);
  if (write_ret == -1) {
    return -1;
  }

  using namespace std::chrono_literals;  // NOLINT
  const auto clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const auto started = clock_->now();
  while (port_handler_->getBytesAvailable() == 0) {
    if (clock_->now() - started > 1s) {
      std::cout << "timeout" << std::endl;
      return -1;
    }
  }

  char read_buf[128];
  const ssize_t read_ret = port_handler_->readPort(read_buf, sizeof(read_buf));
  if (read_ret == -1) {
    return -1;
  }
  std::string recv = "";
  for (long int i = 0; i < read_ret; i++) {
    recv += std::to_string(read_buf[i]) + " ";
  }
  return SCS2host(read_buf[5], read_buf[6]);
}

bool PacketHandler::writeBuf(
  const u_char id, const u_char mem_adder, const u_char * const param,
  const u_int8_t param_len, const u_char function)
{
  const int length = param_len + 7;
  u_char write_buf[length];
  write_buf[0] = 0xFF;
  write_buf[1] = 0xFF;

  write_buf[2] = id;
  write_buf[3] = 3 + param_len;  // Message length

  write_buf[4] = function;
  write_buf[5] = mem_adder;
  for (size_t i = 0; i < param_len; ++i) {
    write_buf[6 + i] = param[i];
  }
  write_buf[6 + param_len] = calcChecksum(write_buf, sizeof(write_buf));

  std::string sent = "";
  for (size_t i = 0; i < sizeof(write_buf); ++i) {
    sent += std::to_string(write_buf[i]) + " ";
  }
  RCLCPP_DEBUG(
    this->getLogger(),
    "Write[%zu]: %s", sizeof(write_buf), sent.c_str());

  const ssize_t write_ret = this->port_handler_->writePort(
    reinterpret_cast<char *>(write_buf), sizeof(write_buf));
  if (write_ret == -1) {
    return false;
  }
  return true;
}

bool PacketHandler::setPWMMode(const u_char id)
{
  u_char bBuf[4] = {0x00, 0x00, 0x00, 0x00};
  return writeBuf(id, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4, INST_WRITE);
}

bool PacketHandler::ping(int id)
{
  return writeBuf(id, 0, nullptr, 0, INST_PING);
}

bool PacketHandler::writePos(
  const u_char id, const u_short position, const u_short time, const u_short speed)
{
  u_char bBuf[6];
  host2SCS(bBuf + 0, bBuf + 1, position);
  host2SCS(bBuf + 2, bBuf + 3, time);
  host2SCS(bBuf + 4, bBuf + 5, speed);

  return writeBuf(id, SCSCL_GOAL_POSITION_L, bBuf, 6, INST_WRITE);
}

bool PacketHandler::writeSpd(
  const u_char id, const int16_t speed)
{
  int16_t speed_pwm = speed;
  if (speed_pwm < 0) {
    speed_pwm = -speed_pwm;
    speed_pwm |= (1 << 10);
  }
  u_char bBuf[2];
  host2SCS(bBuf + 0, bBuf + 1, speed_pwm);

  return writeBuf(id, SCSCL_GOAL_TIME_L, bBuf, 2, INST_WRITE);
}

int16_t PacketHandler::readPos(const u_char id)
{
  auto ret = this->readBuf(id, SCSCL_PRESENT_POSITION_L);
  if (ret == -1) {
    return -1;
  }
  return ret;
}

int16_t PacketHandler::readSpd(const u_char id)
{
  auto ret = this->readBuf(id, SCSCL_PRESENT_SPEED_L);
  if (ret == -1) {
    return -1;
  }
  int16_t speed = ret;
  if (ret && (speed & (1 << 15))) {
    speed = -(speed & ~(1 << 15));
  }
  return speed;
}

const rclcpp::Logger PacketHandler::getLogger() noexcept
{
  return this->logger_;
}

}  // namespace feetech_scs_interface
