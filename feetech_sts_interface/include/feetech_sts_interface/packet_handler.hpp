// Copyright 2023 fateshelled.
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

namespace feetech_sts_interface
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

    bool ping(int);
    bool setWheelMode(const u_char);

    bool writePosEx(const u_char, const u_short, const u_short, const u_short);
    bool syncWritePosEx(const u_char *, const u_char, const u_short *, const u_short *, const u_short *);

    bool setTorque(const u_char, const bool);
    bool calbrationOffset(const u_char);

    bool writeSpd(const u_char, const int16_t, const u_char);

    int16_t readBuf(const u_char, const u_char);
    int16_t readPos(const u_char);
    int16_t readSpd(const u_char);

    bool syncWrite(const u_char *, const u_char, const u_char, u_char *, const u_char);

  private:
    const rclcpp::Logger getLogger() noexcept;
    u_char calcChecksum(const u_char *const, const size_t) noexcept;
    bool checkWritten(const char *);

    bool writeBuf(const u_char, const u_char, const u_char *, const u_int8_t, const u_char);
    void host2STS(u_char *const, u_char *const, const u_short);
    int STS2host(const u_char, const u_char);
  };
} // namespace feetech_scs_interface
