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
    // const ushort END_ = 1; // SCS
    const ushort END_ = 0; // STS

  public:
    PacketHandler() = delete;
    explicit PacketHandler(
        PortHandler::SharedPtr,
        const rclcpp::node_interfaces::NodeClockInterface::SharedPtr = nullptr,
        const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr = nullptr);

    bool ping(int id);

    bool writePos(const u_char id, const u_short position, const u_short time, const u_short speed);
    bool writePosEx(const u_char id, const int16_t position, const int16_t speed, const u_short acc);
    bool syncWritePosEx(const u_char *ID, const u_char IDN, const int16_t *Position, const int16_t *Speed, const u_short *ACC);
    // bool regWritePosEx(const u_char id, const int16_t position, const u_short speed, const u_short acc);

    // bool writeSpd(const u_char id, const int16_t speed, const u_short acc);
    bool syncWriteSpd(const u_char *ID, const u_char IDN, const int16_t *Speed, const u_short *ACC);

    // bool writeTime(const u_char id, const int16_t time);
    bool syncWriteTime(const u_char *ID, const u_char IDN, const int16_t *Time);

    bool setRotationMode(const u_char id);
    bool setWheelMode(const u_char id); // closed loop
    bool setOpenLoopWheelMode(const u_char id); // open loop
    bool setStepMode(const u_char id);

    bool setTorque(const u_char id, const bool on);
    bool calbrationOffset(const u_char id);

    bool setMaxAngleLimit(const u_char id, const int16_t angle);
    bool setMinAngleLimit(const u_char id, const int16_t angle);

    int reg_write_action(const u_char id);

    int16_t readWord(const u_char, const u_char);

    int16_t readPos(const u_char id);
    int16_t readSpd(const u_char id);

    bool lockEprom(const u_char id);
    bool unlockEprom(const u_char id);

  private:
    const rclcpp::Logger getLogger() noexcept;
    // u_char calcChecksum(const u_char *const buf, const size_t len) noexcept;

    void read_flush();
    void write_flush();
    int ask(const u_char id);
    int checkHead();
    int read(const u_char id, const u_char mem_adder, u_char *n_data, u_char n_len);
    bool writeByte(const u_char id, const u_char mem_adder, const u_char data);
    bool writeWord(const u_char id, const u_char mem_adder, const uint16_t data);
    bool genWrite(const u_char id,
                  const u_char mem_adder,
                  const u_char *const param,
                  const u_char param_len);
    bool writeBuf(
        const u_char id, const u_char mem_adder, const u_char *const param,
        const u_int8_t param_len, const u_char function
    );
    //
    bool syncWrite(const u_char *ID, const u_char IDN, const u_char MemAddr, u_char *nDat, const u_char nLen);
    //
    void host2STS(u_char *const data_l, u_char *const data_h, const u_short data);
    //
    int STS2host(const u_char data_l, const u_char data_h);
  };
} // namespace feetech_sts_interface
