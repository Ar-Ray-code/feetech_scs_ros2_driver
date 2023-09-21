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

#include "feetech_sts_interface/packet_handler.hpp"
#include "feetech_sts_interface/INST.h"
#include "feetech_sts_interface/SMS_STS.h"

namespace feetech_sts_interface
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

    u_char PacketHandler::calcChecksum(const u_char *const buf, const size_t len) noexcept
    {
        u_char checksum = 0;
        for (size_t i = 2; i < len - 1; ++i)
        {
            checksum += buf[i];
        }
        return ~checksum;
    }

    void PacketHandler::host2STS(u_char *const data_l, u_char *const data_h, const u_short data)
    {
        bool END = 1; // STS
        if (END)
        {
            *data_l = (data >> 8);
            *data_h = (data & 0xff);
        }
        else
        {
            *data_h = (data >> 8);
            *data_l = (data & 0xff);
        }
    }

    int PacketHandler::STS2host(const u_char data_l, const u_char data_h)
    {
        bool END = 1; // STS
        if (END)
        {
            return int((data_l << 8) + data_h);
        }
        else
        {
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
        write_buf[3] = 2 + length; // Message length
        write_buf[4] = INST_READ;
        write_buf[5] = mem_adder;
        write_buf[6] = length;
        write_buf[7] = calcChecksum(write_buf, write_buf_size);

        std::string sent = "";
        for (long int i = 0; i < write_buf_size; i++)
        {
            sent += std::to_string(write_buf[i]) + " ";
        }

        const ssize_t write_ret = port_handler_->write(
            reinterpret_cast<char *>(write_buf), write_buf_size);
        if (write_ret == -1)
        {
            return -1;
        }

        using namespace std::chrono_literals; // NOLINT
        const auto clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        const auto started = clock_->now();
        // while (port_handler_->getBytesAvailable() == 0) {
        //   if (clock_->now() - started > 1s) {
        //     std::cout << "timeout" << std::endl;
        //     return -1;
        //   }
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        char read_buf[128];
        const ssize_t read_ret = port_handler_->read(read_buf, sizeof(read_buf));
        if (read_ret == -1)
        {
            return -1;
        }
        std::string recv = "";
        for (long int i = 0; i < read_ret; i++)
        {
            recv += std::to_string(read_buf[i]) + " ";
        }
        return STS2host(read_buf[5], read_buf[6]);
    }

    bool PacketHandler::writeBuf(
        const u_char id, const u_char mem_adder, const u_char *const param,
        const u_int8_t param_len, const u_char function)
    {
        const int length = param_len + 7;
        u_char write_buf[length];
        write_buf[0] = 0xFF;
        write_buf[1] = 0xFF;

        write_buf[2] = id;
        write_buf[3] = 3 + param_len; // Message length

        write_buf[4] = function;
        write_buf[5] = mem_adder;
        for (size_t i = 0; i < param_len; ++i)
        {
            write_buf[6 + i] = param[i];
        }
        write_buf[6 + param_len] = calcChecksum(write_buf, sizeof(write_buf));

        std::string sent = "";
        for (size_t i = 0; i < sizeof(write_buf); ++i)
        {
            sent += std::to_string(write_buf[i]) + " ";
        }
        RCLCPP_DEBUG(
            this->getLogger(),
            "Write[%zu]: %s", sizeof(write_buf), sent.c_str());

        const ssize_t write_ret = this->port_handler_->write(
            reinterpret_cast<char *>(write_buf), sizeof(write_buf));
        RCLCPP_DEBUG(
            this->getLogger(),
            "Return: %zd", write_ret);
        if (write_ret == -1)
        {
            return false;
        }
        return true;
    }

    bool PacketHandler::setWheelMode(const u_char id)
    {
        u_char bBuf[2];
        host2STS(bBuf + 0, bBuf + 1, 1);
        return writeBuf(id, SMS_STS_MODE, bBuf, 2, INST_WRITE);
    }

    bool PacketHandler::ping(int id)
    {
        return writeBuf(id, 0, nullptr, 0, INST_PING);
    }

    bool PacketHandler::writePosEx(const u_char id, const u_short position, const u_short speed, const u_short acc)
    {
        u_short pos = position;
        if (pos < 0)
        {
            pos = -pos;
            pos |= (1 << 15);
        }
        u_char bBuf[7];
        bBuf[0] = acc;
        host2STS(bBuf + 1, bBuf + 2, pos);
        host2STS(bBuf + 3, bBuf + 4, 0);
        host2STS(bBuf + 4, bBuf + 6, speed);

        return writeBuf(id, SMS_STS_ACC, bBuf, 7, INST_WRITE);
    }

    bool PacketHandler::writeSpd(
        const u_char id, const int16_t speed, const u_char acc)
    {
        int16_t speed_pwm = speed;
        if (speed_pwm < 0)
        {
            speed_pwm = -speed_pwm;
            speed_pwm |= (1 << 15);
        }
        u_char bBuf[2];
        host2STS(bBuf + 0, bBuf + 1, speed_pwm);

        return writeBuf(id, SMS_STS_GOAL_SPEED_L, bBuf, 2, INST_WRITE);
    }

    int16_t PacketHandler::readPos(const u_char id)
    {
        auto ret = this->readBuf(id, SMS_STS_PRESENT_POSITION_L);
        if (ret == -1)
        {
            return -1;
        }
        int16_t position = ret;
        if (ret && (position & (1 << 15)))
        {
            position = -(position & ~(1 << 15));
        }

        return position;
    }

    int16_t PacketHandler::readSpd(const u_char id)
    {
        auto ret = this->readBuf(id, SMS_STS_PRESENT_SPEED_L);
        if (ret == -1)
        {
            return -1;
        }
        int16_t speed = ret;
        if (ret && (speed & (1 << 15)))
        {
            speed = -(speed & ~(1 << 15));
        }
        return speed;
    }

    // TODO: Fix bug to use syncWrite
    bool PacketHandler::syncWrite(const u_char *ID, const u_char IDN, const u_char MemAddr, u_char *nDat, const u_char nLen)
    {
        u_char mesLen = ((nLen + 1) * IDN + 4);
        u_char Sum = 0;
        u_char bBuf[mesLen + 4];
        bBuf[0] = 0xff;
        bBuf[1] = 0xff;
        bBuf[2] = 0xfe;
        bBuf[3] = mesLen;
        bBuf[4] = INST_SYNC_WRITE;
        bBuf[5] = MemAddr;
        bBuf[6] = nLen;

        Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

        u_char i, j;
        for (i = 0; i < IDN; i++)
        {
            bBuf[7 + i * (nLen + 1)] = ID[i];
            bBuf[7 + i * (nLen + 1) + 1] = nDat[i * nLen];
            Sum += ID[i];
            for (j = 0; j < nLen; j++)
            {
                Sum += nDat[i * nLen + j];
            }
        }

        bBuf[7 + (nLen + 1) * IDN] = static_cast<u_char>(~Sum);
        this->port_handler_->write(reinterpret_cast<char *>(bBuf), mesLen + 4);

        return true;
    }

    bool PacketHandler::syncWritePosEx(const u_char *ID, const u_char IDN, const u_short *Position, const u_short *Speed, const u_short *ACC)
    {
        u_char offbuf[7 * IDN];
        for (u_char i = 0; i < IDN; i++)
        {
            u_short pos = Position[i];
            // if (pos < 0)
            // {
            //     pos = -pos;
            //     pos |= (1 << 15);
            // }
            u_short V;
            if (Speed)
            {
                V = Speed[i];
            }
            else
            {
                V = 0;
            }
            if (ACC)
            {
                offbuf[i * 7] = ACC[i];
            }
            else
            {
                offbuf[i * 7] = 0;
            }
            host2STS(offbuf + i * 7 + 1, offbuf + i * 7 + 3, pos);
            host2STS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
            host2STS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
        }
        return syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 6);
    }

    bool PacketHandler::setTorque(const u_char id, const bool onoff)
    {
        u_char bBuf[2];
        host2STS(bBuf + 0, bBuf + 1, onoff ? 1 : 0);
        return writeBuf(id, SMS_STS_TORQUE_ENABLE, bBuf, 2, INST_WRITE);
    }

    const rclcpp::Logger PacketHandler::getLogger() noexcept
    {
        return this->logger_;
    }

} // namespace feetech_sts_interface
