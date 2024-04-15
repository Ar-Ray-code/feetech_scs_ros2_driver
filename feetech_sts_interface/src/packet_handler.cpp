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

    // u_char PacketHandler::calcChecksum(const u_char *const buf, const size_t len) noexcept
    // {
    //     u_char checksum = 0;
    //     for (size_t i = 2; i < len - 1; ++i)
    //     {
    //         checksum += buf[i];
    //     }
    //     return ~checksum;
    // }

    void PacketHandler::host2STS(u_char *const data_l, u_char *const data_h, const u_short data)
    {
        if (END_)
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
        if (END_)
        {
            return int((data_l << 8) + data_h);
        }
        else
        {
            return int((data_h << 8) + data_l);
        }
    }

    int16_t PacketHandler::readWord(
        const u_char id, const u_char mem_adder)
    {
        uint8_t buf[2] = {};
        int size = read(id, mem_adder, buf, 2);
        if (size != 2)
        {
            return -1;
        }
        return STS2host(buf[0], buf[1]);
    }

    int PacketHandler::ask(const u_char id)
    {
        int error_ = 0;
        int level_ = 1;
        if (id != 0xfe && level_)
        {
            if (!checkHead())
            {
                return 0;
            }
            uint8_t b_buf[4];
            if (this->port_handler_->read(reinterpret_cast<char *>(&b_buf), 4) != 4)
            {
                return 0;
            }
            if (b_buf[0] != id)
            {
                return 0;
            }
            if (b_buf[1] != 2)
            {
                return 0;
            }
            uint8_t cal_sum = ~(b_buf[0] + b_buf[1] + b_buf[2]);
            if (cal_sum != b_buf[3])
            {
                return 0;
            }
            error_ = b_buf[2];
        }
        return 1;
    }

    int PacketHandler::checkHead()
    {
        uint8_t b_dat;
        uint8_t b_buf[2] = {0, 0};
        uint8_t cnt = 0;
        while (1)
        {
            if (!port_handler_->read(reinterpret_cast<char *>(&b_dat), 1))
            {
                return 0;
            }
            b_buf[1] = b_buf[0];
            b_buf[0] = b_dat;
            if (b_buf[0] == 0xff && b_buf[1] == 0xff)
            {
                break;
            }
            cnt++;
            if (cnt > 10)
            {
                return 0;
            }
        }
        return 1;
    }

    int PacketHandler::read(const u_char id, const u_char mem_adder, u_char *n_data, u_char n_len)
    {
        read_flush();
        writeBuf(id, mem_adder, &n_len, 1, INST_READ);
        write_flush();

        if (!checkHead())
        {
            return 0;
        }
        u_char buf[4];
        if (port_handler_->read(reinterpret_cast<char *>(buf), 3) != 3)
        {
            return 0;
        }
        int size = port_handler_->read(reinterpret_cast<char *>(n_data), n_len);
        if (size != n_len)
        {
            return 0;
        }
        if (port_handler_->read(reinterpret_cast<char *>(buf + 3), 1) != 1)
        {
            return 0;
        }
        u_char cal_sum = buf[0] + buf[1] + buf[2];
        for (u_char i = 0; i < size; ++i)
        {
            cal_sum += n_data[i];
        }
        cal_sum = ~cal_sum;
        if (cal_sum != buf[3])
        {
            return 0;
        }
        return size;
    }

    bool PacketHandler::writeByte(const u_char id, const u_char mem_adder, const u_char data)
    {
        read_flush();
        bool ret = writeBuf(id, mem_adder, &data, 1, INST_WRITE);
        write_flush();
        return ret;
    }

    bool PacketHandler::writeWord(const u_char id, const u_char mem_adder, const uint16_t data)
    {
        uint8_t buf[2] = {};
        host2STS(buf + 0, buf + 1, data);
        read_flush();
        bool ret = writeBuf(id, mem_adder, buf, sizeof(buf), INST_WRITE);
        write_flush();
        return ret;
    }

    bool PacketHandler::writeBuf(
        const u_char id, const u_char mem_adder, const u_char *const param,
        const u_int8_t param_len, const u_char function)
    {
        u_char write_buf[7 + param_len];
        u_char check_sum = 0;
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
        check_sum = id + (3 + param_len) + function + mem_adder;
        for (size_t i = 0; i < param_len; ++i)
        {
            check_sum += param[i];
        }
        write_buf[6 + param_len] = ~check_sum;

        auto ret = port_handler_->write(reinterpret_cast<char *>(write_buf), sizeof(write_buf));
        if (ret == -1)
        {
            return false;
        }
        return true;
    }

    bool PacketHandler::genWrite(
        const u_char id, const u_char mem_adder,
        const u_char *const param, const u_int8_t param_len)
    {
        read_flush();
        writeBuf(id, mem_adder, param, param_len, INST_REG_WRITE);
        write_flush();
        return ask(id);
    }

    bool PacketHandler::setRotationMode(const u_char id)
    {
        return writeByte(id, SMS_STS_MODE, 0);
    }
    bool PacketHandler::setWheelMode(const u_char id)
    {
        return writeByte(id, SMS_STS_MODE, 1);
    }
    bool PacketHandler::setOpenLoopWheelMode(const u_char id)
    {
        return writeByte(id, SMS_STS_MODE, 2);
    }
    bool PacketHandler::setStepMode(const u_char id)
    {
        return writeByte(id, SMS_STS_MODE, 3);
    }

    bool PacketHandler::ping(int id)
    {
        read_flush();
        auto ret = writeBuf(id, 0, nullptr, 0, INST_PING);
        write_flush();

        int error_ = 0;
        if (!checkHead())
        {
            return -1;
        }
        uint8_t b_buf[4];
        if (this->port_handler_->read(reinterpret_cast<char *>(b_buf), 4) != 4)
        {
            return -1;
        }
        if (b_buf[0] != id && id != 0xfe)
        {
            return -1;
        }
        if (b_buf[1] != 2)
        {
            return -1;
        }
        uint8_t cal_sum = ~(b_buf[0] + b_buf[1] + b_buf[2]);
        if (cal_sum != b_buf[3])
        {
            return -1;
        }
        error_ = b_buf[2];
        return ret;
    }

    bool PacketHandler::writePos(const u_char id, const u_short position, const u_short time, const u_short speed)
    {
        u_char bBuf[6];
        host2STS(bBuf + 0, bBuf + 1, position);
        host2STS(bBuf + 2, bBuf + 3, time);
        host2STS(bBuf + 4, bBuf + 5, speed);
        // return genWrite(id, SMS_STS_GOAL_POSITION_L, bBuf, sizeof(bBuf));
        return writeBuf(id, SMS_STS_GOAL_POSITION_L, bBuf, sizeof(bBuf), INST_WRITE);
    }

    bool PacketHandler::writePosEx(const u_char id, const int16_t position, const int16_t speed, const u_short acc)
    {
        int16_t pos = position;
        if (pos < 0)
        {
            pos = -pos;
            pos |= (1 << 15);
        }
        int16_t V = speed;
        if (V < 0)
        {
            V = -V;
            V |= (1 << 15);
        }
        u_char bBuf[7];
        bBuf[0] = acc;                       // SMS_STS_ACC 41
        host2STS(bBuf + 1, bBuf + 2, pos);   // SMS_STS_GOAL_POSITION 42, 43
        host2STS(bBuf + 3, bBuf + 4, 0);     // SMS_STS_GOAL_TIME 44, 45
        host2STS(bBuf + 4, bBuf + 6, V);     // SMS_STS_GOAL_SPEED 46, 47
        return genWrite(id, SMS_STS_ACC, bBuf, sizeof(bBuf));
    }

    // bool PacketHandler::writeSpd(
    //     const u_char id, const int16_t speed, const u_short acc)
    // {
    //     int16_t speed_pwm = speed;
    //     if (speed_pwm < 0)
    //     {
    //         speed_pwm = -speed_pwm;
    //         speed_pwm |= (1 << 15);
    //     }
    //     u_char bBuf[2];
    //     bBuf[0] = acc;
    //     writeBuf(id, SMS_STS_ACC, bBuf, 1, INST_WRITE);
    //     host2STS(bBuf + 0, bBuf + 1, speed_pwm);
    //     return genWrite(id, SMS_STS_GOAL_SPEED_L, bBuf, sizeof(bBuf));
    // }

    // bool PacketHandler::writeTime(const u_char id, const int16_t time)
    // {
    //     int16_t t = time;
    //     if (t < 0)
    //     {
    //         t = -t;
    //         t |= (1 << 10);
    //     }
    //     u_char bBuf[7];
    //     bBuf[0] = 0;
    //     host2STS(bBuf + 1, bBuf + 2, 0);
    //     host2STS(bBuf + 3, bBuf + 4, t);  // SMS_STS_GOAL_TIME 44, 45
    //     host2STS(bBuf + 4, bBuf + 6, 0);
    //     return genWrite(id, SMS_STS_ACC, bBuf, sizeof(bBuf));
    // }

    bool PacketHandler::syncWritePosEx(const u_char *ID, const u_char IDN, const int16_t *Position, const int16_t *Speed, const u_short *ACC)
    {
        u_char offbuf[7 * IDN];
        for (u_char i = 0; i < IDN; ++i)
        {
            int16_t pos = Position[i];
            if (pos < 0)
            {
                pos = -pos;
                pos |= (1 << 15);
            }
            int16_t V;
            if (Speed)
            {
                V = Speed[i];
                if (V < 0)
                {
                    V = -V;
                    V |= (1 << 15);
                }
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
            // acc, pos_l, pos_h, 0, 0, vel_l, vel_h
            host2STS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, pos);
            host2STS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
            host2STS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
        }
        return syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
    }

    bool PacketHandler::syncWriteSpd(const u_char *ID, const u_char IDN, const int16_t *Speed, const u_short *ACC)
    {
        u_char offbuf[7 * IDN];
        for (u_char i = 0; i < IDN; ++i)
        {
            int16_t V;
            if (Speed)
            {
                V = Speed[i];
                if (V < 0)
                {
                    V = -V;
                    V |= (1 << 15);
                }
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
            // acc, pos_l, pos_h, time_l, time_h, vel_l, vel_h
            host2STS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, 0);
            host2STS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0);
            host2STS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V);
        }
        return syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
    }

    bool PacketHandler::syncWriteTime(const u_char *ID, const u_char IDN, const int16_t *Time)
    {
        u_char offbuf[7 * IDN];
        for (u_char i = 0; i < IDN; ++i)
        {
            int16_t T;
            if (Time)
            {
                T = Time[i];
                if (T < 0)
                {
                    T = -T;
                    // T |= (1 << 15);
                    T |= (1 << 10);
                }
            }
            else
            {
                T = 0;
            }
            offbuf[i * 7] = 0;
            // acc, pos_l, pos_h, time_l, time_h, vel_l, vel_h
            host2STS(offbuf + i * 7 + 1, offbuf + i * 7 + 2, 0);
            host2STS(offbuf + i * 7 + 3, offbuf + i * 7 + 4, T);
            host2STS(offbuf + i * 7 + 5, offbuf + i * 7 + 6, 0);
        }
        return syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);

    }

    int16_t PacketHandler::readPos(const u_char id)
    {
        int err = 0;
        int16_t pos = -1;
        pos = this->readWord(id, SMS_STS_PRESENT_POSITION_L);
        if (pos == -1)
        {
            err = 1;
        }
        if (!err && (pos & (1 << 15)))
        {
            pos = -(pos & ~(1 << 15));
        }
        return pos;
    }

    int16_t PacketHandler::readSpd(const u_char id)
    {
        int err = 0;
        int16_t speed = -1;
        speed = this->readWord(id, SMS_STS_PRESENT_SPEED_L);
        if (speed == -1)
        {
            err = 1;
            return -1;
        }
        if (!err && (speed & (1 << 15)))
        {
            speed = -(speed & ~(1 << 15));
        }
        return speed;
    }

    bool PacketHandler::syncWrite(const u_char *ID, const u_char IDN, const u_char MemAddr, u_char *nDat, const u_char nLen)
    {
        this->read_flush();

        u_char mesLen = ((nLen + 1) * IDN + 4);
        u_char Sum = 0;
        u_char bBuf[7] = {0};
        bBuf[0] = 0xff;
        bBuf[1] = 0xff;
        bBuf[2] = 0xfe;
        bBuf[3] = mesLen;
        bBuf[4] = INST_SYNC_WRITE;
        bBuf[5] = MemAddr;
        bBuf[6] = nLen;
        this->port_handler_->write(reinterpret_cast<char *>(bBuf), sizeof(bBuf));

        Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;

        for (size_t i = 0; i < IDN; ++i)
        {
            this->port_handler_->write(reinterpret_cast<const char *>(&ID[i]), 1);
            this->port_handler_->write(reinterpret_cast<const char *>(nDat + i * nLen), nLen);
            // std::cout << "write ID: " << (int)ID[i] << std::endl;
            // std::cout << "write nDat: ";
            // for (size_t j = 0; j < nLen; ++j)
            // {
            //     std::cout << (int)nDat[i * nLen + j] << ", ";
            // }
            // std::cout << std::endl;
            Sum += ID[i];
            for (size_t j = 0; j < nLen; ++j)
            {
                Sum += nDat[i * nLen + j];
            }
        }
        char s = ~Sum;
        this->port_handler_->write(&s, 1);
        write_flush();
        return true;
    }

    bool PacketHandler::setTorque(const u_char id, const bool on)
    {
        return writeByte(id, SMS_STS_TORQUE_ENABLE, on ? 1 : 0);
    }

    bool PacketHandler::calbrationOffset(const u_char id)
    {
        return writeByte(id, SMS_STS_TORQUE_ENABLE, 128);
    }

    bool PacketHandler::setMaxAngleLimit(const u_char id, const int16_t angle)
    {
        uint8_t buf[2] = {};
        if(angle < 0)
        {
            host2STS(buf, buf + 1, 0);
        }
        else if(angle > 4095)
        {
            host2STS(buf, buf + 1, 4095);
        }
        else
        {
            host2STS(buf, buf + 1, angle);
        }
        return genWrite(id, SMS_STS_MAX_ANGLE_LIMIT_L, buf, sizeof(buf));
    }

    bool PacketHandler::setMinAngleLimit(const u_char id, const int16_t angle)
    {
        uint8_t buf[2] = {};
        if(angle < 0)
        {
            host2STS(buf, buf + 1, 0);
        }
        else if(angle > 4095)
        {
            host2STS(buf, buf + 1, 4095);
        }
        else
        {
            host2STS(buf, buf + 1, angle);
        }
        return genWrite(id, SMS_STS_MIN_ANGLE_LIMIT_L, buf, sizeof(buf));
    }

    bool PacketHandler::lockEprom(const u_char id)
    {
        return writeByte(id, SMS_STS_LOCK, 1);
    }

    bool PacketHandler::unlockEprom(const u_char id)
    {
        return writeByte(id, SMS_STS_LOCK, 0);
    }

    int PacketHandler::reg_write_action(const u_char id)
    {
        read_flush();
        bool ret = writeBuf(id, 0, nullptr, 0, INST_REG_ACTION);
        write_flush();
        return ask(id);
    }

    const rclcpp::Logger PacketHandler::getLogger() noexcept
    {
        return this->logger_;
    }

    void PacketHandler::read_flush() {}
    void PacketHandler::write_flush() {}

} // namespace feetech_sts_interface
