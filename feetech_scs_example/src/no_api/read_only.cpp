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

#include <rclcpp/rclcpp.hpp>
#include <feetech_scs_interface/feetech_scs_interface.hpp>

int SCS2host(const u_char data_l, const u_char data_h)
{
  bool END = 0; // SCS
  if (END) {
    return (data_l << 8) + data_h;
  } else {
    return (data_h << 8) + data_l;
  }
}

void read(u_char * write_buf, u_int write_buf_size)
{
  auto port_handler_ = std::make_shared<h6x_serial_interface::PortHandler>("/dev/ttyUSB0");
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler_);

  port_handler_->configure(1000000);
  if (!port_handler_->open()) {
    return;
  }

  std::string sent = "";
  for (size_t i = 0; i < write_buf_size; i++) {
    sent += std::to_string(write_buf[i]) + " ";
  }

  std::cout << "Write[" << write_buf_size << "]: " << sent << std::endl;

  const ssize_t write_ret = port_handler_->write(
    reinterpret_cast<char *>(write_buf), write_buf_size);
  if (write_ret == -1) {
    return;
  }

  using namespace std::chrono_literals;  // NOLINT
  const auto clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const auto started = clock_->now();
  // while (port_handler_->getBytesAvailable() == 0) {
  //   if (clock_->now() - started > 1s) {
  //     std::cout << "timeout" << std::endl;
  //     return;
  //   }
  // }

  char read_buf[128];
  const ssize_t read_ret = port_handler_->read(read_buf, sizeof(read_buf));
  if (read_ret == -1) {
    return;
  }
  std::string recv = "";
  for (long int i = 0; i < read_ret; i++) {
    recv += std::to_string(read_buf[i]) + " ";
  }
  std::cout << "Read[" << read_ret << "]: " << recv << std::endl;

  auto data = SCS2host(read_buf[5], read_buf[6]);
  std::cout << "Angle: " << data << std::endl;

  port_handler_->close();
}

u_char gen_checksum(u_char * write_buf, u_int write_buf_size)
{
  u_char checksum = 0;
  for (size_t i = 2; i < write_buf_size - 1; i++) {
    checksum += write_buf[i];
  }
  return ~checksum;
}


int main()
{
  // u_char write_buf[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0x00};
  u_char write_buf[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0x00};
  write_buf[7] = gen_checksum(write_buf, sizeof(write_buf));

  read(write_buf, sizeof(write_buf));

  return EXIT_SUCCESS;
}
