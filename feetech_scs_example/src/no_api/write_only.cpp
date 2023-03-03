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

void write(u_char * write_buf, u_int write_buf_size)
{
  auto port_handler_ = std::make_shared<h6x_serial_interface::PortHandler>("/dev/ttyUSB0", 1000000);
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler_);

  if (!port_handler_->openPort()) {
    return;
  }

  std::string sent = "";
  for (size_t i = 0; i < write_buf_size; i++) {
    sent += std::to_string(write_buf[i]) + " ";
  }

  std::cout << "Write[" << write_buf_size << "]: " << sent << std::endl;

  const ssize_t write_ret = port_handler_->writePort(
    reinterpret_cast<char *>(write_buf), write_buf_size);
  if (write_ret == -1) {
    return;
  }

  using namespace std::chrono_literals;  // NOLINT
  const auto clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const auto started = clock_->now();
  while (port_handler_->getBytesAvailable() == 0) {
    if (clock_->now() - started > 1s) {
      std::cout << "timeout" << std::endl;
      return;
    }
  }

  char read_buf[128];
  const ssize_t read_ret = port_handler_->readPort(read_buf, sizeof(read_buf));
  if (read_ret == -1) {
    return;
  }
  std::string recv = "";
  for (long int i = 0; i < read_ret; i++) {
    recv += std::to_string(read_buf[i]) + " ";
  }
  std::cout << "Read[" << read_ret << "]: " << recv << std::endl;

  port_handler_->closePort();
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
  // ping (0xff, 0xff, 0x01, 0x02, 0x01, 0xfb)
  // write pos : FF FF 01 09 03 2A 00 00 03 E8 00 00 DD)
  // FF FF 01 0A 03 29 32 E0 2E 00 00 5F 00 csum (multi turn)

  const int data_size = 13;
  int checksum = 0;

  u_char write_buf[data_size] =
  {0xFF, 0xFF, 0x02, 0x0A, 0x03, 0x29, 0x32, 0xE0, 0x2E, 0x00, 0x00, 0x5F, 0x00};

  for (long int i = 2; i < data_size - 1; i++) {
    checksum += write_buf[i];
  }
  write_buf[data_size - 1] = ~checksum;

  write(write_buf, sizeof(write_buf));

  return EXIT_SUCCESS;
}
