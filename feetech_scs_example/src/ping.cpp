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

#include <feetech_scs_example/ping.hpp>

int main(int argc, char ** argv)
{
  int TARGET_ID = 1;

  rclcpp::init(argc, argv);
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>("/dev/ttyUSB0");
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler);

  port_handler->configure(1000000);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }
  std::cout << "open success" << std::endl;

  using namespace std::chrono_literals;  // NOLINT
  for (int i = 0; i < 3; ++i) {
    if (packet_handler->ping(TARGET_ID)) {
      std::cout << "ping success" << std::endl;
    } else {
      std::cout << "ping fail" << std::endl;
    }
    rclcpp::sleep_for(1s);
  }

  port_handler->close();
  return EXIT_SUCCESS;
}
