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
#include <feetech_scs_interface/feetech_scs_interface.hpp>
#include <unistd.h>

using namespace feetech_scs_interface;

int main(int argc, char ** argv)
{
  int TARGET_ID = 1;
  int baudrate = 1000000;
  std::string port_name = "/dev/ttyUSB0";

  rclcpp::init(argc, argv);
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }

  packet_handler->setTorque(TARGET_ID, 0);

  using namespace std::chrono_literals;  // NOLINT
  for (int i = 0; i < 100; i++)
  {
    std::cout << "pos: " << feetech_scs_interface::SCS0009::data2angle(packet_handler->readSpd(TARGET_ID)) << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  port_handler->close();
  return EXIT_SUCCESS;
}
