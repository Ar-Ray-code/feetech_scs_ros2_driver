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

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <unistd.h>
#include <iomanip>

using namespace feetech_sts_interface;

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cout << "Usage: " << argv[0] << " <id (0)> <port_name (/dev/ttyUSB0)> <baudrate (1000000)>" << std::endl;
    return EXIT_FAILURE;
  }
  int TARGET_ID = std::stoi(argv[1]);
  std::string port_name = argv[2];
  int baudrate = std::stoi(argv[3]);

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }

  packet_handler->setTorque(TARGET_ID, 0);

  using namespace std::chrono_literals;  // NOLINT
  while (true)
  {
    float pos_angle = feetech_sts_interface::STS3032::data2angle(packet_handler->readPos(TARGET_ID));
    std::cout << "pos: ";
    std::cout << std::fixed << std::setprecision(3) << pos_angle;
    std::cout << " deg" << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  port_handler->close();
  return EXIT_FAILURE;
}
