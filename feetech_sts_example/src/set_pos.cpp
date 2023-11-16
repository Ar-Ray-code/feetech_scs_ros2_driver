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

void set_target_pose(std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler,
  int target_id, int position, int speed, unsigned int acceleration=50)
{
  int current_pos = -1;
  while (current_pos < 0) {
    current_pos = packet_handler->readPos(target_id);
  }
  if (acceleration <= 0) {
    acceleration = 50;
  }

  int target_angle = feetech_sts_interface::STS3032::angle2data(position);

  packet_handler->writePosEx(target_id, target_angle, speed, acceleration);
  // int sleep_time = int((fabs((float)current_pos - (float)target_angle) / (float)speed) * 1000 + ((float)speed / ((float)acceleration * 100)) * 1000);
  const int sleep_time = 10 * 1000;

  for (int i = 0; i < 10; i++) {
    std::cout << "pos: " << feetech_sts_interface::STS3032::data2angle(packet_handler->readPos(target_id)) << " deg" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time / 10));
  }
}

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cout << "Usage: " << argv[0] << " <id (0)> <port_name (/dev/ttyUSB0)> <baudrate (1000000)> <position (0-300)>" << std::endl;
    return EXIT_FAILURE;
  }
  int TARGET_ID = std::stoi(argv[1]);
  std::string port_name = argv[2];
  int baudrate = std::stoi(argv[3]);
  int position = std::stoi(argv[4]);

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  packet_handler->setTorque(TARGET_ID, 1);
  packet_handler->setWheelMode(TARGET_ID);

  set_target_pose(packet_handler, TARGET_ID, position, 1000);

  packet_handler->setTorque(TARGET_ID, 0);
  port_handler->close();
  return EXIT_SUCCESS;
}
