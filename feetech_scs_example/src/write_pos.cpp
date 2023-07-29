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

#include <feetech_scs_interface/feetech_scs_interface.hpp>

void set_target_pose(std::shared_ptr<feetech_scs_interface::PacketHandler> packet_handler,
  int target_id, int position, int speed, unsigned int acceleration=50)
{
  int current_pos = -1;
  while (current_pos < 0) {
    current_pos = packet_handler->readPos(target_id);
  }
  if (acceleration <= 0) {
    acceleration = 50;
  }

  int target_angle = feetech_scs_interface::SCS0009::angle2data(position);

  packet_handler->writePos(target_id, target_angle, acceleration, speed);
  int sleep_time = int((fabs((float)current_pos - (float)target_angle) / (float)speed) * 1000 + ((float)speed / ((float)acceleration * 100)) * 1000);

  for (int i = 0; i < 10; i++) {
    std::cout << "pos: " << feetech_scs_interface::SCS0009::data2angle(packet_handler->readPos(target_id)) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time / 10));
  }
}

int main(int argc, char ** argv)
{
  int TARGET_ID = 2;
  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 1000000;

  rclcpp::init(argc, argv);
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    return EXIT_FAILURE;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  packet_handler->setTorque(TARGET_ID, 1);

  set_target_pose(packet_handler, TARGET_ID, 0, 1000);
  set_target_pose(packet_handler, TARGET_ID, 90, 1000);
  set_target_pose(packet_handler, TARGET_ID, 180, 1000);
  set_target_pose(packet_handler, TARGET_ID, 270, 1000);
  set_target_pose(packet_handler, TARGET_ID, 0, 1000);

  packet_handler->setTorque(TARGET_ID, 0);
  port_handler->close();
  return EXIT_SUCCESS;
}
