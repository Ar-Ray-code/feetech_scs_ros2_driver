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

int main(int argc, char ** argv)
{
  int TARGET_ID = 1;

  rclcpp::init(argc, argv);
  using namespace std::chrono_literals;    // NOLINT
  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>("/dev/ttyUSB0", 1000000);
  auto packet_handler = std::make_shared<feetech_scs_interface::PacketHandler>(port_handler);

  if (!port_handler->openPort()) {
    return EXIT_FAILURE;
  }

  bool set_pwm_mode = packet_handler->setPWMMode(1);
  std::cout << "set_pwm_mode: " << set_pwm_mode << std::endl;
  rclcpp::sleep_for(1000ms);

  packet_handler->writeSpd(TARGET_ID, 500);
  for (int i = 0; i < 100; i++) {
    std::cout << "pos: " << packet_handler->readPos(1) << std::endl;
    rclcpp::sleep_for(10ms);
  }
  packet_handler->writeSpd(TARGET_ID, 0);
  for (int i = 0; i < 100; i++) {
    std::cout << "pos: " << packet_handler->readPos(1) << std::endl;
    rclcpp::sleep_for(10ms);
  }
  // rev
  packet_handler->writeSpd(TARGET_ID, -500);
  for (int i = 0; i < 100; i++) {
    std::cout << "pos: " << packet_handler->readPos(1) << std::endl;
    rclcpp::sleep_for(10ms);
  }
  packet_handler->writeSpd(TARGET_ID, 0);
  for (int i = 0; i < 100; i++) {
    std::cout << "pos: " << packet_handler->readPos(1) << std::endl;
    rclcpp::sleep_for(10ms);
  }


  port_handler->closePort();
  return EXIT_SUCCESS;
}
