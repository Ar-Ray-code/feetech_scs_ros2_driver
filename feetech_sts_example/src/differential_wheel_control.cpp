#include <feetech_sts_example/differential_wheel_control.hpp>

#define max(a, b) ((a < b) ? b : a)
#define min(a, b) ((a < b) ? a : b)
#define clip(a, max_val, min_val) (max(min(a, max_val), min_val))

wheel_speed convert_to_speed(geometry_msgs::msg::Twist cmd_vel)
{
  wheel_speed ret = {};
  ret.right = cm_sec_to_step_sec(linear_coef * cmd_vel.linear.x + angular_coef_r * cmd_vel.angular.z);
  ret.left = -cm_sec_to_step_sec(linear_coef * cmd_vel.linear.x + angular_coef_l * cmd_vel.angular.z);
  ret.right = clip(ret.right, MAX_SPEED, -MAX_SPEED);
  ret.left = clip(ret.left, MAX_SPEED, -MAX_SPEED);
  return ret;
}

void move_cmd(const u_char id_right, const u_char id_left,
              std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler,
              const geometry_msgs::msg::Twist &cmd_vel)
{
  wheel_speed speed = convert_to_speed(cmd_vel);
  u_char id_list[2] = {(u_char)id_right, (u_char)id_left};
  int16_t vel_list[2] = {speed.right, speed.left};
  u_short acc_list[2] = {(u_short)ACC, (u_short)ACC};
  int16_t goal_list[2] = {+1000, -1000,};
  packet_handler->syncWritePosEx(id_list, sizeof(id_list), goal_list, vel_list, acc_list);
}

void stop(
  const u_char id_right, const u_char id_left,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  move_cmd(id_right, id_left, packet_handler, geometry_msgs::msg::Twist());
}

void set_wheel_mode(
  const u_char id_right, const u_char id_left,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  packet_handler->unlockEprom(id_right);
  // packet_handler->setOpenLoopWheelMode(id_right);
  packet_handler->setWheelMode(id_right);
  // packet_handler->setStepMode(id_right);
  packet_handler->setTorque(id_right, true);
  // packet_handler->setTorque(id_right, false);
  // packet_handler->setMaxAngleLimit(id_right, 0);
  packet_handler->calbrationOffset(id_right);
  packet_handler->lockEprom(id_right);

  packet_handler->unlockEprom(id_left);
  // packet_handler->setOpenLoopWheelMode(id_left);
  packet_handler->setWheelMode(id_left);
  // packet_handler->setStepMode(id_left);
  packet_handler->setTorque(id_left, true);
  // packet_handler->setTorque(id_left, false);
  // packet_handler->setMaxAngleLimit(id_left, 0);
  packet_handler->calbrationOffset(id_left);
  packet_handler->lockEprom(id_left);
}

/// @brief Wheel Speedを表示する.高頻度に呼び出すと落ちるので注意。
/// @param id_right Right wheel motorのID
/// @param id_left Left wheel motorのID
/// @param packet_handler PacketHandlerのインスタンス
void print_wheel_speed(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  auto speed_right = feetech_sts_interface::STS3032::data2angle(packet_handler->readSpd(id_right));
  auto speed_left = feetech_sts_interface::STS3032::data2angle(packet_handler->readSpd(id_left));

  auto pos_right = packet_handler->readPos(id_right);
  auto pos_left = packet_handler->readPos(id_left);
  std::cout << "wheel speed right/ left: " << speed_right << " / " << speed_left << " [deg/sec]" << std::endl;
  std::cout << "pos right/ left: " << pos_right << " / " << pos_left << std::endl;
}

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cout << "Usage: " << argv[0] << " <id right> <id left> <port_name (/dev/ttyUSB0)> <baudrate (1000000)>" << std::endl;
    return EXIT_FAILURE;
  }
  const int RIGHT_ID = std::atoi(argv[1]);
  const int LEFT_ID = std::atoi(argv[2]);
  std::string port_name = argv[3];
  const int baudrate = std::stoi(argv[4]);
  std::cout << "RIGHT_ID: " << RIGHT_ID << std::endl;
  std::cout << "LEFT_ID: " << LEFT_ID << std::endl;
  std::cout << "port_name: " << port_name << std::endl;
  std::cout << "baudrate: " << baudrate << std::endl;

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open())
  {
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "set wheel mode." << std::endl;
  set_wheel_mode(RIGHT_ID, LEFT_ID, packet_handler);
  std::cout << "done." << std::endl;

  geometry_msgs::msg::Twist cmd_vel;

  std::cout << "MOVE FORWARD." << std::endl;
  cmd_vel.linear.x = 2.0;  // [cm/sec]
  cmd_vel.angular.z = 0.0; // [rad/sec]
  move_cmd(RIGHT_ID, LEFT_ID, packet_handler, cmd_vel);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::cout << "STOP." << std::endl;
  stop(RIGHT_ID, LEFT_ID, packet_handler);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  port_handler->close();
  std::cout << "shutdown." << std::endl;
  return EXIT_SUCCESS;
}