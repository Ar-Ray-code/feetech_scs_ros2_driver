#pragma once
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <feetech_sts_interface/feetech_sts_interface.hpp>

const double PI = 3.14159265359;
const double BASELINE = 17.5;                             // between wheel to wheel [cm]
const double WHEEL_R = 3.0;                               // wheel radius[cm]
const double STEP_RATIO = STS3032_ANGLE_360 / (2.0 * PI); // cmとstepの変換係数
const double linear_coef = 1.0 / WHEEL_R;                 // [1/cm]
const double angular_coef_r = BASELINE / (2.0 * WHEEL_R);
const double angular_coef_l = -angular_coef_r;
const u_char ACC = 254;      // 加速度 step/sec^2. MAX 254
const int16_t MAX_SPEED = 3000; // 最大速度 step/sec  -> 3000/STS3032_ANGLE_360 * (2 * WHEEL_R * PI) = 13.8cm/sec

struct wheel_speed
{
  int16_t right;
  int16_t left;
};

inline short cm_sec_to_step_sec(double cm_sec)
{
  return cm_sec * STEP_RATIO;
}

wheel_speed convert_to_speed(geometry_msgs::msg::Twist cmd_vel);
void move_cmd(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler, const geometry_msgs::msg::Twist &cmd_vel);
void stop(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler);
void set_wheel_mode(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler);
void print_wheel_speed(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler);

int main(int, char **);
