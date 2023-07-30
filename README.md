# feetech_scs_ros2_driver

ROS2 Driver for FeeTech-SCS

Hardware Interface : TODO

<br>

## Support 📜⚙️

### ROS2 Distro 🐢

| ROS2 Distro | Support |
| --- | --- |
| Humble        | ✔️      |

### FeeTech-SCS ⚙️

https://pages.switch-science.com/comparison/feetech-servos#serial

| Product Name | Support |
| --- | --- |
| SCS0009 | ✔️ |
| SCS215 | |
| SCS115 | |
| SCS20-360T | |
| SCS15 | |

### FeeTech-SCS

STS will be supported in the future.

<br>

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/HarvestX/h6x_serial_interface.git -b humble
git clone https://github.com/Ar-Ray-code/feetech_scs_ros2_driver -b main
cd ../

colcon build
```

## Usage (Example)

Run `ros2 run <target_exec> <ID> <port> <baudrate>` to execute the example.

### Write Position

```bash
ros2 run feetech_scs_example write_pos 1 /dev/ttyUSB0 1000000
```

### Read position

```bash
ros2 run feetech_scs_example read_pos 1 /dev/ttyUSB0 1000000
```

### Write speed

```bash
ros2 run feetech_scs_example write_spd 1 /dev/ttyUSB0 1000000
```

### Read pwm

```bash
ros2 run feetech_scs_example read_spd 1 /dev/ttyUSB0 1000000
```

