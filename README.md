🤖 AMR_ROS_DIFF_BOT

https://drive.google.com/file/d/107mmFezZz4mKWtgOjHwmKOvJSWchzOxr/view?usp=sharing
<img width="1082" height="515" alt="image" src="https://github.com/user-attachments/assets/4bdd6e4f-2acf-4beb-8cda-5a0b2d1b63d6" />

https://drive.google.com/file/d/1DHJlutC4Qy_kuBZ27PgUqE3D-1o_8Lau/view?usp=sharing
<img width="1062" height="466" alt="image" src="https://github.com/user-attachments/assets/f64cd004-59cb-4645-982d-684666e2eefb" />

# 🤖 AMR_ROS_DIFF_BOT

**Autonomous Mobile Robot — ROS Noetic Differential Drive with SLAM, TEB Navigation, EKF Sensor Fusion & PLC Modbus Motor Control**

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Repository Structure](#repository-structure)
- [Installation & Setup](#installation--setup)
  - [1. Hardware & OS Setup](#1-hardware--os-setup)
  - [2. ROS Noetic Installation](#2-ros-noetic-installation)
  - [3. ROS Workspace Setup](#3-ros-workspace-setup)
  - [4. Slamware LIDAR SDK](#4-slamware-lidar-sdk)
  - [5. Navigation Stack (TEB Planner)](#5-navigation-stack-teb-planner)
  - [6. EKF Sensor Fusion](#6-ekf-sensor-fusion)
  - [7. IMU (BNO055) Setup](#7-imu-bno055-setup)
  - [8. PLC Modbus Motor Control](#8-plc-modbus-motor-control)
  - [9. TF & Debugging Tools](#9-tf--debugging-tools)
  - [10. Remote Access](#10-remote-access)
- [Configuration](#configuration)
  - [TF Frames](#tf-frames)
  - [Robot Footprint](#robot-footprint)
  - [Modbus Parameters](#modbus-parameters)
- [Running the Robot](#running-the-robot)
  - [Full Autonomous Navigation Launch](#full-autonomous-navigation-launch)
  - [Manual Joystick Control](#manual-joystick-control)
- [Node & Topic Reference](#node--topic-reference)
- [Key Source Files](#key-source-files)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This repository contains the complete ROS Noetic implementation of an Autonomous Mobile Robot (AMR) with a differential drive configuration. The system integrates:

- **Slamware RPLIDAR** for simultaneous localization and mapping (SLAM)
- **TEB (Timed Elastic Band) Local Planner** for dynamic obstacle avoidance and path following
- **Extended Kalman Filter (EKF)** for fusing wheel odometry and IMU data
- **BNO055 IMU** for orientation and inertial sensing over I2C
- **PLC Modbus TCP** for real-time motor velocity control via industrial registers
- **Custom `TwistToMotors` node** with smooth acceleration/deceleration profiles for both autonomous and manual modes

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Jetson Orin Nano                           │
│                                                                 │
│  ┌──────────┐    /scan     ┌────────────┐   /cmd_vel           │
│  │Slamware  │────────────► │  move_base │──────────────────┐   │
│  │LIDAR SDK │    /odom     │  (TEB)     │                  │   │
│  │  Node    │◄─────────────│            │                  ▼   │
│  └──────────┘              └────────────┘           ┌──────────┐│
│       │ /slamware_map                               │ twist_to ││
│       ▼                                             │ _motors  ││
│  ┌──────────┐   /imu/data  ┌────────────┐           │  node    ││
│  │ BNO055   │─────────────►│ robot_pose │           └────┬─────┘│
│  │  IMU     │              │   _ekf     │                │      │
│  │  Node    │   /odom      │            │  /odom_combined│      │
│  └──────────┘◄─────────────│            │◄───────────────┘      │
│                            └────────────┘                       │
│                                              modbus/regs_write  │
│                                         ┌──────────────────┐   │
│                                         │  plc_modbus_node │   │
│                                         │  (Modbus TCP)    │   │
│                                         └────────┬─────────┘   │
└──────────────────────────────────────────────────┼─────────────┘
                                                   │ TCP:502
                                          ┌────────▼─────────┐
                                          │   PLC / Motor    │
                                          │   Controllers    │
                                          │  (192.168.1.5)   │
                                          └──────────────────┘
```

---

## Hardware Requirements

| Component | Details |
|---|---|
| **Compute** | NVIDIA Jetson Orin Nano |
| **OS** | Ubuntu 20.04 (via JetPack SDK) |
| **LIDAR** | Slamware RPLIDAR / Slamtec robot base |
| **IMU** | Bosch BNO055 (I2C bus 7, address `0x28`) |
| **Drive** | Differential drive (2 independently driven wheels) |
| **Motor Controller** | PLC over Modbus TCP (default IP: `192.168.1.5`, port `502`) |
| **Wheel Radius** | 0.16 m |
| **Wheel Base Width** | 0.38 m |
| **Robot Footprint** | 0.38 m × 1.0 m |
| **Connectivity** | Ethernet or WiFi |

---

## Software Requirements

| Package | Version |
|---|---|
| ROS | Noetic (Ubuntu 20.04) |
| Python | 3.x |
| `modbus_tk` | via pip |
| `ros-noetic-navigation` | apt |
| `ros-noetic-teb-local-planner` | apt |
| `ros-noetic-robot-pose-ekf` | apt |
| `ros-noetic-tf2-tools` | apt |
| `libi2c-dev` | apt |

---

## Repository Structure

```
amr_ros_diff_bot/
├── launch/
│   ├── slamware_ros_sdk_server_node.launch       # LIDAR + TF setup
│   ├── slamware_ros_sdk_intergated_with_teb.launch  # Full nav stack
│   ├── motor_control.launch                      # IMU + motor node
│   └── robot_pose_ekf.launch                     # EKF + full sensor fusion
├── scripts/
│   ├── twist_motors.py                           # TwistToMotors with acc/dec
│   └── plc_modbus_node.py                        # Modbus TCP bridge node
├── cfg/
│   └── diff_drive/
│       ├── costmap_common_params.yaml
│       ├── local_costmap_params.yaml
│       ├── global_costmap_params.yaml
│       └── teb_local_planner_params.yaml
├── rviz/
│   └── slamware_ros_sdk_teb.rviz
└── README.md
```

---

## Installation & Setup

### 1. Hardware & OS Setup

1. Connect all peripherals (LIDAR, IMU, Ethernet) to the Jetson Orin Nano.
2. Download the **JetPack SDK** from [NVIDIA](https://developer.nvidia.com/embedded/jetpack).
3. Flash **Ubuntu 20.04** to your SD card using Balena Etcher or NVIDIA SDK Manager.
4. Boot the Jetson from the SD card.
5. Configure a static IP address for the robot and the PLC network interface.

Verify the target architecture:
```bash
uname -m
# Expected: aarch64
```

---

### 2. ROS Noetic Installation

```bash
# Add ROS apt source
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic desktop full
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 3. ROS Workspace Setup

```bash
mkdir -p ~/slam_ws/src
cd ~/slam_ws
catkin_make
echo "source ~/slam_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Clone this repository into `src`:
```bash
cd ~/slam_ws/src
git clone https://github.com/<your-username>/amr_ros_diff_bot.git
cd ~/slam_ws
catkin_make
source devel/setup.bash
```

---

### 4. Slamware LIDAR SDK

Download the Slamware ROS SDK from the [Slamtec Wiki](https://wiki.slamtec.com/display/SD/2.Robot+Base):

```bash
cd ~/slam_ws/src
# Clone the Slamware ROS SDK (refer to Slamtec download portal for the correct repo)
cd ~/slam_ws
catkin_make
```

Expected build output:
```
[100%] Built target slamware_ros_sdk_server_node
```

**Verify LIDAR connection:**
```bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1
```

Check that topics are publishing:
```bash
rostopic list
rostopic echo /scan
```

**Visualize in RViz:**
```bash
roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch
```

---

### 5. Navigation Stack (TEB Planner)

```bash
sudo apt install ros-noetic-navigation -y
sudo apt install ros-noetic-teb-local-planner -y
sudo apt install ros-noetic-global-planner -y
sudo apt install ros-noetic-move-base -y
```

---

### 6. EKF Sensor Fusion

```bash
sudo apt install ros-noetic-robot-pose-ekf -y
```

The EKF node subscribes to `/odom`, `/imu_data`, and `/vo`, and publishes the fused pose to `/robot_pose_ekf/odom_combined`.

---

### 7. IMU (BNO055) Setup

**Detect the I2C bus:**
```bash
sudo i2cdetect -y -r 7
# Look for device at address 0x28 (28 in the grid)
```

**Install dependencies and build the IMU package:**
```bash
cd ~/slam_ws/src
git clone https://github.com/dheera/ros-imu-bno055.git
sudo apt install libi2c-dev -y
cd ~/slam_ws
catkin_make --only-pkg-with-deps imu_bno055
```

**Install the RViz IMU plugin (optional, for visualization):**
```bash
sudo apt install ros-noetic-rviz-imu-plugin -y
```

The IMU node is configured for:
- Device: `/dev/i2c-7`
- Address: `0x28` (40 decimal)
- Frame ID: `imu`

---

### 8. PLC Modbus Motor Control

**Install the Python Modbus library:**
```bash
pip3 install modbus_tk
```

**Install the ROS Modbus package:**
```bash
cd ~/slam_ws/src
git clone https://github.com/sonyccd/ros_plc_modbus.git
cd ~/slam_ws
catkin_make
```

**Default Modbus parameters** (override in launch file or via `rosparam`):

| Parameter | Default | Description |
|---|---|---|
| `~ip` | `192.168.1.5` | PLC IP address |
| `~port` | `502` | Modbus TCP port |
| `~spin_rate` | `30` | Read loop frequency (Hz) |
| `~regs_addr` | `[]` | Holding register addresses to monitor |
| `~coils_addr` | `[]` | Coil addresses to monitor |

**Motor command format** (published to `modbus/regs_write` as `UInt16MultiArray`):

```
[left_wheel_speed, right_wheel_speed, direction_left, direction_right]
```
- Speed values are in units of `(RPM / 60) × 10000` (integer)
- Direction: `0` = forward, `1` = reverse

---

### 9. TF & Debugging Tools

```bash
sudo apt install ros-noetic-tf2-tools -y

# Generate TF tree PDF
rosrun tf2_tools view_frames.py
evince frames.pdf

# View node graph
rosrun rqt_graph rqt_graph
```

**Publish a static TF (for testing):**
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map world 100
```

---

### 10. Remote Access

**SSH:**
```bash
sudo apt install openssh-server -y
# From remote machine:
ssh username@<ROBOT_IP>
```

**VNC (for remote desktop):**
```bash
sudo apt install x11vnc -y
x11vnc -display :0
# Connect from remote with any VNC client to <ROBOT_IP>:5900
```

---

## Configuration

### TF Frames

The TF tree for this robot is:

```
slamware_map
    └── odom  (static, via map2odom publisher in slamware launch)
            └── base_link
                    ├── laser      (x=0.5, y=0, z=0  — front-mounted LIDAR)
                    └── imu        (x=0, y=0, z=0.15 — from motor_control.launch)
```

Static transforms are defined in `slamware_ros_sdk_server_node.launch`:

```xml
<node name="map2odom" pkg="tf" type="static_transform_publisher"
      args="0 0 0 0 0 0 1 /slamware_map /odom 100"/>
<node name="base2laser" pkg="tf" type="static_transform_publisher"
      args="0.5 0 0 0 0 0 1 /base_link /laser 100"/>
<node name="base2imu" pkg="tf" type="static_transform_publisher"
      args="0 0 0 0 0 0 1 /base_link /imu 100"/>
```

> **Note:** Adjust the `x`, `y`, `z` translation arguments to match your physical sensor mounting positions.

### Robot Footprint

Configured in `teb_local_planner_params.yaml` for a robot body of ~0.38 m × 1.0 m:

```yaml
footprint: [[0.45, 0.76], [0.45, 0.24], [-0.07, 0.24], [-0.07, 0.76]]
```

### Modbus Parameters

The `plc_modbus_node` uses ROS parameters. You can override them in your launch file:

```xml
<node pkg="plc_modbus_node" type="plc_modbus_node.py" name="ros_plc_modbus">
  <param name="ip"         value="192.168.1.5"/>
  <param name="port"       value="502"/>
  <param name="spin_rate"  value="30"/>
  <rosparam param="regs_addr">[100, 101, 102, 103]</rosparam>
</node>
```

---

## Running the Robot

### Full Autonomous Navigation Launch

Open terminals in the following order:

**Terminal 1 — Source the workspace (do this in every terminal):**
```bash
source ~/slam_ws/devel/setup.bash
```

**Terminal 2 — Launch Slamware LIDAR + TEB Navigation + RViz:**
```bash
roslaunch slamware_ros_sdk slamware_ros_sdk_intergated_with_teb.launch
```

This single launch file starts:
- `slamware_ros_sdk_server_node` (LIDAR + odometry + map)
- `move_base` with TEB local planner
- Static TF publishers (map → odom, base_link → laser, base_link → imu)
- RViz with the pre-configured `.rviz` file

**Terminal 3 — Launch IMU + Motor Control:**
```bash
roslaunch differential_drive motor_control.launch
```

This starts:
- `bno055_i2c_node` (IMU on `/dev/i2c-7`)
- `twist_to_motors` node (converts `/cmd_vel` → Modbus motor commands)
- `baselink2imu` static TF

**Terminal 4 — Launch PLC Modbus Bridge:**
```bash
roslaunch plc_modbus_node modbus_launch.launch
```

**Send navigation goals** via RViz using the "2D Nav Goal" tool, or programmatically via `/move_base_simple/goal`.

---

### Manual Joystick Control

With all nodes running, launch the virtual joystick interface:
```bash
python3 v_joystick.py
```

The joystick sends `UInt8` messages on `button_action_CMD` with the following command codes:

| Code | Action |
|---|---|
| `14` | Switch to **Auto Mode** (follows `/cmd_vel` from `move_base`) |
| `15` | Switch to **Manual Mode** |
| `1` | Move Forward |
| `2` | Forward Left |
| `3` | Forward Right |
| `4` | Rotate Left |
| `5` | Rotate Right |
| `6` | Backward |
| `7` | Backward Right |
| `8` | Backward Left |
| `9` | **Stop** |
| `10` | Right wheel forward only |
| `11` | Right wheel backward only |
| `12` | Left wheel forward only |
| `13` | Left wheel backward only |

Speed limits are set via the `slider_CMD` topic (`Float32MultiArray`):
```
[left_wheel_max, right_wheel_max, linear_vel_max, angular_vel_max]
```
Values are in the range `0–255` (scaled proportionally to RPM).

---

## Node & Topic Reference

| Node | Subscribes | Publishes |
|---|---|---|
| `slamware_ros_sdk_server_node` | `/cmd_vel` | `/scan`, `/odom`, `/slamware_map` |
| `move_base` (TEB) | `/scan`, `/odom`, `/slamware_map` | `/cmd_vel` |
| `imu_node` (BNO055) | — | `/imu/data` |
| `robot_pose_ekf` | `/odom`, `/imu/data` | `/robot_pose_ekf/odom_combined` |
| `twist_to_motors` | `/cmd_vel`, `button_action_CMD`, `slider_CMD` | `modbus/regs_write`, `disp_spd` |
| `ros_plc_modbus` | `modbus/regs_write`, `modbus/coils_write` | `modbus/regs_read`, `modbus/coils_read` |

---

## Key Source Files

### `twist_motors.py` — TwistToMotors Node

Converts `/cmd_vel` (linear + angular velocity) into left/right wheel RPM commands published over Modbus. Key features:

- **Auto mode**: follows `/cmd_vel` from `move_base`, with velocity clamped to ±0.5 m/s
- **Manual mode**: takes joystick commands via `button_action_CMD`
- **Smooth acceleration/deceleration**: `findlacc()` / `findracc()` compute per-wheel ramp step sizes dynamically based on the magnitude of the speed delta
- **Wheel kinematics**: differential drive equations with configurable `base_width` (0.38 m) and `wheel_radius` (0.16 m)

### `plc_modbus_node.py` — PLC Modbus Bridge

Bridges ROS topics to a Modbus TCP device (PLC or motor drive):

- Reads holding registers and coils at `~spin_rate` Hz, publishing results
- Writes register/coil values received from ROS topics to the PLC
- Gracefully handles `ModbusError` with `rospy.logerr` without crashing

---

## Troubleshooting

**LIDAR not connecting:**
```bash
# Verify network reachability
ping 192.168.11.1
# Check if Slamware port is open
nc -zv 192.168.11.1 1445
```

**IMU not detected on I2C:**
```bash
sudo i2cdetect -y -r 7
# If not shown, check physical wiring and try buses 0–9
for i in $(seq 0 9); do sudo i2cdetect -y -r $i 2>/dev/null | grep -q "28" && echo "Found on bus $i"; done
```

**Modbus connection failed:**
```bash
# Verify PLC is reachable
ping 192.168.1.5
# Test Modbus port
nc -zv 192.168.1.5 502
```

**TF tree broken / navigation not working:**
```bash
rosrun tf2_tools view_frames.py && evince frames.pdf
# Look for disconnected frames or large time jumps
rosrun tf tf_monitor
```

**`move_base` not receiving laser scans:**
```bash
rostopic hz /scan
# Should be ~10 Hz
rostopic echo /scan --noarr   # Check header.frame_id matches costmap config
```

**Robot not moving in auto mode:**
```bash
# Check cmd_vel is being published by move_base
rostopic echo /cmd_vel
# Check motor node is receiving and translating it
rostopic echo /modbus/regs_write
```

---

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

---

*Built with ROS Noetic on Jetson Orin Nano. Contributions welcome — open an issue or pull request.*
