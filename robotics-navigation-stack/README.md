# Sensor Fusion & Odometry Analysis Stack

This workspace implements a robust sensor fusion pipeline for a differential drive mobile robot. It combines wheel odometry with high-precision IMU data using an Extended Kalman Filter (EKF) to produce a stable and accurate odometry estimate.

## 📦 System Overview

### Data Flow

1. **Motors** $\to$ `solo_motor_controller` $\to$ Raw Encoder Ticks
2. **Encoder Ticks** $\to$ `nandi_controller` $\to$ Wheel Odometry (`/mobile_base_controller/odom`)
3. **IMU Sensor** $\to$ `phidgets_drivers` $\to$ IMU Data (`/imu/data`)
4. **Odometry + IMU** $\to$ `robot_localization` $\to$ Fused State (`/odometry/filtered`)

**Goal**: Accurate state estimation (`odom` $\to$ `base_footprint`) by correcting wheel slip and drift using IMU data.

---

## 🛠️ Prerequisites & Installation

Assumption: A fresh installation of **Ubuntu 24.04** and **ROS 2 Jazzy Jalisco**.

### 1. System Dependencies

Install build tools and driver requirements (including `libusb` for Phidgets).

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions git
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro
sudo apt install -y libusb-1.0-0-dev
```

### 2. Python Dependencies

The motor driver requires `SoloPy`.

```bash
# Install SoloPy (Check your motor hardware documentation for the specific source)
pip3 install SoloPy
```

### 3. Hardware Permissions (udev)

For USB access to Phidgets and Motor Controllers:

```bash
sudo usermod -aG dialout $USER
# Log out and log back in!
```

*See `docs/udev_rule_for_motorcontrollers.md` for detailed motor setup.*

---

## 🚀 Building the Workspace

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Build
colcon build --symlink-install
```

---

## 📦 Package Details

### 1. `robot_localization` (Modified)

A streamlined build of the standard EKF package.

* **Original Repository**: [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
* **Function**: Fuses Wheel Velocity ($v_x, v_y$) and IMU Yaw Rate ($\omega_{yaw}$).
* **Modifications**: Removed heavy dependencies (UKF, NavSat, Tests) for a streamlined build.

| Category | Name | Description |
| :--- | :--- | :--- |
| **Sub (Topic)** | `/mobile_base_controller/odom` | Input Wheel Odometry. |
| **Sub (Topic)** | `/imu/data_raw` | Input IMU Data. |
| **Pub (Topic)** | `/odometry/filtered` | Output Fused Odometry. |
| **Pub (Topic)** | `/tf` | Output TF (`odom` $\to$ `base_footprint`). |
| **Param** | `frequency` | `50.0` Hz |
| **Param** | `two_d_mode` | `true` |
| **Service** | `/set_pose` | Reset filter pose. |
| **Service** | `/toggle_filter_processing` | Pause/Resume filter. |
| **Service** | `/get_state` | Get current state. |

### 2. `phidgets_drivers`

Drivers for Phidgets Spatial 3/3/3 IMU.

* **Original Repository**: [ros-drivers/phidgets_drivers](https://github.com/ros-drivers/phidgets_drivers)

| Category | Name | Description |
| :--- | :--- | :--- |
| **Pub (Topic)** | `/imu/data` | Calibrated IMU data. |
| **Pub (Topic)** | `/imu/mag` | Magnetometer data. |
| **Param** | `serial` | Device Serial Number. |
| **Param** | `hub_port` | VINT Hub Port. |
| **Param** | `data_interval_ms` | Polling interval. |

### 3. `nandi_controller`

Hardware Interface for the robot.

| Category | Name | Description |
| :--- | :--- | :--- |
| **Sub (Topic)** | `/cmd_vel` | Input velocity commands. |
| **Pub (Topic)** | `/mobile_base_controller/odom` | Raw Wheel Odometry. |
| **Pub (Topic)** | `/joint_states` | Wheel positions. |
| **Param** | `wheel_separation` | `0.452` m |
| **Param** | `wheel_radius` | `0.049` m |

### 4. `solo_motor_controller`

Low-level Python UART driver.

| Category | Name | Description |
| :--- | :--- | :--- |
| **Pub (Topic)** | `/left_encoder_counts` | Raw Left Ticks. |
| **Pub (Topic)** | `/right_encoder_counts` | Raw Right Ticks. |
| **Param** | `motor_port` | Serial Port (e.g. `/dev/ttyUSB0`). |
| **Param** | `motor_addr` | Device Address (e.g. `0x01`). |

### 5. `pose_logger_pkg`

Analysis tool for benchmarking.

| Category | Name | Description |
| :--- | :--- | :--- |
| **Sub (Topic)** | `/mobile_base_controller/odom` | Raw Path input. |
| **Sub (Topic)** | `/odometry/filtered` | Fused Path input. |
| **Pub (Topic)** | `/viz/wheel_path` | Visualization (Red). |
| **Pub (Topic)** | `/viz/filtered_path` | Visualization (Green). |
| **Param** | `log_dir` | Directory to save CSV logs. |
| **Param** | `min_dist_threshold` | `0.10` m (Recording threshold). |

> **Note**: Update `log_dir` in `src/pose_logger_pkg/config/pose_logger_params.yaml` to your actual path.

---

## 🏃 Usage Guide

### Step 1: Launch Hardware

```bash
ros2 launch solo_motor_controller solo_motors.launch.py
ros2 launch nandi_controller bringup.launch.py
```

### Step 2: Launch Encoded Sensors

```bash
ros2 launch phidgets_spatial spatial.launch.py
```

### Step 3: Launch Sensor Fusion

```bash
ros2 launch robot_localization ekf.launch.py
```

### Step 4: Visualize

```bash
ros2 run pose_logger_pkg pose_logger_node
```

Open **RViz2**, set Fixed Frame to `odom`, and add the Path topics.

## Maintainer

Abhinav Sharma
