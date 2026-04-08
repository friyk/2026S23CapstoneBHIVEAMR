# B.HIVE AMR — Autonomous Mobile Robot

B.HIVE is a functional autonomous mobile robot prototype built from the ground up as a capstone project. The robot autonomously navigates indoor environments, localises against a pre-built map, and communicates with a companion mobile app over Bluetooth Low Energy — designed around a real-world use case of transporting and weighing luggage.

This repository contains the full ROS 2 software stack running on the robot. The companion embedded firmware for the loading and weighing subsystem lives in a separate repository: **[2026S23CapstoneBHIVELOADANDWEIGH](https://github.com/friyk/2026S23CapstoneBHIVELOADANDWEIGH)**.

---

## What This Project Demonstrates

- **Full-stack robotics integration** — hardware drivers, sensor fusion, localisation, path planning, and a BLE communication layer, all coordinated through ROS 2 launch files on an NVIDIA Jetson Orin Nano.
- **Differential-drive control and odometry** — custom Modbus RTU driver for ZLAC8015D hub-motor controllers with real-time wheel encoder feedback and current monitoring.
- **Sensor fusion** — an Extended Kalman Filter fuses wheel odometry with a 6-axis IMU (ISM330DHCX) to produce a smooth, drift-corrected pose estimate.
- **Map-based localisation** — AMCL (Adaptive Monte Carlo Localisation) running against a pre-built occupancy grid, with Nav2 handling global and local path planning.
- **BLE-to-ROS bridge** — a BlueZ GATT peripheral server that lets a phone app send booking, follow, and navigation commands to the robot, and receive live status updates back.
- **URDF-driven TF tree** — a complete Xacro robot model with accurate sensor placements, enabling correct transform publishing across the full perception and navigation pipeline.

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                      Jetson Orin Nano                         │
│                                                               │
│  grove_sensors       zlac_driver         hivebot_ble          │
│  ┌────────────┐     ┌─────────────┐     ┌────────────┐       │
│  │ IMU        │     │ Motor ctrl  │     │ BLE GATT   │       │
│  │ (I2C)      │     │ (Modbus     │     │ server     │       │
│  │            │     │  RS-485)    │     │ (BlueZ)    │       │
│  └─────┬──────┘     └──────┬──────┘     └─────┬──────┘       │
│        │                   │                  │               │
│   /imu/data_raw       /odom  /cmd_vel    /ble/commands        │
│        │                   │                  │               │
│  ┌─────▼───────────────────▼──────────────────▼──────────┐   │
│  │              robot_bringup (launch + config)           │   │
│  │                                                        │   │
│  │   imu_filter_madgwick → EKF → AMCL → Nav2             │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                               │
│  RPLiDAR ──── /scan          RealSense D435 ──── /camera/*   │
└───────────────────────────────────────────────────────────────┘

                        ┌─────────────┐
                        │  Phone App  │
                        │  (BLE)      │
                        └─────────────┘
```

---

## Hardware

| Component | Model | Interface | Notes |
|---|---|---|---|
| Compute | NVIDIA Jetson Orin Nano | — | Ubuntu 22.04 + ROS 2 Humble |
| Motor driver | ZLAC8015D (dual channel) | RS-485 / Modbus RTU | Velocity mode, 8″ hub motors |
| LiDAR | RPLiDAR (A-series) | USB serial (`/dev/rplidar`) | 360°, up to 12 m range |
| Depth camera | Intel RealSense D435 | USB 3.0 | RGB 640×480, depth 848×480 |
| IMU | ISM330DHCX (Grove) | I2C bus 7, addr `0x6A` | 6-axis, sampled at 100 Hz |
| Ultrasonic | HC-SR04 (×4 positions) | GPIO (`gpiochip0`) | Front / back / left / right |
| BLE | Onboard Bluetooth (hci0) | BlueZ D-Bus | Phone ↔ robot communication |

**Chassis dimensions:** 565 × 400 × 384 mm (L × W × H). Wheel radius 87.5 mm, track width 275 mm. The wheel axle is offset 132.5 mm behind the geometric centre.

---

## Repository Structure

```
├── robot_description/          URDF / Xacro model with all sensor frames
│   └── urdf/robot.urdf.xacro
│
├── grove_sensors/              ISM330DHCX IMU + HC-SR04 ultrasonic drivers
│   └── grove_sensors/
│       ├── imu_node.py
│       └── ultrasonic_node.py
│
├── zlac_driver/                ZLAC8015D motor control + wheel odometry
│   └── zlac_driver/
│       ├── ZLAC8015D.py            Low-level Modbus API
│       └── zlac_ros2_node.py       ROS 2 node (cmd_vel ↔ motors ↔ odom)
│
├── hivebot_ble/                BLE GATT server + follow behaviour
│   └── hivebot_ble/
│       ├── ble_node.py             BlueZ peripheral + ROS 2 bridge
│       └── follow_node.py          Obstacle-aware forward-follow mode
│
└── robot_bringup/              Launch files and parameter configs
    ├── launch/
    │   ├── localisation.launch.py              AMCL localisation stack
    │   ├── navigation.launch.py                AMCL + Nav2 autonomous nav
    │   ├── robot.launch.py                     Full sensor stack (mapping)
    │   ├── rtabmap_mapping.launch.py           RTAB-Map (LiDAR + RGB-D)
    │   ├── rtabmap_lidar_mapping.launch.py     RTAB-Map (LiDAR only)
    │   ├── rtabmap_navigation.launch.py        RTAB-Map nav (RGB-D)
    │   └── rtabmap_lidar_navigation.launch.py  RTAB-Map nav (LiDAR only)
    └── config/
        ├── ekf.yaml                            EKF sensor fusion params
        ├── nav2_params.yaml                    Nav2 planner / controller
        ├── slam_toolbox_params.yaml            (map building utility)
        └── slam_toolbox_localisation.yaml      (map building utility)
```

---

## Package Details

### `robot_description`

Xacro URDF defining the robot's physical geometry and every sensor frame. All TF transforms originate from this model via `robot_state_publisher`. Sensor placements match the physical prototype: the LiDAR sits 12.5 cm forward of the wheel axle at 43.9 cm height, the RealSense is mounted 41.5 cm forward at 30.25 cm, and the IMU is at the chassis centre.

### `grove_sensors`

**`imu_node`** — Reads the ISM330DHCX accelerometer (±2 g) and gyroscope (±250 dps) over I2C at 100 Hz. Performs a 2-second gyro-Z bias calibration on startup (robot must be stationary). Publishes `sensor_msgs/Imu` on `/imu/data_raw`. Downstream, `imu_filter_madgwick` fuses this into an orientation estimate on `/imu/data`.

**`ultrasonic_node`** — Drives HC-SR04 sensors via the `gpiod` v2 character-device API at 20 Hz. Publishes `sensor_msgs/Range` per sensor. The GPIO mapping is configured for the Jetson Orin Nano's pin layout.

### `zlac_driver`

**`ZLAC8015D.py`** — A complete Modbus RTU client wrapping the ZLAC8015D's register map: velocity commands (±3000 RPM), encoder position feedback (32-bit pulse counts), motor current readback, acceleration/deceleration time configuration, fault detection, and both software and hardware emergency stop.

**`zlac_ros2_node.py`** — Subscribes to `/cmd_vel`, performs differential-drive kinematics (linear + angular velocity → per-wheel RPM), and sends commands to the motor controller. Publishes `/odom` at 20 Hz using encoder-based dead reckoning, and `/motor_amps` at 2 Hz with per-motor current draw (warns if either motor exceeds 15 A). On shutdown, the node zeroes the motors and disables the driver.

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial device for the ZLAC controller |
| `wheel_base` | `0.275` | Track width in metres |
| `wheel_radius` | `0.0865` | Wheel radius in metres |

### `hivebot_ble`

**`ble_node`** — Exposes the robot as a BLE peripheral named **"Hivebot"** using a custom GATT service. A write characteristic receives commands from a phone app; a notify characteristic pushes status updates back. Commands are routed to ROS 2 topics by prefix:

| Message prefix | Published to | Purpose |
|---|---|---|
| `book:` | `/ble/booking` | Booking request |
| `goto:` | `/ble/navigation_goal` | Send robot to a location |
| `follow:` / `loaded` / `done` / `cancel` | `/ble/commands` | Session lifecycle |

Status messages published to `/ble/status` by any node are forwarded to the phone automatically.

**`follow_node`** — When activated via a `start_follow` BLE command, drives the robot forward at 0.2 m/s while monitoring the front 60° of the LiDAR scan. Halts immediately if any obstacle is detected within 0.5 m, and resumes when the path clears.

### `robot_bringup`

Orchestrates the full system through seven launch files. The primary operational pipeline is:

1. **Build a map** using `robot.launch.py` (brings up sensors + SLAM Toolbox for mapping), then save with `map_saver_cli`.
2. **Run autonomously** using `navigation.launch.py`, which starts AMCL against the saved map and the full Nav2 stack.

The EKF (`robot_localization`) fuses wheel odometry and IMU data into `/odometry/filtered`. AMCL localises against the occupancy grid using LiDAR scans. Nav2 uses a Regulated Pure Pursuit local controller at 0.3 m/s with collision detection, and a NavFn global planner. The robot footprint accounts for the asymmetric axle offset.

RTAB-Map launch files are also included as alternatives for environments where visual loop closure or 3D map data is beneficial.

---

## Prerequisites

**Platform:** NVIDIA Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble

**ROS 2 packages (apt):**

```
ros-humble-rplidar-ros          ros-humble-nav2-bringup
ros-humble-realsense2-camera    ros-humble-nav2-amcl
ros-humble-robot-localization   ros-humble-nav2-map-server
ros-humble-slam-toolbox         ros-humble-nav2-lifecycle-manager
ros-humble-rtabmap-ros          ros-humble-imu-filter-madgwick
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
ros-humble-xacro
ros-humble-tf2-ros
```

**Python packages (pip):**

```
pymodbus==2.5.3   smbus2   gpiod   numpy   dbus-python   PyGObject
```

---

## Installation

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/friyk/2026S23CapstoneBHIVEAMR.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### udev Rules

Create `/etc/udev/rules.d/99-robot.rules` so that devices appear at the paths the launch files expect:

```udev
# ZLAC8015D (USB-RS485 adapter) — replace idVendor/idProduct with your adapter's values
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="zlac", MODE="0666"

# RPLiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Find your adapter's IDs with `lsusb`.

---

## Usage

Source the workspace before running any command: `source ~/ros2_ws/install/setup.bash`

### Build a Map

Bring up the full sensor stack with SLAM Toolbox for initial map creation. Drive the robot around (e.g. with `teleop_twist_keyboard`) to build the occupancy grid:

```bash
ros2 launch robot_bringup robot.launch.py
```

Save the map when complete:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
```

### Autonomous Navigation

Loads the saved map, runs AMCL for localisation, and starts the full Nav2 stack. Set an initial pose estimate in RViz2, then send navigation goals:

```bash
ros2 launch robot_bringup navigation.launch.py
```

### Localisation Only (No Nav2)

Useful for verifying localisation before enabling autonomous driving:

```bash
ros2 launch robot_bringup localisation.launch.py
```

### RTAB-Map Alternatives

Available for environments where visual loop closure or 3D map data is beneficial:

```bash
# Mapping — LiDAR only
ros2 launch robot_bringup rtabmap_lidar_mapping.launch.py

# Mapping — LiDAR + RealSense depth
ros2 launch robot_bringup rtabmap_mapping.launch.py

# Navigation — LiDAR only
ros2 launch robot_bringup rtabmap_lidar_navigation.launch.py

# Navigation — LiDAR + RealSense depth
ros2 launch robot_bringup rtabmap_navigation.launch.py
```

### BLE Follow Mode

Run alongside any base launch file:

```bash
ros2 run hivebot_ble ble_node       # BLE GATT server
ros2 run hivebot_ble follow_node    # Follow behaviour
```

Send `start_follow` / `stop_follow` from the phone app via the BLE write characteristic.

---

## Key ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `Twist` | Velocity commands (Nav2, teleop, or follow_node) |
| `/odom` | `Odometry` | Wheel odometry from encoder feedback (20 Hz) |
| `/odometry/filtered` | `Odometry` | EKF-fused odometry (wheel + IMU) |
| `/imu/data_raw` | `Imu` | Raw accelerometer + gyroscope (100 Hz) |
| `/imu/data` | `Imu` | Madgwick-filtered IMU with orientation |
| `/scan` | `LaserScan` | RPLiDAR 360° scan |
| `/motor_amps` | `Float32MultiArray` | Motor current [left, right] (2 Hz) |
| `/ble/commands` | `String` | BLE commands from phone |
| `/ble/navigation_goal` | `String` | Navigation goal from phone |
| `/ble/status` | `String` | Status messages pushed to phone |

---

## TF Tree

```
map
 └── odom                         (published by AMCL)
      └── base_footprint
           └── base_link          (wheel axle centre)
                ├── laser_frame
                ├── camera_link
                │    └── camera_color_optical_frame
                ├── imu_link
                ├── ultrasonic_front_link
                ├── ultrasonic_back_link
                ├── ultrasonic_left_link
                └── ultrasonic_right_link
```

---

## Configuration Reference

| File | Purpose |
|---|---|
| `ekf.yaml` | EKF sensor fusion — fuses wheel odom (x, y, vx, vyaw) with IMU (yaw, angular vel, linear accel). 30 Hz, 2D mode. |
| `nav2_params.yaml` | Nav2 — Regulated Pure Pursuit controller (0.3 m/s), NavFn planner, asymmetric polygon footprint, 0.4 m goal tolerance. |
| `slam_toolbox_params.yaml` | SLAM Toolbox mapping mode — used during initial map building only. 5 cm resolution, 12 m range, loop closure enabled. |
| `slam_toolbox_localisation.yaml` | SLAM Toolbox localisation mode — available as an alternative to AMCL. |

---

## Related Repository

**[2026S23CapstoneBHIVELOADANDWEIGH](https://github.com/friyk/2026S23CapstoneBHIVELOADANDWEIGH)** — PlatformIO (C/C++) firmware for the B.HIVE loading and weighing subsystem. Handles load cell sensing and luggage weight measurement. Together with this repository, these form the complete B.HIVE capstone system.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| Motor driver not connecting | Verify `ls /dev/zlac` exists. Confirm ZLAC8015D is powered and baud rate is 115200. Run `sudo dmesg \| tail` after plugging in. |
| IMU not found | Run `i2cdetect -y 7` — device should appear at `0x6A`. If bus 7 doesn't exist, adjust the `i2c_bus` parameter. |
| LiDAR not spinning | Verify `ls /dev/rplidar`. Check udev rules and USB connection. |
| AMCL not converging | Set an accurate initial pose in RViz2 ("2D Pose Estimate"). Verify the correct map `.yaml` and `.pgm` are being loaded. |
| BLE not advertising | Ensure `bluetoothd` is running and `hci0` is up (`sudo hciconfig hci0 up`). Node requires D-Bus permissions. |
| EKF drift | Keep the robot stationary during the first 2 s after launch (gyro bias calibration). Verify `wheel_base` and `wheel_radius`. |

---

## License

Individual packages declare their own licenses in `package.xml`. The ZLAC8015D low-level driver is derived from [baptisteLynx/zlac_driver](https://github.com/baptisteLynx/zlac_driver).
