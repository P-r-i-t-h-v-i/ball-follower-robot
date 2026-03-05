# 🤖 Ball Follower Robot — Autonomous Object Tracking with ROS2 Mapping

An autonomous differential-drive robot that tracks a yellow ball using computer vision, follows it, and maps the surrounding environment using an ultrasonic sensor — all visualized in ROS2/RViz.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red)
![Arduino](https://img.shields.io/badge/Arduino-Uno-teal)

## 🎯 Features

- **Object Tracking** — Detects and follows a yellow ball using OpenCV HSV filtering
- **Differential Drive** — 2-wheel robot controlled via L293D motor shield (AFMotor library)
- **Ultrasonic Mapping** — HC-SR04 on a servo sweeps 0°–180° to scan surroundings
- **ROS2 SLAM** — Builds occupancy grid map using `slam_toolbox`
- **Path Visualization** — Records and displays the robot's traveled path in RViz
- **Split Architecture** — Pi runs robot nodes headlessly, laptop runs RViz over WiFi

## 🏗️ Hardware

| Component | Details |
|-----------|---------|
| Raspberry Pi 5 | Main computer (runs ROS2 nodes) |
| Arduino Uno | Motor/sensor controller (via USB serial) |
| L293D Motor Shield | Adafruit Motor Shield v1 (AFMotor.h) |
| 2× DC Motors | Rear differential drive wheels |
| Servo Motor | Sweeps ultrasonic sensor 0°–180° |
| HC-SR04 | Ultrasonic distance sensor |
| USB Webcam | Yellow ball detection |
| Caster Wheel | Front passive wheel |

### Chassis Dimensions
- **Body**: 15cm × 19cm × 8cm
- **Wheel Diameter**: 5.5cm
- **Layout**: Drive wheels at back, caster + servo + camera at front

## 📁 Project Structure

```
├── arduino/
│   └── ball_follower_arduino.ino   ← Upload to Arduino Uno
├── ball_follower_robot/            ← ROS2 package (copy to Pi)
│   ├── ball_follower_robot/
│   │   ├── serial_bridge.py        ← Motor + ultrasonic (single serial)
│   │   ├── object_tracker.py       ← Yellow ball detection (OpenCV)
│   │   ├── path_tracker.py         ← Path recording
│   │   ├── motor_controller.py     ← Standalone motor node
│   │   └── ultrasonic_mapper.py    ← Standalone scan node
│   ├── launch/
│   │   ├── launch_pi.py            ← ⭐ Run on Raspberry Pi
│   │   ├── launch_laptop.py        ← ⭐ Run on Laptop (RViz only)
│   │   ├── launch_robot.py         ← All-in-one launch
│   │   └── launch_gazebo.py        ← Gazebo simulation
│   ├── urdf/robot.urdf
│   ├── rviz/robot_view.rviz
│   ├── package.xml
│   └── setup.py
├── SETUP_GUIDE.md                  ← Full wiring + installation guide
└── README.md
```

## ⚡ Quick Start

### 1. Arduino
Upload `arduino/ball_follower_arduino.ino` to your Arduino Uno via Arduino IDE.
Requires the **AFMotor** library.

### 2. Raspberry Pi
```bash
# Install dependencies
sudo apt install -y ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-slam-toolbox ros-jazzy-tf2-ros ros-jazzy-cv-bridge python3-colcon-common-extensions
pip3 install pyserial opencv-python
sudo usermod -aG dialout $USER

# Build
mkdir -p ~/robot_ws/src
cp -r ball_follower_robot/ ~/robot_ws/src/
cd ~/robot_ws && colcon build && source install/setup.bash

# Run
ros2 launch ball_follower_robot launch_pi.py
```

### 3. Laptop (RViz visualization)
```bash
# Same package must be built on laptop too
ros2 launch ball_follower_robot launch_laptop.py
```

> Both Pi and Laptop must be on the **same WiFi** with `export ROS_DOMAIN_ID=0`

## 🔌 Wiring

| Connection | Arduino Pin |
|------------|-------------|
| Left Motor | M1 terminal on shield |
| Right Motor | M2 terminal on shield |
| Servo | Pin 10 (Servo2 header) |
| HC-SR04 TRIG | Pin 2 |
| HC-SR04 ECHO | Pin 13 |
| Arduino → Pi | USB cable |

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed wiring diagrams.

## 🎨 Tuning Ball Color Detection

Default HSV range is for a yellow tennis ball. Adjust via launch parameters:
```bash
ros2 launch ball_follower_robot launch_pi.py h_low:=15 s_low:=80 h_high:=40
```

## 📡 ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/target_cmd` | `Twist` | Steering commands from ball tracker |
| `/scan` | `LaserScan` | Ultrasonic sweep data |
| `/odom` | `Odometry` | Dead-reckoning odometry |
| `/path` | `Path` | Robot's traveled path |
| `/map` | `OccupancyGrid` | SLAM-generated map |
| `/camera/image_raw` | `Image` | Camera feed |

## 📄 License

MIT
