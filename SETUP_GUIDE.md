# Ball Follower Robot — Setup & Wiring Guide

Complete guide to set up the autonomous ball-following robot with ROS2 mapping on Raspberry Pi 5 + Arduino Uno.

---

## 1. Wiring Diagram

### L293D Motor Driver → Arduino Uno

| L293D Pin     | Arduino Uno Pin | Description            |
|---------------|-----------------|------------------------|
| Enable A (ENA)| Pin 10 (PWM)    | Left motor speed       |
| Input 1 (IN1) | Pin 8           | Left motor direction   |
| Input 2 (IN2) | Pin 9           | Left motor direction   |
| Enable B (ENB)| Pin 5 (PWM)     | Right motor speed      |
| Input 3 (IN3) | Pin 6           | Right motor direction  |
| Input 4 (IN4) | Pin 7           | Right motor direction  |
| Vcc1 (Logic)  | 5V              | Logic power            |
| Vcc2 (Motor)  | External 6-12V  | Motor power supply     |
| GND (all)     | GND             | Common ground          |

### Motors

| Motor      | L293D Output |
|------------|-------------|
| Left Motor | Output 1 & 2 (Motor A side) |
| Right Motor| Output 3 & 4 (Motor B side) |

### HC-SR04 Ultrasonic Sensor → Arduino Uno

| HC-SR04 Pin | Arduino Pin | Description  |
|-------------|-------------|-------------|
| VCC         | 5V          | Power       |
| TRIG        | Pin 12      | Trigger     |
| ECHO        | Pin 11      | Echo        |
| GND         | GND         | Ground      |

### Servo Motor → Arduino Uno

| Servo Wire | Arduino Pin | Description  |
|------------|-------------|-------------|
| Signal (Orange/White) | Pin 3 (PWM) | Control |
| VCC (Red)  | 5V or external 5V | Power |
| GND (Brown/Black) | GND | Ground |

> [!IMPORTANT]
> If the servo draws too much current, power it from an external 5V supply (share GND with Arduino).

### Connections to Raspberry Pi 5

| Connection      | Details                              |
|-----------------|--------------------------------------|
| Arduino Uno     | USB-B cable → RPi 5 USB port (`/dev/ttyUSB0` or `/dev/ttyACM0`) |
| USB Webcam      | USB cable → RPi 5 USB port          |
| Power           | RPi 5: USB-C 5V/5A power supply     |
| Motor Power     | Separate battery pack (6-12V) to L293D Vcc2 |

### Wiring Overview
```
 [Battery 6-12V] ──→ [L293D Vcc2]
                         │
   [Arduino Uno] ── pins ──→ [L293D] ──→ [Left Motor]
         │                             ──→ [Right Motor]
         │── Pin 3  ──→ [Servo]
         │── Pin 12 ──→ [HC-SR04 TRIG]
         │── Pin 11 ←── [HC-SR04 ECHO]
         │
         │── USB ──→ [Raspberry Pi 5]
                         │
                    [USB Webcam]
```

---

## 2. Arduino Setup

### Install Arduino IDE
Download from [arduino.cc](https://www.arduino.cc/en/software) on your PC (or use `arduino-cli` on the Pi).

### Upload the Firmware
1. Open `arduino/ball_follower_arduino.ino` in Arduino IDE
2. Select **Board**: Arduino Uno
3. Select **Port**: the correct COM/serial port
4. Click **Upload**

### Test (Serial Monitor)
1. Open Serial Monitor at **115200 baud**
2. You should see `READY` and then `S,angle,distance` scan data
3. Type `M,150,150` + Enter → both motors should spin forward
4. Type `STOP` + Enter → motors stop

---

## 3. Raspberry Pi 5 Setup

### 3.1 Install Ubuntu 24.04 + ROS2 Jazzy

```bash
# If not already done, flash Ubuntu 24.04 Server for RPi 5
# Then install ROS2 Jazzy:
sudo apt update && sudo apt upgrade -y

# Add ROS2 repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 3.2 Install Dependencies

```bash
# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install required packages
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-opencv \
  ros-jazzy-cv-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-ros

# Install Python serial library
pip3 install pyserial
```

### 3.3 Serial Port Permissions

```bash
# Add your user to dialout group (for serial access)
sudo usermod -aG dialout $USER

# Log out and back in, or:
newgrp dialout
```

### 3.4 Copy the ROS2 Package

```bash
# Create a ROS2 workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Copy the package (from your dev machine or USB)
# Option A: scp from your PC
scp -r user@your-pc:/path/to/ball_follower_robot .

# Option B: if files are already on the Pi
cp -r /path/to/ball_follower_robot .
```

### 3.5 Build the Package

```bash
cd ~/robot_ws
colcon build --packages-select ball_follower_robot
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

### 3.6 Find Your Serial Port

```bash
# Plug in Arduino via USB, then:
ls /dev/ttyUSB* /dev/ttyACM*
# Usually /dev/ttyUSB0 or /dev/ttyACM0
```

---

## 4. Running the Robot

### Launch Everything
```bash
# Default (serial on /dev/ttyUSB0, camera 0)
ros2 launch ball_follower_robot launch_robot.py

# Custom serial port
ros2 launch ball_follower_robot launch_robot.py serial_port:=/dev/ttyACM0

# Disable preview window (headless)
ros2 launch ball_follower_robot launch_robot.py show_preview:=false
```

### Run Nodes Individually (for debugging)
```bash
# Terminal 1: Serial bridge
ros2 run ball_follower_robot serial_bridge --ros-args -p serial_port:=/dev/ttyUSB0

# Terminal 2: Object tracker
ros2 run ball_follower_robot object_tracker --ros-args -p camera_index:=0

# Terminal 3: Path tracker
ros2 run ball_follower_robot path_tracker

# Terminal 4: SLAM
ros2 run slam_toolbox async_slam_toolbox_node
```

---

## 5. RViz Visualization

### Open RViz2
```bash
rviz2
```

### Add these displays in RViz:
1. **Map** — Topic: `/map` — shows the occupancy grid
2. **LaserScan** — Topic: `/scan` — shows ultrasonic readings
3. **Path** — Topic: `/path` — shows robot's traveled path
4. **TF** — shows coordinate frames (odom, base_link, laser_frame)
5. **Image** — Topic: `/camera/image_raw` — camera feed

### Set Fixed Frame
In the **Global Options** panel (top-left), set **Fixed Frame** to `map` or `odom`.

---

## 6. Tuning HSV for Your Yellow Ball

The default HSV range works for a typical yellow tennis ball. If detection is poor:

```bash
# Run the tracker with preview
ros2 run ball_follower_robot object_tracker --ros-args -p show_preview:=true

# Adjust HSV via parameters at launch:
ros2 launch ball_follower_robot launch_robot.py \
  h_low:=15 s_low:=80 v_low:=80 \
  h_high:=40 s_high:=255 v_high:=255
```

### HSV Tuning Helper (run on Pi with display):
```python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.namedWindow('Trackbars')
cv2.createTrackbar('H Low', 'Trackbars', 20, 179, lambda x: None)
cv2.createTrackbar('S Low', 'Trackbars', 100, 255, lambda x: None)
cv2.createTrackbar('V Low', 'Trackbars', 100, 255, lambda x: None)
cv2.createTrackbar('H High', 'Trackbars', 35, 179, lambda x: None)
cv2.createTrackbar('S High', 'Trackbars', 255, 255, lambda x: None)
cv2.createTrackbar('V High', 'Trackbars', 255, 255, lambda x: None)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low = np.array([cv2.getTrackbarPos('H Low', 'Trackbars'),
                    cv2.getTrackbarPos('S Low', 'Trackbars'),
                    cv2.getTrackbarPos('V Low', 'Trackbars')])
    high = np.array([cv2.getTrackbarPos('H High', 'Trackbars'),
                     cv2.getTrackbarPos('S High', 'Trackbars'),
                     cv2.getTrackbarPos('V High', 'Trackbars')])
    mask = cv2.inRange(hsv, low, high)
    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f"HSV Low:  {low}")
        print(f"HSV High: {high}")
        break
cap.release()
cv2.destroyAllWindows()
```

---

## 7. Troubleshooting

| Issue | Solution |
|-------|----------|
| `Cannot open serial port` | Check `ls /dev/ttyUSB*`, ensure `dialout` group, try `/dev/ttyACM0` |
| Motors don't move | Check L293D wiring, ensure external motor power connected |
| Ball not detected | Tune HSV values (see Section 6), check lighting |
| Servo jittering | Power servo from external 5V, not Arduino's 5V pin |
| Map not building | Check `ros2 topic echo /scan` for data, ensure SLAM is running |
| `cv_bridge` error | Install: `sudo apt install ros-jazzy-cv-bridge` |

---

## 8. Project File Structure

```
Random/
├── arduino/
│   └── ball_follower_arduino.ino     ← Upload to Arduino Uno
├── ball_follower_robot/              ← ROS2 package (copy to ~/robot_ws/src/)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── ball_follower_robot
│   ├── ball_follower_robot/
│   │   ├── __init__.py
│   │   ├── object_tracker.py         ← Yellow ball detection
│   │   ├── motor_controller.py       ← Motor serial (standalone)
│   │   ├── ultrasonic_mapper.py      ← Scan publisher (standalone)
│   │   ├── serial_bridge.py          ← Combined motor+scan (recommended)
│   │   └── path_tracker.py           ← Path visualization
│   └── launch/
│       └── launch_robot.py           ← Launch all nodes
└── SETUP_GUIDE.md                    ← This file
```
