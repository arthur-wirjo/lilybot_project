<img width="300" alt="lilybot picture" src="https://github.com/user-attachments/assets/915bd0c1-2c73-4630-b25f-63a977137c55" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/47d8cae2-4256-45af-8b1b-dd87b037f47f" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/959347f6-119d-4a2d-b1f8-67e85aff1a18" />

## Lilybot: Omnidirectional Robot with LiDAR-Based Navigation

A custom omnidirectional mobile robot combining ESP32-S3 firmware, micro-ROS, and ROS 2 Nav2 for autonomous indoor navigation.

## Overview

Lilybot is a personal robotics project built to explore the full stack of an autonomous robot from scratch, from mechanical design and PCB design to embedded motor control and ROS 2 navigation.

The original motivation was simple: I wanted to build a small "smart walking table" that could help carry items around, such as bringing water or transporting cooking utensils to my dorm pantry. More importantly, this project became a way for me to learn how to build a complete autonomous robot system myself instead of only working on isolated parts.

The robot uses an ESP32-S3 for low-level control and sensing, and a Raspberry Pi 5 for higher-level ROS 2 tasks such as LiDAR processing, SLAM, localization, and navigation.

---

## Key Features

- Custom omnidirectional chassis
- Custom PCB integrating microcontroller, IMU, motor control, and encoder interfaces
- ESP32-S3-based embedded motor control firmware
- IMU and encoder feedback
- Feedforward control using a 1D lookup table with linear interpolation
- PID wheel speed control
- micro-ROS integration between ESP32-S3 and ROS 2
- LiDAR-based SLAM and Nav2 navigation on Raspberry Pi 5

---

## Hardware

### Main Components

- **Motor:** PG36-3650 geared BLDC motor, 12V rated, 1:50 reduction
- **Wheels:** Generic 75 mm omnidirectional wheels
- **IMU:** MPU6050
- **Computer:** Raspberry Pi 5 running Ubuntu 24.04
- **LiDAR:** RPLIDAR C1
- **Microcontroller:** ESP32-S3 Development Board (N16R8 variant)
- **Battery 1:** Generic 3S Li-Po battery
- **Power Distribution Board:** Generic 12V PDB with 5V output for ESP32
- **Battery 2:** Generic power bank dedicated to Raspberry Pi 5

### Custom PCB

The custom PCB includes:

- ESP32-S3 microcontroller
- IMU
- motor control outputs
- encoder feedback inputs
- two TXS0108E bidirectional level shifters

The level shifters are used because the ESP32 operates at 3.3V logic while the motor interfaces operate at 5V logic.

---

## Note on the Motor Choice

The motors used in this project are inexpensive geared BLDC motors with built-in motor drivers and encoder feedback. They are convenient for prototyping, but the velocity response is noticeably nonlinear under PWM control.

A simple PID controller alone was too slow and resulted in noticeable drift. Adding feedforward improved the response, but drift remained, especially at certain velocity ranges. I also tried a 2nd-order feedforward model with PID, which improved things further but was still not consistent enough across the full operating range.

To address this, I wrote a motor testing program to measure steady-state motor speed at different PWM duty cycles, plotted the results in a spreadsheet, and converted the data into a 1D lookup table. Linear interpolation was then used between table entries to generate the feedforward term. Combined with PID feedback, this gave the most satisfactory real-world performance.

---

## System Architecture

The system is split into two computing layers.

### ESP32-S3

The ESP32-S3 handles low-level real-time tasks:

- encoder reading
- IMU reading
- motor control loop
- odometry calculation
- micro-ROS communication
- publishing odometry-related data
- subscribing to commanded velocity data

### Raspberry Pi 5

The Raspberry Pi 5 handles high-level ROS 2 tasks:

- ROS 2 nodes
- LiDAR reading
- SLAM and localization
- Nav2 planning and control

---

## RTOS Task Structure

To keep the firmware more reliable and responsive, the ESP32-S3 uses both cores for separate responsibilities.

### Core 0: Real-Time Control

Core 0 is dedicated to the low-level control loop. Its tasks include:

- reading encoder pulse counts
- calculating odometry
- converting robot velocity targets into individual wheel targets
- running feedforward + PID wheel speed control
- updating PWM outputs to the motors

### Core 1: Communication and Sensors

Core 1 handles communication-heavy and less timing-critical tasks:

- running the micro-ROS executor
- receiving `cmd_vel` commands
- reading IMU data
- publishing IMU and odometry messages to ROS 2 over UART

This separation helped keep the control loop more stable while still maintaining communication with the Raspberry Pi.

---

## Setup

These instructions assume the Raspberry Pi 5 is running Ubuntu 24.04 and development is done over SSH.

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/arthur-wirjo/lilybot_project.git
```

---

### 2. Install ESP-IDF

Lilybot firmware is developed using ESP-IDF.

Install prerequisites:

```bash
sudo apt-get update
sudo apt-get install git wget flex bison gperf python3 python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

Clone ESP-IDF:

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v6.0.2 --recursive https://github.com/espressif/esp-idf.git
```

Install the toolchain:

```bash
cd ~/esp/esp-idf
./install.sh esp32
```

Export the environment:

```bash
. $HOME/esp/esp-idf/export.sh
```

Optionally, add this alias to `~/.bashrc` for convenience:

```bash
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

Then reload:

```bash
source ~/.bashrc
```

---

### 3. Flash the ESP32-S3 Firmware

In one terminal:

```bash
cd ~/lilybot_project/firmware_esp32s3/micro-ROS
idf.py flash monitor
```

---

### 4. Run the micro-ROS Agent

In another terminal:

```bash
cd ~/lilybot_project/firmware_esp32s3
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyAMA0 -b 115200
```

Alternatively:

```bash
./run_microros_agent.sh
```

---

### 5. Build the ROS 2 Workspace

In another terminal:

```bash
cd ~/lilybot_project/ros2_ws
colcon build
```

---

### 6. Run the ROS 2 Stack

The main setup commands are:

```bash
ros2 launch my_robot_description rsp.launch.py
ros2 run foxglove_bridge foxglove_bridge
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0
ros2 run robot_localization ekf_node --ros-args --params-file src/lilybot_localization/config/ekf.yaml
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/lilybot_localization/config/mapper_params.yaml
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=src/lilybot_navigation/dorm_map.yaml params_file:=src/lilybot_navigation/config/custom_nav2_params.yaml
ros2 run nav2_map_server map_saver_cli -f src/lilybot_navigation/dorm_map
```

Additional setup commands are documented in the repository.

---

## Implementation Notes

This project was built solo, including:

- mechanical design
- PCB design
- embedded firmware
- control implementation
- ROS 2 integration
- robot assembly and testing

---

## Challenges and Lessons Learned

### Nonlinear Motor Behavior

One of the main challenges was achieving stable and consistent wheel speed control using low-cost motors with nonlinear PWM-to-speed behavior.

The progression was roughly:

1. PID only  
2. feedforward + PID  
3. 2nd-order feedforward + PID  
4. 1D lookup table feedforward + PID

The lookup-table-based feedforward gave the best results in practice.

### Noisy Encoder-Derived Odometry

Encoder-based odometry was noisier than expected, especially due to the characteristics of the motors and drivetrain. This made low-pass filtering important for producing more stable velocity and odometry estimates.

### Multi-Core Task Separation

Using both ESP32 cores helped keep the real-time motor control loop more reliable while offloading communication and sensor publishing to the other core.

### PCB Design Retrospective

Looking back with more hardware experience from my later DIY FOC controller project, I would have used a ground copper pour instead of manually routing ground traces point-to-point. In this case the board was simple and low power, so it still worked acceptably, but it was still not the best layout practice.
