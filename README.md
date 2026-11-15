# ROS 2 AMR Sensor Integration
ROS 2 C++ nodes for autonomous mobile robot (AMR) sensor integration: LiDAR, IMU and wheel encoders connected to ARM-based controllers over I2C, SPI and CAN, enabling real-time sensor data acquisition for robot positioning and motion control.

Personal project based on professional AMR embedded experience (no employer code).

## Features
- C++ ROS 2 publishers, subscribers and services connecting sensor, motor-control and navigation components
- Sensor drivers / interfaces for LiDAR, IMU and wheel encoders
- Real-time data paths on Embedded Linux / ARM

## Planned Structure
- `src/` — ROS 2 packages and nodes
- `launch/` — launch files
- `config/` — sensor and controller configuration

## Tech
C++17, ROS 2, Embedded Linux, ARM, I2C, SPI, CAN
