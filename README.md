# 🤖 ROS 2 AMR Sensor Integration

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue) ![ROS 2](https://img.shields.io/badge/ROS%202-Humble%2FDesktop-brightgreen) ![Embedded Linux](https://img.shields.io/badge/Embedded-Linux-orange) ![Build](https://img.shields.io/badge/build-passing-success)

ROS 2 C++ nodes and a standalone sensor-fusion simulation for autonomous mobile robots (AMR): LiDAR, IMU and wheel encoders on ARM controllers over I2C, SPI and CAN — fusing encoder + IMU data to estimate robot pose in real time.

*Personal project based on professional AMR embedded experience at Honeywell — no employer code.*

## ✨ Highlights
- Complementary-filter pose fusion: **0.009 m position error, 0.50° heading error** over a 3 m drive (see sample output below)
- Genuine ROS 2 `rclcpp` node publishing `/scan`, `/imu/data` and `/odom`
- Deterministic simulation (fixed seed) — output is byte-identical on every run

## Code, Build & Sample Output

Real, runnable code — not just a description:

- `src/amr_sensor_sim.cpp` — self-contained C++17 simulation (standard library only): simulated LiDAR (ray distances to the walls of a 6×4 m room), IMU (yaw + acceleration, with bias and noise), and quantised wheel encoders. Encoder + IMU heading are fused with a complementary filter into an estimated pose, logged against ground truth over a 9 s drive (forward → +90° turn → forward → stop). Deterministic: fixed random seed (42), identical output on every run.
- `src/amr_sensor_node.cpp` — the ROS 2 (rclcpp) version: subscribes to `/cmd_vel` and publishes `/scan` (`sensor_msgs/LaserScan`), `/imu/data` (`sensor_msgs/Imu`), and `/odom` (`nav_msgs/Odometry`). Build it with colcon/ament in a ROS 2 Humble/Jazzy workspace (see the header of that file); it is not part of the local Makefile build.

### Build & run (standalone simulation, no ROS 2 needed)

```bash
make            # or: g++ -std=c++17 -Wall -Wextra -O2 -o amr_sensor_sim src/amr_sensor_sim.cpp
make run        # or: ./amr_sensor_sim
```

### Sample output (actual run)

```text
AMR sensor simulation -- room 6.0 x 4.0 m, dt=0.1 s, seed=42
Drive plan: FORWARD 3s -> TURN +90deg 2s -> FORWARD 3s -> STOP 1s
--------------------------------------------------------------------------
[t= 0.0s] FORWARD true=(-1.50,+0.00,   0.0deg) est=(-1.50,+0.00,   0.0deg) lidar[L=2.00 F=4.49 R=2.00]m imu_yaw=  -0.1deg accel=+0.01 enc[L=0 R=0]
[t= 0.5s] FORWARD true=(-1.25,+0.00,   0.0deg) est=(-1.25,+0.00,   0.1deg) lidar[L=2.00 F=4.26 R=2.00]m imu_yaw=   0.1deg accel=-0.02 enc[L=143 R=143]
[t= 1.0s] FORWARD true=(-1.00,+0.00,   0.0deg) est=(-1.00,+0.00,  -0.0deg) lidar[L=1.99 F=4.00 R=2.00]m imu_yaw=  -0.1deg accel=-0.04 enc[L=286 R=286]
[t= 1.5s] FORWARD true=(-0.75,+0.00,   0.0deg) est=(-0.75,+0.00,   0.2deg) lidar[L=2.00 F=3.75 R=1.99]m imu_yaw=   0.2deg accel=-0.02 enc[L=430 R=430]
[t= 2.0s] FORWARD true=(-0.50,+0.00,   0.0deg) est=(-0.50,+0.00,   0.1deg) lidar[L=2.00 F=3.50 R=2.00]m imu_yaw=   0.1deg accel=+0.04 enc[L=573 R=573]
[t= 2.5s] FORWARD true=(-0.25,+0.00,   0.0deg) est=(-0.25,+0.00,   0.2deg) lidar[L=2.00 F=3.25 R=1.99]m imu_yaw=   0.2deg accel=-0.00 enc[L=716 R=716]
[t= 3.0s] TURN    true=(+0.00,+0.00,   0.0deg) est=(-0.00,+0.00,   0.3deg) lidar[L=1.99 F=3.00 R=2.00]m imu_yaw=   0.4deg accel=+0.50 enc[L=859 R=859]
[t= 3.5s] TURN    true=(+0.00,+0.00,  22.5deg) est=(+0.00,+0.00,  22.7deg) lidar[L=2.15 F=3.25 R=2.16]m imu_yaw=  22.7deg accel=-0.01 enc[L=803 R=916]
[t= 4.0s] TURN    true=(+0.00,+0.00,  45.0deg) est=(+0.00,+0.00,  45.2deg) lidar[L=2.83 F=2.82 R=2.83]m imu_yaw=  45.3deg accel=+0.01 enc[L=747 R=972]
[t= 4.5s] TURN    true=(+0.00,+0.00,  67.5deg) est=(-0.00,+0.00,  67.8deg) lidar[L=3.25 F=2.16 R=3.24]m imu_yaw=  67.9deg accel=+0.00 enc[L=691 R=1028]
[t= 5.0s] FORWARD true=(+0.00,+0.00,  90.0deg) est=(-0.00,+0.00,  90.3deg) lidar[L=3.00 F=1.99 R=3.00]m imu_yaw=  90.3deg accel=+0.50 enc[L=634 R=1084]
[t= 5.5s] FORWARD true=(+0.00,+0.25,  90.0deg) est=(-0.00,+0.25,  90.2deg) lidar[L=3.00 F=1.75 R=3.00]m imu_yaw=  90.3deg accel=+0.01 enc[L=778 R=1228]
[t= 6.0s] FORWARD true=(+0.00,+0.50,  90.0deg) est=(-0.00,+0.50,  90.4deg) lidar[L=3.00 F=1.49 R=3.00]m imu_yaw=  90.5deg accel=+0.00 enc[L=921 R=1371]
[t= 6.5s] FORWARD true=(+0.00,+0.75,  90.0deg) est=(-0.00,+0.75,  90.4deg) lidar[L=3.00 F=1.25 R=2.99]m imu_yaw=  90.5deg accel=+0.00 enc[L=1064 R=1514]
[t= 7.0s] FORWARD true=(+0.00,+1.00,  90.0deg) est=(-0.01,+1.00,  90.3deg) lidar[L=3.00 F=1.00 R=3.00]m imu_yaw=  90.4deg accel=+0.00 enc[L=1207 R=1657]
[t= 7.5s] FORWARD true=(+0.00,+1.25,  90.0deg) est=(-0.01,+1.25,  90.2deg) lidar[L=3.00 F=0.76 R=3.01]m imu_yaw=  90.3deg accel=-0.01 enc[L=1351 R=1801]
[t= 8.0s] STOP    true=(+0.00,+1.50,  90.0deg) est=(-0.01,+1.50,  90.2deg) lidar[L=3.00 F=0.49 R=3.01]m imu_yaw=  90.3deg accel=+0.50 enc[L=1494 R=1944]
[t= 8.5s] STOP    true=(+0.00,+1.50,  90.0deg) est=(-0.01,+1.50,  90.5deg) lidar[L=2.99 F=0.50 R=2.99]m imu_yaw=  90.7deg accel=-0.02 enc[L=1494 R=1944]
[t= 9.0s] STOP    true=(+0.00,+1.50,  90.0deg) est=(-0.01,+1.50,  90.5deg) lidar[L=3.00 F=0.50 R=3.00]m imu_yaw=  90.7deg accel=-0.01 enc[L=1494 R=1944]
--------------------------------------------------------------------------
FINAL POSE SUMMARY
  Ground truth : x=+0.000 m  y=+1.500 m  theta=+90.00 deg
  Estimated    : x=-0.008 m  y=+1.503 m  theta=+90.50 deg
  Pose error   : 0.009 m position, 0.50 deg heading
  Distance     : 3.00 m travelled, encoders L=1494 R=1944 ticks
```

The fused estimate finishes within 0.009 m and 0.50° of ground truth. Full untrimmed output is also saved in `sample_output.txt`.


---
*Created by Rohith Reddy Chinthala — Embedded Software Engineer at Honeywell, Saint Louis, MO.*
