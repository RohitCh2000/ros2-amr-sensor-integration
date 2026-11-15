// amr_sensor_sim.cpp
//
// Self-contained C++17 simulation of an AMR (autonomous mobile robot)
// driving in a small rectangular room. Standard library only -- no ROS 2
// required to build or run this file (see amr_sensor_node.cpp for the
// ROS 2 version of the same idea).
//
// Simulated sensors:
//   * LiDAR  -- ray distances to the room walls for a fan of beam angles
//   * IMU    -- yaw (heading) and forward acceleration, with bias + noise
//   * Wheel encoders -- left/right tick counts (quantised, differential drive)
//
// Sensor fusion:
//   Encoder delta-distances give travelled distance; the heading change is
//   a complementary blend of the IMU yaw delta (70%) and the encoder
//   differential heading delta (30%). The fused heading + distance are
//   integrated into an estimated pose (x, y, theta), which is compared with
//   ground truth at the end of the run.
//
// Drive plan (dt = 0.1 s, 9 s total):
//   0-3 s  FORWARD  at 0.50 m/s
//   3-5 s  TURN     in place, +90 deg (counter-clockwise)
//   5-8 s  FORWARD  at 0.50 m/s
//   8-9 s  STOP     decelerate / hold position
//
// Deterministic: all "noise" comes from a std::mt19937 generator with a
// fixed seed (42), so every run prints identical output.
//
// Build:  g++ -std=c++17 -Wall -Wextra -O2 -o amr_sensor_sim src/amr_sensor_sim.cpp
// Run:    ./amr_sensor_sim

#include <array>
#include <cmath>
#include <cstdio>
#include <random>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Room: axis-aligned rectangle, metres.
constexpr double kRoomXMin = -3.0;
constexpr double kRoomXMax = 3.0;
constexpr double kRoomYMin = -2.0;
constexpr double kRoomYMax = 2.0;

// Robot / sensor parameters.
constexpr double kWheelRadius = 0.10;    // m
constexpr double kTrackWidth = 0.50;     // m, distance between wheels
constexpr int kTicksPerRev = 360;        // encoder ticks per wheel revolution
constexpr double kLidarMaxRange = 8.0;   // m
constexpr double kDt = 0.1;              // s, simulation step

double wrap_angle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double deg(double rad) { return rad * 180.0 / kPi; }

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // rad, 0 = +x (east), counter-clockwise positive
};

// Distance from (x, y) along a ray at absolute angle `angle` to the nearest
// room wall, using the slab method for an axis-aligned box.
double lidar_ray(double x, double y, double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double t_min = kLidarMaxRange;

  if (std::fabs(dx) > 1e-9) {
    for (double wall_x : {kRoomXMin, kRoomXMax}) {
      const double t = (wall_x - x) / dx;
      if (t > 0.0) {
        const double hit_y = y + t * dy;
        if (hit_y >= kRoomYMin && hit_y <= kRoomYMax && t < t_min) t_min = t;
      }
    }
  }
  if (std::fabs(dy) > 1e-9) {
    for (double wall_y : {kRoomYMin, kRoomYMax}) {
      const double t = (wall_y - y) / dy;
      if (t > 0.0) {
        const double hit_x = x + t * dx;
        if (hit_x >= kRoomXMin && hit_x <= kRoomXMax && t < t_min) t_min = t;
      }
    }
  }
  return t_min;
}

struct Command {
  double v;      // m/s, forward velocity
  double omega;  // rad/s, yaw rate
  const char* phase;
};

Command command_at(double t) {
  if (t < 3.0) return {0.50, 0.0, "FORWARD"};
  if (t < 5.0) return {0.0, kPi / 4.0, "TURN   "};  // +45 deg/s for 2 s = 90 deg
  if (t < 8.0) return {0.50, 0.0, "FORWARD"};
  return {0.0, 0.0, "STOP   "};
}

}  // namespace

int main() {
  std::mt19937 rng(42);  // fixed seed: deterministic output
  std::normal_distribution<double> lidar_noise(0.0, 0.005);  // 5 mm
  std::normal_distribution<double> imu_noise(0.0, 0.002);    // rad
  std::normal_distribution<double> accel_noise(0.0, 0.02);   // m/s^2

  Pose truth{-1.5, 0.0, 0.0};
  Pose estimate = truth;

  double left_dist_true = 0.0;   // cumulative true wheel travel, m
  double right_dist_true = 0.0;
  long left_ticks_prev = 0;
  long right_ticks_prev = 0;
  double prev_imu_yaw = truth.theta;
  double total_distance = 0.0;

  const double dist_per_tick = 2.0 * kPi * kWheelRadius / kTicksPerRev;

  std::printf("AMR sensor simulation -- room %.1f x %.1f m, dt=%.1f s, seed=42\n",
              kRoomXMax - kRoomXMin, kRoomYMax - kRoomYMin, kDt);
  std::printf("Drive plan: FORWARD 3s -> TURN +90deg 2s -> FORWARD 3s -> STOP 1s\n");
  std::printf("------------------------------------------------------------------"
              "--------\n");

  const int steps = static_cast<int>(9.0 / kDt);
  for (int step = 0; step <= steps; ++step) {
    const double t = step * kDt;
    const Command cmd = command_at(t);

    // --- Ground-truth motion (exact per-step differential-drive update) ---
    // The motion in the interval ending at time t uses the command that
    // was active at the start of that interval, command_at(t - dt), so
    // each drive phase lasts its full stated duration.
    if (step > 0) {
      const Command motion_cmd = command_at(t - kDt);
      const double dtheta = motion_cmd.omega * kDt;
      const double ds = motion_cmd.v * kDt;
      const double mid = truth.theta + dtheta / 2.0;
      truth.x += ds * std::cos(mid);
      truth.y += ds * std::sin(mid);
      truth.theta = wrap_angle(truth.theta + dtheta);
      total_distance += std::fabs(ds);

      // True wheel travel this step.
      const double dl = ds - dtheta * kTrackWidth / 2.0;
      const double dr = ds + dtheta * kTrackWidth / 2.0;
      left_dist_true += dl;
      right_dist_true += dr;
    }

    // --- Encoders: quantised tick counts, then measured wheel deltas ---
    const long left_ticks =
        static_cast<long>(std::llround(left_dist_true / dist_per_tick));
    const long right_ticks =
        static_cast<long>(std::llround(right_dist_true / dist_per_tick));
    const double dl_meas = (left_ticks - left_ticks_prev) * dist_per_tick;
    const double dr_meas = (right_ticks - right_ticks_prev) * dist_per_tick;
    left_ticks_prev = left_ticks;
    right_ticks_prev = right_ticks;

    // --- IMU: yaw with slow bias drift + noise; accel from command change ---
    const double imu_yaw =
        wrap_angle(truth.theta + 0.001 * t + imu_noise(rng));
    const double imu_accel_x = (step == 30 || step == 50 || step == 80)
                                   ? 0.50
                                   : 0.0 + accel_noise(rng);

    // --- Fusion: complementary blend of IMU and encoder heading deltas ---
    if (step > 0) {
      const double ds_meas = (dl_meas + dr_meas) / 2.0;
      const double dtheta_enc = (dr_meas - dl_meas) / kTrackWidth;
      const double dtheta_imu = wrap_angle(imu_yaw - prev_imu_yaw);
      const double dtheta_fused = 0.7 * dtheta_imu + 0.3 * dtheta_enc;
      const double mid = estimate.theta + dtheta_fused / 2.0;
      estimate.x += ds_meas * std::cos(mid);
      estimate.y += ds_meas * std::sin(mid);
      estimate.theta = wrap_angle(estimate.theta + dtheta_fused);
    }
    prev_imu_yaw = imu_yaw;

    // --- LiDAR: beams at -90, -45, 0, +45, +90 deg relative to heading ---
    const std::array<double, 5> beam_offsets = {-kPi / 2, -kPi / 4, 0.0,
                                                kPi / 4, kPi / 2};
    std::array<double, 5> beams{};
    for (std::size_t i = 0; i < beam_offsets.size(); ++i) {
      beams[i] = lidar_ray(truth.x, truth.y, truth.theta + beam_offsets[i]) +
                 lidar_noise(rng);
    }

    // --- Log every 0.5 s ---
    if (step % 5 == 0) {
      std::printf(
          "[t=%4.1fs] %s true=(%+.2f,%+.2f,%6.1fdeg) est=(%+.2f,%+.2f,%6.1fdeg) "
          "lidar[L=%.2f F=%.2f R=%.2f]m imu_yaw=%6.1fdeg accel=%+.2f "
          "enc[L=%ld R=%ld]\n",
          t, cmd.phase, truth.x, truth.y, deg(truth.theta), estimate.x,
          estimate.y, deg(estimate.theta), beams[4], beams[2], beams[0],
          deg(imu_yaw), imu_accel_x, left_ticks, right_ticks);
    }
  }

  const double pos_err =
      std::hypot(estimate.x - truth.x, estimate.y - truth.y);
  const double head_err = std::fabs(deg(wrap_angle(estimate.theta - truth.theta)));

  std::printf("------------------------------------------------------------------"
              "--------\n");
  std::printf("FINAL POSE SUMMARY\n");
  std::printf("  Ground truth : x=%+.3f m  y=%+.3f m  theta=%+.2f deg\n", truth.x,
              truth.y, deg(truth.theta));
  std::printf("  Estimated    : x=%+.3f m  y=%+.3f m  theta=%+.2f deg\n",
              estimate.x, estimate.y, deg(estimate.theta));
  std::printf("  Pose error   : %.3f m position, %.2f deg heading\n", pos_err,
              head_err);
  std::printf("  Distance     : %.2f m travelled, encoders L=%ld R=%ld ticks\n",
              total_distance, left_ticks_prev, right_ticks_prev);
  return 0;
}
