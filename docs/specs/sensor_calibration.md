# Sensor Calibration Notes

This document captures the covariance values currently used in
`TopicPublisher` and explains their origins.  It is the starting point for
future empirical calibration work (see "Future Work" section at the bottom).

---

## BNO055 IMU (Board 2, NDOF fusion mode)

### Source

- **BNO055 datasheet** BST-BNO055-DS000-14, Bosch Sensortec:
  - Heading accuracy: ±1° (fully calibrated), ±2–3° typical before calibration
  - Gyro RMS noise (NDOF mode): ~0.05 dps
  - Accelerometer zero-g offset: ≤ 80 mg (≈ 0.78 m/s²)
- **flynneva/bno055 ROS 2 driver** (GitHub, widely used community driver):
  uses the same diagonal values listed below and cites the datasheet as source.
- **imu_tools / robot_localization wiki** (ROS Answers, multiple threads):
  community consensus for a partially-calibrated BNO055 is 0.01–0.02 rad²
  for orientation and 0.001–0.005 (rad/s)² for angular velocity.

### Values used (conservative / pre-calibration)

| Field                            | Diagonal value | σ (1-sigma)       | Justification                                         |
| -------------------------------- | -------------: | ----------------: | ----------------------------------------------------- |
| `orientation_covariance`         | **0.01 rad²**  | ≈ 5.7°            | Safe pre-calibration estimate; covers ±2–3° BNO055 spec plus margin |
| `angular_velocity_covariance`    | **0.0025 (rad/s)²** | ≈ 0.05 rad/s | Accounts for gyro noise + quantization; matches flynneva/bno055 driver |
| `linear_acceleration_covariance` | **0.04 (m/s²)²**   | ≈ 0.2 m/s²  | Conservative; covers accel zero-g offset + dynamic mounting noise |

All three are diagonal 3×3 matrices.  Off-diagonal elements are 0.

> **Note:** The BNO055 gyro noise floor in NDOF mode is far lower than the
> values above (~7.6 × 10⁻⁷ (rad/s)² from the datasheet).  The inflated values
> are intentional: they reflect the combined effect of thermal drift, sensor
> fusion latency, vibration coupling from motors, and the fact that the sensor
> is typically **not** fully calibrated on every boot.  Reduce these values
> after running the empirical calibration procedure described in "Future Work".

---

## Wheel Odometry (differential drive, Board 1)

### Source

- **REP-105** ("Coordinate Frames for Mobile Platforms", ROS Enhancement
  Proposal): guidance on how to populate constrained vs free axes.
- **robot_localization documentation** and **Nav2 TurtleBot3 tutorial**:
  widely-cited starting point is diagonal [0.001, 0.001, 1e-9, 1e-9, 1e-9,
  0.001] for pose (x, y, z, roll, pitch, yaw) and [0.01, 1e-9, 1e-9, 1e-9,
  1e-9, 0.01] for twist (vx, vy, vz, wx, wy, wz).
- **Practical note (ROS Discourse, nav2 thread):** values that are too small
  (over-confident odometry) cause the EKF to ignore other sensors; values
  that are too large cause noisy localisation.  0.001 / 0.01 is a good
  middle ground for indoor hard-floor operation.

### Values used

#### `pose.covariance` (6×6 row-major; order: x, y, z, roll, pitch, yaw)

| Index | Axis  | Value      | Reason                                              |
| ----: | ----- | ---------: | --------------------------------------------------- |
|     0 | x     | **0.001 m²**  | Open-loop forward error; ~3 cm/σ per update       |
|     7 | y     | **0.001 m²**  | Lateral error (differential drive has small y drift) |
|    14 | z     | **1e-9**   | Ground-plane constraint; z is always ≈ 0           |
|    21 | roll  | **1e-9**   | Ground-plane constraint                             |
|    28 | pitch | **1e-9**   | Ground-plane constraint                             |
|    35 | yaw   | **0.001 rad²** | ≈ 1.8°/σ heading error per update              |

All other (off-diagonal) elements are 0.

#### `twist.covariance` (6×6 row-major; order: vx, vy, vz, wx, wy, wz)

| Index | Axis | Value         | Reason                                              |
| ----: | ---- | ------------: | --------------------------------------------------- |
|     0 | vx   | **0.01 (m/s)²**  | Forward velocity encoder noise; ≈ 0.1 m/s σ    |
|     7 | vy   | **1e-9**      | No sideslip for differential drive                  |
|    14 | vz   | **1e-9**      | Vertical velocity constrained to 0                  |
|    21 | wx   | **1e-9**      | Roll rate constrained to 0 on flat ground           |
|    28 | wy   | **1e-9**      | Pitch rate constrained to 0 on flat ground          |
|    35 | wz   | **0.01 (rad/s)²** | Angular velocity encoder noise; ≈ 0.1 rad/s σ |

---

## Future Work — Empirical Calibration

The values above are reasonable starting points but have **not** been validated
against measured data from Sigyn's actual hardware.  The following tasks are
needed to replace them with calibrated values.

### IMU (BNO055)

1. **Allan deviation analysis** (gyro & accel noise characterisation):
   - Record a static 2-hour IMU log (robot stationary, motors off).
   - Run `allan_variance_ros` or the Python `allantools` library on the raw
     gyro and accel streams.
   - Read off angle random walk (ARW) and velocity random walk (VRW) from the
     one-second cluster; use those to set `angular_velocity_covariance` and
     `linear_acceleration_covariance`.
   - Reference: IEEE Std 952-1997 "Specification Format Guide and Test
     Procedure for Single-Axis Interferometric Fiber Optic Gyros".

2. **Orientation accuracy measurement**:
   - Mount the robot on a precision turntable (or use a surveyor's mark).
   - Record BNO055 quaternion output at 0°, 45°, 90°, 180°, 270° over
     multiple runs.
   - Compute RMS error → set `orientation_covariance[0,4,8]` = variance.

3. **Temperature sensitivity**:
   - Repeat Allan deviation test with motors running to introduce thermal load.
   - Inflate gyro/accel covariance by the observed degradation factor.

### Wheel Odometry

1. **Repeatability runs on a measured track**:
   - Drive 2 m forward and back 20 times; measure final position error.
   - Drive in a 1 m radius circle 20 times; measure cumulative heading error.
   - Fit variance to the measured errors to obtain `kOdomPoseFree` and
     `kOdomTwistFree`.

2. **robot_localization EKF tuning**:
   - Start with the current conservative values and run the robot with a
     known ground-truth source (e.g., April-tag fiducials or LiDAR scan-match).
   - Tune both odometry and IMU covariances using the `rqt_robot_localization`
     diagnostic plugin and the NIS (Normalised Innovation Squared) metric.
   - When NIS ≈ 1, the covariances are consistent with the actual errors.

3. **Consult "A Tutorial on SE(2) Estimation" (Barfoot, Forbes, Furgale)**
   for the proper formulation of pose covariance for planar mobile robots.
