# EE4308 Project 2 - Overview for Claude

## What This Project Is

EE4308 Autonomous Robot Systems (AY25/26 Sem 2, NUS).
A drone simulation project using a simplified Kalman Filter estimator.
The user is responsible for the **estimator** part.

## Key Files

| File | Role |
|---|---|
| `src/ee4308_drone/src/estimator.cpp` | Main estimator implementation |
| `src/ee4308_drone/include/ee4308_drone/estimator.hpp` | State vectors, parameters, constants |
| `src/ee4308_bringup/params/proj2.yaml` | Runtime parameters (no rebuild needed) |
| `src/ee4308_bringup/params/proj2_gt.yaml` | Same params but `use_ground_truth: true` |
| `docs/proj2.md` | Assignment handout |
| `docs/estimator_plan_cn.md` | Chinese planning/implementation log |
| `docs/estimator_plan.md` | English version of same |

## How to Run

Inside Docker `ee4308_jazzy_proj2`:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws/ee4308
colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup
source install/setup.bash

# With real estimator:
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True param_file:=proj2

# With ground truth (test behavior/controller only):
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=True param_file:=proj2_gt
```

Kill residual processes before each clean run:
```bash
pkill -9 -f "[p]roj2_sim.launch.py" || true
pkill -9 -f "[r]viz2" || true
pkill -9 -f "[c]omponent_container" || true
pkill -9 -f "[g]z sim" || true
```

## State Design

Four independent 2D states (+ Xz_ is 3D for baro bias augmentation):

| State | Meaning |
|---|---|
| `Xx_ = [x, ẋ]` | World-frame x position & velocity |
| `Xy_ = [y, ẏ]` | World-frame y position & velocity |
| `Xz_ = [z, ż, bias]` | World-frame z + barometer bias |
| `Xa_ = [yaw, ẏaw]` | Yaw angle & rate |

Covariances: `Px_` (2x2), `Py_` (2x2), `Pz_` (3x3), `Pa_` (2x2)

## Sensor Callbacks

| Callback | Sensor | What it corrects |
|---|---|---|
| `callbackSubIMU_()` | IMU | Prediction: all axes |
| `callbackSubGPS_()` | GPS | x, y, z correction |
| `callbackSubSonar_()` | Sonar | z correction (low altitude only, ≤3.8m) |
| `callbackSubMagnetic_()` | Magnetometer | yaw correction |
| `callbackSubBaro_()` | Barometer | z + bias correction (optional, 3-person team) |

## Current Implementation Status (as of 2026-04-05)

All core callbacks implemented:
- IMU prediction: world-frame rotation + constant-acceleration model
- GPS correction: ECEF → NED → ENU (Gazebo world frame)
- Magnetic correction: atan2(-my, mx) for yaw
- Sonar correction: low-altitude gating + innovation gating
- Baro correction: bias-augmented state [z, ż, bias], H=[1,0,1]
- GPS velocity pseudo-measurement: finite-difference velocity correction

## Known Issues / Active Work

- x,y lag bug: GPS position correction lags behind truth. Latest commit "try to fix x,y lag bug".
- The `aligned_score` metric from param sweeps: lower is better (smaller MAE).
- Best known xy params (from xy_run3 sweep): `var_gps_x/y=0.4`, `var_imu_x/y=3.0-3.5`.
- z is stable: `mae_z ≈ 0.017–0.025` across all sweep cases.

## Parameter Sweep Infrastructure

- `tools/run_proj2_xy_sweep.py`: runs multiple param combinations, outputs per-case CSVs + summary
- Results in `tmp/proj2_param_sweeps/`
- Scoring: `aligned_score` = weighted MAE (lower = better)
- Key window: `w10_18` = 10s–18s after launch (first cruise phase)
