# Estimator Plan for Project 2

## 1. Current baseline

We are now on branch `proj2-estimator`, based on the integrated `behavior + controller` code.

Current files already in place:

- `src/ee4308_drone/src/behavior.cpp`
- `src/ee4308_drone/src/controller.cpp`
- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`
- `src/ee4308_bringup/params/proj2.yaml`
- `src/ee4308_bringup/params/proj2_gt.yaml`

Important parameter status:

- `proj2.yaml` is the normal project config.
- `proj2.yaml` already keeps the required controller settings from the handout:
  - `max_z_vel: 0.5`
  - `yaw_vel: -0.3`
- `proj2_gt.yaml` is not a different controller config.
- `proj2_gt.yaml` only exists to keep the same controller/behavior settings while changing:
  - `drone.estimator.use_ground_truth: true`

That means:

- Use `proj2_gt.yaml` when validating `behavior/controller` independently of estimator quality.
- Use `proj2.yaml` when testing the real estimator.

This matches the handout statement that `use_ground_truth` is useful to troubleshoot `behavior` and `controller`.

## 2. What the teacher is likely to care about

From `docs/notes/email.md`, the teacher is not only looking for code that works. The stronger submission will:

- include equations where possible
- explain why and how a change improves performance
- justify tuned values with deliberately designed experiments
- compare alternatives, not just describe results vaguely
- avoid claiming features that were not actually implemented
- mention failed attempts only if there is meaningful analysis

For our estimator work, this means the report and experiments should answer:

- Why each sensor is modelled the way it is.
- Why each variance was tuned up or down.
- Which error source each correction is compensating for.
- Under what situation a sensor helps or hurts.

## 3. Scope of estimator work

According to `docs/proj2.md`, the estimator part is split into prediction and correction.

### Required / core

1. `Estimator::callbackSubIMU_()`
2. `Estimator::callbackSubGPS_()`
3. `Estimator::getECEF_()`
4. `Estimator::callbackSubMagnetic_()`
5. `Estimator::callbackSubSonar_()` check and clean up from Lab 2

### Optional for a team of 3

6. `Estimator::callbackSubBaro_()`

For a team of 3, barometer is optional in the handout. It should be treated as a later improvement, not as the first milestone.

## 4. Code map

### Main estimator implementation

- `src/ee4308_drone/src/estimator.cpp`
  - prediction and correction logic
  - sensor callbacks
  - odom publishing
  - verbose diagnostic printing

- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`
  - state vectors
  - covariance matrices
  - sensor measurement cache
  - parameters
  - constants

### Parameters and integration

- `src/ee4308_bringup/params/proj2.yaml`
  - final project configuration
  - estimator variances to tune

- `src/ee4308_bringup/params/proj2_gt.yaml`
  - same behavior/controller settings
  - temporary integration config with `use_ground_truth: true`

### Upstream dependencies of estimator output

- `src/ee4308_drone/src/behavior.cpp`
  - consumes `/drone/odom`
  - depends on estimated position to decide transitions

- `src/ee4308_drone/src/controller.cpp`
  - consumes `/drone/odom`
  - depends on estimated pose for path tracking

## 5. State and measurement design

The current estimator keeps four independent 2D state vectors:

- `Xx_ = [x, x_dot]^T`
- `Xy_ = [y, y_dot]^T`
- `Xz_ = [z, z_dot]^T`
- `Xa_ = [yaw, yaw_dot]^T`

Their covariance matrices are:

- `Px_`, `Py_`, `Pz_`, `Pa_`

Sensor-side cached measurements for terminal output:

- `Ygps_`
- `Ysonar_`
- `Ymagnet_`
- `Ybaro_`

This structure is already compatible with the handout for:

- IMU prediction on `x`, `y`, `z`, `yaw`
- GPS correction on `x`, `y`, `z`
- sonar correction on `z`
- magnetic correction on `yaw`

If barometer bias augmentation is later implemented, `Xz_` and `Pz_` would need resizing and the `z`-related math would need to change consistently.

## 6. Function-by-function plan

### 6.1 `callbackSubIMU_()`

Goal:

- extend the Lab 2 prediction from `z` only to `x`, `y`, `z`, and `yaw`

Inputs used:

- `msg.linear_acceleration.x`
- `msg.linear_acceleration.y`
- `msg.linear_acceleration.z`
- `msg.angular_velocity.z`
- current yaw estimate `Xa_(0)`
- `dt`

Required formulas:

1. Rotate body-frame acceleration to world frame while ignoring roll and pitch:

```math
\begin{bmatrix} a_x \\ a_y \end{bmatrix}
=
\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix} u_x \\ u_y \end{bmatrix}
```

2. Constant-acceleration prediction for `x`, `y`, `z`:

```math
F =
\begin{bmatrix}
1 & dt \\
0 & 1
\end{bmatrix}, \quad
W =
\begin{bmatrix}
\frac{1}{2}dt^2 \\
dt
\end{bmatrix}
```

3. For `z`, remove gravity:

```math
a_z = u_z - G
```

4. For yaw:

```math
\hat{X}_{a,k|k-1} =
\begin{bmatrix}
\psi_{k-1|k-1} + dt \cdot u_\psi \\
u_\psi
\end{bmatrix}
```

Implementation notes:

- `x` and `y` process noise comes from a 2x2 diagonal IMU covariance in body frame:
  - `Qxy = diag(var_imu_x_, var_imu_y_)`
- `z` process noise uses `var_imu_z_`
- `yaw` process noise uses `var_imu_a_`
- after yaw prediction, wrap angle with `ee4308::limitAngle()`

Code sections involved:

- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/core.hpp`

### 6.2 `getECEF_()`

Goal:

- convert GPS latitude, longitude, altitude to ECEF

Required formulas:

```math
e^2 = 1 - \frac{b^2}{a^2}
```

```math
N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2(\varphi)}}
```

```math
\begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} =
\begin{bmatrix}
(N+h)\cos\varphi\cos\lambda \\
(N+h)\cos\varphi\sin\lambda \\
\left(\frac{b^2}{a^2}N+h\right)\sin\varphi
\end{bmatrix}
```

Inputs used:

- `RAD_EQUATOR`
- `RAD_POLAR`
- `sin_lat`, `cos_lat`
- `sin_lon`, `cos_lon`
- `alt`

Code section involved:

- `src/ee4308_drone/src/estimator.cpp`

### 6.3 `callbackSubGPS_()`

Goal:

- correct `x`, `y`, `z` in world frame using GPS

Required steps:

1. Convert latitude and longitude to radians.
2. Compute ECEF using `getECEF_()`.
3. On the first GPS message:
  - initialize `initial_ECEF_`
  - return without correction
4. Convert ECEF delta to local NED.
5. Rotate NED to Gazebo/world frame using the handout matrix.
6. Add `initial_position_` to get world-frame measurement.
7. Store result in `Ygps_`.
8. Correct `Xx_`, `Xy_`, `Xz_` separately with scalar measurement models.

Required formulas:

```math
\mathbf{p}_{ned} = R_{e/n}^T ( \mathbf{p}_{ecef} - \mathbf{p}_{ecef,0} )
```

```math
\mathbf{p}_{gps} = R_{m/n} \mathbf{p}_{ned} + \mathbf{p}_0
```

For each axis, use:

```math
H = [1 \; 0], \quad V = 1, \quad R = \sigma^2
```

Implementation notes:

- do not mix the three axes into one big matrix; the current code structure is axis-wise
- use one scalar Kalman correction per axis
- `var_gps_x_`, `var_gps_y_`, `var_gps_z_` should stay separate

Code sections involved:

- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/estimator.hpp`

### 6.4 `callbackSubMagnetic_()`

Goal:

- correct yaw using the compass

Required measurement model:

```math
Y_{mag} = \psi_{mag} + \varepsilon
```

```math
H = [1 \; 0], \quad V = 1, \quad R = \sigma^2_{mag}
```

Measurement construction:

- Gazebo magnetic north behaves like it points toward world `+x`
- because the drone starts facing `+x`, no initial heading offset is needed
- use `atan2()` on the magnetic vector in the drone frame

Implementation notes:

- store the measurement in `Ymagnet_`
- wrap angle differences with `ee4308::limitAngle()`
- for yaw correction, the innovation must also be angle-wrapped

Code sections involved:

- `src/ee4308_drone/src/estimator.cpp`
- `src/ee4308_drone/include/ee4308_drone/core.hpp`

### 6.5 `callbackSubSonar_()`

Goal:

- keep the Lab 2 `z` correction, but remove Lab 2-only hacks before final tuning

Current status:

- the basic scalar Kalman correction is already present
- however, the current code still contains this Lab 2-only block:
  - resetting `Px_` and `Py_` to small constants

That block should not stay in the final project estimator because it artificially modifies unrelated axes.

Required measurement model:

```math
H = [1 \; 0], \quad V = 1, \quad R = \sigma^2_{sonar}
```

Implementation notes:

- keep the out-of-range guard
- keep writing `Ysonar_` for verbose output
- remove the Lab 2 covariance reset after the base estimator is working

### 6.6 `callbackSubBaro_()` (optional)

Goal:

- optionally add another `z` correction source

Important note:

- for a team of 3, this is optional
- the handout explicitly warns that barometer altitude can have strong bias
- the mathematically cleaner version is not a simple offset subtraction, but augmenting the `z` state with a bias term

Recommendation:

- do not do barometer first
- only implement it after the base estimator is already stable
- if time is limited, better do a strong GPS + sonar + magnet estimator and a good experiment section

## 7. Recommended implementation order

### Phase 0: keep integration stable

1. keep `proj2.yaml` as the normal config
2. keep `proj2_gt.yaml` for behavior/controller-only verification
3. use temporary logs in `behavior.cpp` and `controller.cpp` while estimator is under development

### Phase 1: finish prediction

1. complete `callbackSubIMU_()` for `x`, `y`, `z`, `yaw`
2. verify `/drone/odom` updates smoothly with only IMU prediction
3. temporarily disable or downplay interpretation of correction quality until prediction is sane

### Phase 2: GPS position correction

1. implement `getECEF_()`
2. implement `callbackSubGPS_()`
3. verify that `Ygps_` is in the same world frame as the drone pose
4. check that GPS pulls long-term drift back instead of causing jumps in the wrong axis

### Phase 3: yaw correction

1. implement `callbackSubMagnetic_()`
2. verify that yaw drift reduces and the controller still tracks paths properly

### Phase 4: clean sonar and tune the base estimator

1. keep sonar correction active
2. remove the Lab 2 covariance reset in `callbackSubSonar_()`
3. tune:
  - `var_imu_x_`, `var_imu_y_`, `var_imu_z_`, `var_imu_a_`
  - `var_gps_x_`, `var_gps_y_`, `var_gps_z_`
  - `var_sonar_`
  - `var_magnet_`

### Phase 5: optional barometer improvement

1. decide whether time is sufficient
2. if yes, design the augmented `z` state properly
3. if not, stop here and focus on experiments, tuning, and report quality

## 8. Validation plan

### A. Behavior/controller isolation

Use:

- `proj2_gt.yaml`

Purpose:

- confirm any flight failure is not caused by the estimator

Expected:

- takeoff to cruise height
- rotate clockwise at about `-0.3 rad/s`
- cycle through turtle position, turtle goal, and initial air position
- complete cycle before landing after turtle stops

### B. Estimator-only debugging

Use:

- `proj2.yaml`
- optionally disable controller and behavior when collecting sensor data

Purpose:

- inspect whether estimated pose reacts correctly to each sensor

Suggested checks:

- `Pose` and `ErrPose` in verbose output
- `GPS`, `Sonar`, `Magnt`, `Baro` lines
- `ros2 topic echo /drone/odom`

### C. Sensor-by-sensor reasoning

Use experiments that isolate one question at a time:

- IMU only: how fast does drift accumulate
- IMU + sonar: how well does altitude stay bounded
- IMU + GPS: how much planar drift is corrected
- IMU + magnet: how much yaw drift is corrected
- full base estimator: whether behavior/controller still complete the mission

This is the type of experimental structure the teacher feedback is encouraging.

## 9. Concrete execution checklist

1. compile after adding temporary logs
2. run one more GT integration check and confirm logs are readable
3. implement `callbackSubIMU_()`
4. compile and run with `proj2.yaml`
5. implement `getECEF_()` and `callbackSubGPS_()`
6. compile and run again
7. implement `callbackSubMagnetic_()`
8. remove the sonar Lab 2 covariance hack
9. start tuning and recording results
10. decide whether barometer is worth the time

## 10. Notes for report writing

The estimator section of the report should not just say:

- "we tuned the variances"
- "the result is more stable"

It should instead say things like:

- increasing `var_imu_z_` reduces trust in vertical acceleration integration, so sonar dominates altitude correction more strongly
- decreasing `var_sonar_` makes altitude respond more aggressively to range readings, but may amplify measurement noise
- GPS mainly fixes long-term position drift, not short-term smoothness
- magnetometer mainly bounds yaw drift, which indirectly improves the world-to-body velocity transform used by the controller

That style is much closer to what the feedback is asking for.
