# Estimator Current Status & Issues

Last updated: 2026-04-05

## What's Working

1. **IMU Prediction** — correct body→world rotation for x/y, constant-accel model
   - Uses `Fa` matrix with 0 on diagonal[1,1] to zero out old yaw rate, then sets to `uyaw`
   - Vertical accel clamped to ±1.5 m/s² to suppress IMU spikes
   - yaw wrapped with `limitAngle()` after each step

2. **GPS Correction** — ECEF → NED → ENU pipeline
   - First frame only initializes `initial_ECEF_`
   - Subsequent frames: `Ygps = R_m_n * R_e_n^T * (ECEF - ECEF_0) + initial_position`
   - Separate 1D KF corrections for x, y, z
   - **GPS velocity pseudo-measurement** (`maybeApplyGPSVelocityCorrection_`): finite-difference velocity with EMA filter

3. **Sonar Correction** — z axis only
   - Trusted only when `z ≤ 3.8m` AND `|innovation| ≤ 1.0m`
   - Lab2 hack (resetting Px_/Py_) was already removed

4. **Magnetic Correction** — yaw axis
   - `Ymagnet = limitAngle(atan2(-my, mx))`
   - Note: Gazebo magnetic north points +x (bug), so no extra offset needed
   - Uses `applyAngularCorrection` which wraps innovation with `limitAngle`

5. **Barometer Correction** — z + bias
   - Augmented state: `Xz_ = [z, ż, bias]`, `Hbaro = [1, 0, 1]`
   - First frame initializes bias as `Ybaro - z_est`
   - Correction updates all 3 components

## Current Main Issue: x,y Lag

The estimator's x/y position lags behind the ground truth, especially during fast horizontal movement.

### Observed symptoms
- MAE x ≈ 0.4–0.65 m (from sweep results)
- MAE y ≈ 0.1–0.26 m
- The lag is worse for x than y

### Likely causes
1. GPS rate is low (~1 Hz), so prediction drifts between corrections
2. IMU accelerometer x/y noise is relatively large
3. GPS velocity pseudo-measurement may not be aggressive enough
4. GPS position variance may still be too large (trusting IMU too much)

### What has been tried
- Varying `var_gps_x/y` from 0.35 to 0.45 and `var_imu_x/y` from 2.5 to 3.5
- Best from xy_run3: `var_gps_x/y=0.4`, `var_imu_x/y=3.0`, score=0.644
- Current proj2.yaml: `var_imu_x/y=3.0`, `var_gps_x/y=0.4`

### Ideas not yet tried
- Increase GPS trust even more (lower `var_gps_x/y` below 0.35)
- Reduce `gps_velocity_variance_scale` or `gps_velocity_min_variance` for more aggressive velocity correction
- Tune `gps_velocity_max_innovation` threshold
- Check if the GPS velocity correction is actually firing frequently

## z Status

z is already good: MAE ≈ 0.017–0.025 m across sweep cases.
Barometer correction is implemented and helping.

## yaw Status

yaw error is small (≈ 0.004–0.008 rad), not a priority.

## What to Do Next

Priority order:
1. Diagnose whether GPS velocity correction is actually firing (check TMP LOG in output)
2. Try more aggressive GPS trust: `var_gps_x/y` in range 0.2–0.35
3. Check if `gps_velocity_alpha` (currently 0.6) EMA is causing delay
4. If x/y lag persists, consider whether roll/pitch compensation would help
