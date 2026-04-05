# Kalman Filter Formulas Used in This Estimator

## Prediction (IMU callback)

State transition model (x and y axes):
```
F = [[1, dt], [0, 1]]
W = [0.5*dt^2, dt]^T

X_new = F * X + W * a
P_new = F * P * F^T + W_full * Q * W_full^T
```

For x-axis, the process noise matrix accounts for rotation:
```
Wx = [[0.5*dt^2*cos(yaw), -0.5*dt^2*sin(yaw)],
      [dt*cos(yaw),        -dt*sin(yaw)       ]]
Qxy = diag(var_imu_x, var_imu_y)
Px_new = F*Px*F^T + Wx*Qxy*Wx^T
```

Body → World acceleration rotation:
```
ax = cos(yaw)*ux - sin(yaw)*uy
ay = sin(yaw)*ux + cos(yaw)*uy
az = uz - GRAVITY   (clamped to ±1.5 m/s²)
```

Yaw prediction (angular velocity direct):
```
Fa = [[1, 0], [0, 0]]
Wa = [dt, 1]^T
Xa_new = Fa*Xa + Wa*uyaw
Xa(0) = limitAngle(Xa(0))
Pa_new = Fa*Pa*Fa^T + Wa*var_imu_a*Wa^T
```

z (3D for baro bias):
```
Fz = [[1, dt, 0], [0, 1, 0], [0, 0, 1]]
Wz = [0.5*dt^2, dt, 0]^T
Xz_new = Fz*Xz + Wz*az
Pz_new = Fz*Pz*Fz^T + Wz*var_imu_z*Wz^T
```

## Scalar Correction (Joseph form for numerical stability)

For a scalar measurement Y with noise variance R and observation H:
```
innovation = Y - H*X
S = H*P*H^T + R
K = P*H^T / S
X_new = X + K*innovation
P_new = (I - K*H)*P*(I - K*H)^T + K*R*K^T
```

The Joseph form `(I-KH)P(I-KH)^T + KRK^T` is used (not the simpler `(I-KH)P`) for better numerical stability.

## GPS

ECEF conversion:
```
e^2 = 1 - (b/a)^2
N(phi) = a / sqrt(1 - e^2 * sin^2(phi))
ECEF = [(N+h)*cos(phi)*cos(lam),
        (N+h)*cos(phi)*sin(lam),
        (b^2/a^2*N + h)*sin(phi)]
```

NED from ECEF delta:
```
R_e_n = [[-sin_lat*cos_lon, -sin_lon, -cos_lat*cos_lon],
          [-sin_lat*sin_lon,  cos_lon, -cos_lat*sin_lon],
          [ cos_lat,          0,       -sin_lat         ]]
ned = R_e_n^T * (ECEF - ECEF_0)
```

NED → ENU (Gazebo world frame):
```
R_m_n = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
Ygps = R_m_n * ned + initial_position
```

## Magnetic Yaw

```
Ymagnet = limitAngle(atan2(-my, mx))
```
(Gazebo magnetic north → +x, so no extra heading offset needed)

Angular correction wraps innovation:
```
innovation = limitAngle(Ymagnet - yaw_est)
```

## Barometer

Barometer measures `z + bias`:
```
Hbaro = [1, 0, 1]
Ybaro = 44330 * (1 - (P / 101325)^0.1903)
```
Bias initialized on first frame: `Xz(2) = Ybaro - Xz(0)`.
