# Document 1: `computeVelocityCommands()` Code Explanation and Logic Breakdown

This document explains the implemented Pure Pursuit controller in `computeVelocityCommands()` in a report-ready, step-by-step format.

## Step 1: Path Verification

The function first checks whether `global_plan_.poses` is empty.

- If empty, it immediately returns zero command using `writeCmdVel(0, 0)`.
- This prevents undefined behavior when no path exists.

Key variable:
- `global_plan_`

## Step 2: Robot Pose in Map Frame

The input robot pose arrives in odom frame (`rbt_pose_odom`). It is transformed into map frame:

\[
\texttt{tf\_->transform(rbt\_pose\_odom, rbt\_pose, "map")}
\]

This is necessary because path points in `global_plan_` are in map frame, so all distance computations must use a common frame.

Key variable:
- `rbt_pose`

## Step 3: Goal Proximity Check

The goal pose is the last waypoint:

\[
\texttt{goal\_pose = global\_plan\_.poses.back()}
\]

Distance to goal is computed by:

\[
d_{\text{goal}}=\sqrt{(x_g-x_r)^2+(y_g-y_r)^2}
\]

If \(d_{\text{goal}} < \texttt{xy\_goal\_thres\_}\), the robot stops with \((v,\omega)=(0,0)\).

Key variables:
- `goal_pose`
- `xy_goal_thres_`

## Step 4: Find the Closest Path Point

The controller scans all points in `global_plan_.poses` and finds the index `closest_idx` with minimum Euclidean distance to the robot.

\[
i^\*=\arg\min_i \sqrt{(x_i-x_r)^2+(y_i-y_r)^2}
\]

This gives the local anchor for searching a forward lookahead point.

Key variables:
- `closest_idx`
- `min_dist`

## Step 5: Find the Lookahead Point

Starting at `closest_idx`, the controller searches forward for the first path point whose distance to the robot is at least `desired_lookahead_dist_`.

\[
\sqrt{(x_i-x_r)^2+(y_i-y_r)^2}\ge \texttt{desired\_lookahead\_dist\_}
\]

If no such point exists before the end of the plan, the controller uses `goal_pose` as fallback.

Key variables:
- `lookahead_pose`
- `desired_lookahead_dist_`

## Step 6: Coordinate Transformation (Map Frame to Robot Frame)

Let the robot heading be:

\[
\psi = \texttt{rbt\_yaw} = \texttt{ee4308::getYawFromQuaternion(rbt\_pose.pose.orientation)}
\]

Relative lookahead vector in map frame:

\[
dx=x_l-x_r,\quad dy=y_l-y_r
\]

To express this in robot frame \((x',y')\), rotate by \(-\psi\):

\[
\begin{bmatrix}
x'\\
y'
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi\\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
dx\\
dy
\end{bmatrix}
\]

Which matches the code:

\[
x' = dx\cos\psi + dy\sin\psi,\quad
y' = -dx\sin\psi + dy\cos\psi
\]

Interpretation:
- \(x' > 0\): lookahead point is in front of robot.
- \(y' > 0\): point is on robot's left.
- \(y' < 0\): point is on robot's right.

Key variables:
- `rbt_yaw`
- `dx`, `dy`
- `x_rbt`, `y_rbt`

## Step 7: Curvature and Velocity Computation

Define:

\[
L^2 = x'^2 + y'^2
\]

Then Pure Pursuit curvature:

\[
\gamma=\frac{2y'}{L^2}
\]

Command generation:

\[
v=\texttt{desired\_linear\_vel\_},\quad
\omega=v\gamma
\]

A numerical guard is used: if \(L^2\) is too small (`ee4308::THRES`), return zero command to avoid division by zero.

Key variables:
- `desired_linear_vel_`
- `curvature`
- `linear_vel`
- `angular_vel`

## Step 8: Velocity Constraints and Output

The controller applies limits:

\[
v \leftarrow \text{clamp}(v, 0, \texttt{max\_linear\_vel\_})
\]
\[
\omega \leftarrow \text{clamp}(\omega, -\texttt{max\_angular\_vel\_}, \texttt{max\_angular\_vel\_})
\]

Finally, it returns:

\[
\texttt{writeCmdVel}(v,\omega)
\]

Key variables:
- `max_linear_vel_`
- `max_angular_vel_`

## Curvature Geometry and Turning Radius

Pure Pursuit assumes the robot follows a circular arc to the lookahead point. If arc radius is \(R\), then curvature is:

\[
\gamma=\frac{1}{R}
\]

From robot-frame geometry:

\[
x'^2+(y'-R)^2=R^2
\]
\[
x'^2+y'^2-2Ry'=0
\]
\[
R=\frac{x'^2+y'^2}{2y'}=\frac{L^2}{2y'}
\]

Therefore:

\[
\gamma=\frac{1}{R}=\frac{2y'}{L^2}
\]

This shows why sign of \(y'\) determines turn direction and why small \(L\) or large \(|y'|\) can produce large angular demand.
