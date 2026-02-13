# Document 2: Answers to Lab 1 Questions (Section 6)

Controller context: Pure Pursuit with
\[
\omega = v\gamma,\quad \gamma=\frac{2y'}{L^2},\quad L^2=x'^2+y'^2
\]

## Q1: What problems occur when lookahead distance is too large or too small?

If lookahead distance is too small:

- The selected point is very close to the robot, so curvature changes rapidly between control cycles.
- This makes steering highly sensitive to path discretization, localization noise, and small pose errors.
- The robot tends to oscillate or zig-zag around the path (poor damping), often with frequent angular saturation.

If lookahead distance is too large:

- The selected point is far ahead, so the controller smooths too aggressively and reacts late to local path changes.
- The robot cuts corners and does not track sharp turns accurately.
- Tracking error increases near bends or narrow passages, which can reduce safety margin to obstacles.

In short, lookahead controls a stability-tracking tradeoff: too small gives instability/oscillation; too large gives sluggish response and corner-cutting.

## Q2: What problems occur when desired linear velocity is too large, especially for \(y' < 0\)?

From Pure Pursuit:
\[
\omega = v\frac{2y'}{L^2}
\]

When \(v\) is large, the required \(|\omega|\) also becomes large for the same geometry. If the lookahead is strongly to one side (large \(|y'|\), including \(y' < 0\)), the required turning rate can exceed actuator limits:

\[
|\omega_{\text{req}}| > \texttt{max\_angular\_vel\_}
\]

Then \(\omega\) is clamped, so the robot cannot realize the requested curvature (understeer). Consequences:

- Large lateral tracking error and overshoot.
- Wider-than-intended turning arc.
- Delayed convergence back to path.

Physical effects on real platforms:

- High-speed turning increases lateral acceleration \(a_y=\frac{v^2}{R}=v|\omega|\).
- This can approach traction limits, causing wheel slip/skid, odometry degradation, and unstable motion.

For \(y' < 0\), the turn direction is rightward (\(\omega < 0\)); the same saturation and dynamic-limit arguments apply.

## Q3: What problems occur when the lookahead point is behind the robot (\(x' < 0\))?

If \(x' < 0\), the target point lies behind the robot in robot frame, but the controller still commands forward motion (\(v>0\)). This creates a geometric mismatch: the robot tries to face and reach a behind point while moving forward.

Typical outcomes:

- Large looping trajectories instead of direct convergence.
- Orbit-like or spiral-like behavior before re-acquiring a forward point.
- Oscillatory heading corrections, especially with discrete path points.

Additional mathematical note:

- Curvature depends on \(y'\) and \(L^2=x'^2+y'^2\), not on the sign of \(x'\) directly.
- Therefore, a behind-point condition is not explicitly rejected by the basic formula.
- If \(y'\approx 0\) and \(x'<0\), curvature can become small, so the robot may continue forward away from the desired point before geometry changes.

Hence, \(x'<0\) can degrade convergence and produce inefficient or unstable path-following unless additional logic is added (for example, selecting only forward lookahead points or adding rotate-in-place behavior).
