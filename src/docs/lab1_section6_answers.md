# Document 2: Answers to Lab 1 Questions (Section 6)

Controller context: Pure Pursuit with
$$
\omega = v\gamma,\quad \gamma=\frac{2y'}{L^2},\quad L^2=x'^2+y'^2
$$

## Q1: What problems occur when lookahead distance is too large or too small?

### English
If lookahead distance is too small:

- The target point is too close, so the commanded curvature changes a lot from step to step.
- The robot becomes "twitchy" and tends to oscillate / zig-zag around the path, and can hit angular limits more often.

If lookahead distance is too large:

- The target point is too far, so the controller reacts late.
- The robot cuts corners (corner cutting) and tracks turns poorly, giving larger tracking error near bends.

In short, lookahead controls a stability-tracking tradeoff: too small gives instability/oscillation; too large gives sluggish response and corner-cutting.

### 中文翻译
Lookahead 距离太小：

- 目标点离机器人太近，导致每次算出来的曲率/转向变化很大。
- 机器人会变得“抖”，容易左右来回修正（oscillation / zig-zag），也更容易触发角速度限幅。

Lookahead 距离太大：

- 目标点太靠前，控制反应会变慢。
- 转弯处容易切弯（corner cutting），急弯跟不上，弯道附近误差更大。

总之，lookahead 在“稳定性”和“贴合路径”的折中上起关键作用：太小易振荡，太大易切弯且跟踪不紧。

## Q2: What problems occur when desired linear velocity is too large, especially for $y' < 0$?

From Pure Pursuit:
$$
\omega = v\frac{2y'}{L^2}
$$

### English
For the same lookahead geometry, increasing $v$ increases the required turning rate $|\omega|$. If the lookahead point is to the side (large $|y'|$, including $y' < 0$), $|\omega|$ can become too large and get clamped by `max_angular_vel_`. Then the robot cannot turn as tightly as needed (understeer). Typical effects:

- Large lateral tracking error and overshoot.
- Wider-than-intended turning arc.
- Delayed convergence back to path.

On real robots, high-speed turning also increases lateral acceleration $a_y=\frac{v^2}{R}=v|\omega|$, which can cause slip/skid and worse odometry.

For $y' < 0$, the turn direction is rightward ($\omega < 0$); the same saturation and dynamic-limit arguments apply.

### 中文翻译
同一组 lookahead 几何下，$v$ 越大，需要的转向速度 $|\omega|$ 越大。lookahead 点如果在侧方（$|y'|$ 大，包括 $y'<0$），就更容易让 $|\omega|$ 超过 `max_angular_vel_` 被限幅。限幅后就是“转不过去”（understeer）：转弯半径变大、误差变大、容易超调。

真实机器人还会因为 $a_y=\frac{v^2}{R}=v|\omega|$ 变大而更容易打滑/侧滑，里程计变差。$y'<0$ 只是代表右转（$\omega<0$），道理一样。

## Q3: What problems occur when the lookahead point is behind the robot ($x' < 0$)?

### English
If $x' < 0$, the target point lies behind the robot in robot frame, but the controller still commands forward motion ($v>0$). This creates a geometric mismatch: the robot tries to face and reach a behind point while moving forward.

Using the pure pursuit formulas,
$$
\gamma=\frac{2y'}{L^2},\quad L^2=x'^2+y'^2,\quad \omega=v\gamma
$$
note that the sign of $x'$ does not appear directly in $\gamma$ (since $x'$ is squared in $L^2$). So the basic controller does not "special-case" $x'<0$; it still drives forward and steers using $y'$.

A common behavior is a wide detour: the robot loops around until the lookahead point moves back to the front.

Important special case: if the point is directly behind ($y'=0$, $x'<0$), then $\gamma=0$ and $\omega=0$, so the robot drives straight forward (away from the point) until the lookahead selection changes.

Hence, $x'<0$ can degrade convergence and produce inefficient or unstable path-following unless additional logic is added (for example, selecting only forward lookahead points or adding rotate-in-place behavior).

### 中文翻译
当 $x' < 0$ 时，lookahead 点在机器人后方，但控制器仍然让机器人保持前进（$v>0$）。这会造成几何上的不匹配：机器人一边向前走，一边又试图“追”后面的点，往往不能直接收敛到路径。

从公式角度看：
$$
\gamma=\frac{2y'}{L^2},\quad L^2=x'^2+y'^2,\quad \omega=v\gamma
$$
因为 $x'$ 在 $L^2$ 里被平方了，所以 $x'$ 的正负不会直接进到 $\gamma$ 里。也就是说，基础 pure pursuit 不会特别处理 $x'<0$，它还是会照算曲率、然后让机器人往前开。

实际效果通常就是先走出一个很大的回环/绕圈，直到 lookahead 点又回到前方才会正常跟踪。

关键特殊情况：如果点几乎正好在正后方（$y'=0$ 且 $x'<0$），则 $\gamma=0$、$\omega=0$，机器人会一直直线往前走，离目标点反而越来越远，直到 lookahead 点更新。

所以 $x'<0$ 会让收敛变慢、轨迹变怪。工程上常见做法是只选 $x'>0$ 的点，或者先原地转向再前进。
