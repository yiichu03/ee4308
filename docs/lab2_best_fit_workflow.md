# Lab 2 Sonar Variance Workflow

This note is for Question 3 in `lab2.md`: use a best fit line to estimate an initial value for `sigma_snr,z^2`, then do further tuning in simulation.

## What I added

I did not change the estimator logic.

I only added these helper files:

- [collect_sonar_samples.sh](/home/liuyi/projects/ee4308_course/collect_sonar_samples.sh)
- [analyze_sonar_samples.sh](/home/liuyi/projects/ee4308_course/analyze_sonar_samples.sh)
- [record_sonar_samples.py](/home/liuyi/projects/ee4308_course/tools/record_sonar_samples.py)
- [estimate_sonar_variance.py](/home/liuyi/projects/ee4308_course/tools/estimate_sonar_variance.py)

## What the scripts do

- `collect_sonar_samples.sh`
  - subscribes to `/drone/sonar`
  - records valid `ranges[0]` samples
  - writes `index,time_sec,sonar_range_m` to a CSV

- `analyze_sonar_samples.sh`
  - fits a best fit line to `sonar_range_m` versus `time_sec`
  - subtracts that fitted line
  - computes the sample variance of the residuals
  - writes a short summary and a residual CSV

## Recommended procedure

1. In terminal 1, run the simulation.

```bash
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
./bd.sh
./proj2_sim.sh
```

2. Wait until the simulator is stable and the drone is stationary.

3. In terminal 2, collect about 100 to 120 valid sonar samples.

```bash
cd /ws/ee4308
source /opt/ros/jazzy/setup.bash
./collect_sonar_samples.sh 120
```

4. Analyze the CSV with the best fit line method.

```bash
cd /ws/ee4308
./analyze_sonar_samples.sh tmp/lab2_sonar/<timestamp>/sonar_samples.csv
```

5. Read `residual_variance_m2` from `variance_summary.txt`.
   Use that as the initial value for `var_sonar` in [proj2.yaml](/home/liuyi/projects/ee4308_course/src/ee4308_bringup/params/proj2.yaml#L39).

6. Rebuild and run the simulator again.

```bash
cd /ws/ee4308
./bd.sh
./proj2_sim.sh
```

7. Do a small amount of further tuning around that initial value.

- If the estimated `z` follows sonar noise too aggressively, increase `var_sonar`.
- If the estimated `z` drifts too much and sonar correction is too weak, decrease `var_sonar`.

## Suggested tuning range

After the best fit line step, test a small neighborhood around the estimated variance, for example:

- `0.5 x initial value`
- `1.0 x initial value`
- `2.0 x initial value`

Then compare:

- `Pose.z`
- `Twist.z`
- `ErrPose.z`
- `ErrTwis.z`
- `Sonar`

For Lab 2, a good result should keep:

- `Pose.z` close to `Sonar`
- `Twist.z` close to `0`
- `ErrPose.z` small
- `ErrTwis.z` small
