# Contribution and implementation notes

Group 19 consists of Enqian Tang (25646922). Teaching staff approved the individual group. The project covers the complete simulation, from the motion model and camera frames to estimation, control and evaluation.

## Starting point and corrections

The working files came from the existing IntegratedProject directory. On 3 November the route was set to three waypoints, with the unused scene markers removed. The repeated experiments and video were regenerated for that route. Earlier four-waypoint results are archived outside the submitted project.

Two corrections matter for interpreting the results. First, the motor loop now estimates wheel velocity from quantised encoder increments instead of reading simulator joint velocity. This keeps both feedback loops on measured or estimated quantities. Second, position error is sampled before the physics step, at the same instant as the state estimate. Comparing that estimate with the next physics state would include movement during the step in the reported localization error.

The evaluation scripts add final docking error against the scene reference, heading error, covariance diagnostics, path-following error and a torque-effort measure. The marker validation uses 28 separate poses. Repeated experiments cover observation loss and wheel-distance calibration error, with a complete-loss case retained as a failure.

The implementation combines standard differential-drive kinematics, EKF, A*, LQR and ArUco/PnP. The references below describe the simulator and camera-pose interfaces used in the project.

## References

- UTS 41014 Integrated Project v2 marking guide, Canvas file 14048181. The copy checked on 30 October matched the supplied brief byte for byte.
- [MuJoCo Python interface](https://mujoco.readthedocs.io/en/stable/python.html).
- [OpenCV PnP coordinate convention and pose solvers](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html). The runtime is pinned to OpenCV 4.11.
- [GitHub Pages publishing instructions](https://docs.github.com/en/pages/quickstart).

The shorter return route exposed a stale visual station target in a blocked plan. The docking logic now reacquires such a target, and the search command is 0.8 rad/s to avoid prolonged low-speed rotation. The new regression check verifies stale-target recovery without ground-truth feedback.
