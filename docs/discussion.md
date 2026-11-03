# Results and limitations

The route has three waypoints, (1,0), (0,1) and (0,0) m, followed by visual docking. The outward leg requires a LiDAR detour around the obstacle. The second leg crosses the room; the third returns to the start before the forward-camera docking approach. A four-corner room circuit is unnecessary for these checks.

The three-waypoint route completes in 58.29 s on average in nominal trials, including the one-second charging demonstration. Mean localization RMSE is 3.81 mm and mean final docking error is 59.65 mm. These measure estimator accuracy and final stopping position respectively.

Each of the four conditions uses three seeds. The recorded success counts are nominal: 3/3, dropout40: 3/3, blackout20to26: 3/3, encoder_plus2pct: 3/3. Mean localization RMSE is 6.32 mm under 40% random observation loss, 7.86 mm under a six-second blackout, and 4.00 mm with +2% encoder scale error. All reported results belong to the three-waypoint runtime and scene.

Continuous blackout leaves a longer interval of encoder-only prediction than scattered observation gaps. The uncertainty band expands during that interval and contracts after a usable observation returns. LiDAR continues mapping obstacles; it is not a substitute absolute-pose observation in this EKF.

Shortening the route exposed a case where an old station observation led to a blocked docking plan. If a docking plan is BLOCKED and the station measurement is older than two seconds, the robot now clears that target and searches for a new observation. It does not discard the target merely because an otherwise valid approach takes more than two seconds. This distinction avoids unnecessary repeated searches. A regression test covers the blocked/stale-target case.

Docking error is much larger than localization error because translation stops near the 60 mm arrival threshold before heading alignment. Connection retention uses wider position and angle thresholds. This is a virtual connector, not a physical contact experiment. The equivalent circuit is a small additional simulation of charge current and SOC, not evidence of a hardware charger.

With complete camera-observation loss, the run ends in SEARCH DOCK after 50.0 s, with 3/3 estimated waypoint arrivals and 0.274 m localization RMSE. It is a failed mission. Estimated arrival counts do not independently prove true arrival, and no visual station target is available. This separate failure case is not added to the denominator of the repeated-condition trials.

In the seed-7 sensitivity check, R x0.1 gives 1.74 mm localization RMSE, compared with 3.78 mm at the common baseline and 11.83 mm at R x10. The noise settings are engineering allowances, not a fitted physical noise model. One sensitivity seed does not establish an optimum.

Other limits remain. Static camera-pose validation has small base-position errors, but the accepted station-relative depth error reaches about 79 mm at distant oblique views. Current-path RMSE uses a path that changes when A* replans. Unknown grid cells are treated optimistically. A valid PnP pose resets camera age even when the EKF rejects that pose; accepted-update freshness would be a useful further check. The small trial count and one static room do not establish general reliability.
