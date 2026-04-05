# Version History

| Version | File | Notes |
|---------|------|-------|
| v1.6 | [`Hyperlib_v1_6.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py) | Added Pure Pursuit path following, concurrent odometry via `multitask` |
| v1.5 | [`Hyperlib_v1_5.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py) | Added arc odometry, encoder-based turns (`TurnMM`, `SwingMM`), improved PID gains |
| v1.4 | [`Hyperlib_V1_4/`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_4/) | Previous stable version |
| v1.3 | [`Hyperlib V1.3.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Hyperlib%20V1.3.py) | Initial release |

## v1.5 → v1.6 Changes

### Pure Pursuit (`PurePursuit`)

New function for following multi-waypoint paths smoothly. Replaces chaining multiple `MoveToPoint` calls when you need the robot to travel through a sequence of coordinates without stopping at each one.

Uses a lookahead circle to find the next steering target on the path, producing naturally curved trajectories. Path is defined by setting `PArray` (a list of `[x, y]` waypoints) before calling `PurePursuit`.

### Concurrent Odometry via `multitask`

Odometry is now separated into its own `odometry_task()` coroutine that runs concurrently with the mission code using Pybricks' `multitask`.

In v1.5, odometry was called inside each movement function. In v1.6, it runs continuously in the background, so position is always current regardless of which movement is executing.

The new program structure splits `main()` into `my_mission()` + `odometry_task()` composed with `multitask`.
