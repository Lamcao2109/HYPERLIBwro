# Hyperlib — LEGO SPIKE Prime Motion Control Library

![Hyperlib Logo](https://github.com/Lamcao2109/HYPERLIBwro/raw/main/Media/Hyperlib_horizontal.png)

A MicroPython motion-control library for a differential-drive robot built on the **LEGO SPIKE Prime** hub, using [Pybricks](https://pybricks.com/). Hyperlib provides odometry, PID-based control, and a full suite of movement functions for autonomous navigation.

---

## The Overall Idea

Odometry is not magic — it cannot pinpoint exactly where your robot is at every moment. It is an approximation, and the error increases as the robot travels. In the World Robot Olympiad context, odometry is a method to cut corners and travel to approximate positions. For exact positioning, you must plan waypoints (walls, black lines to follow, colors on the map, etc.) that the robot can track using other sensors.

These functions can **never** make the robot arrive at the desired position/heading with 100% accuracy. The idea is that the robot keeps steering toward its target to **minimize** the error, not completely eliminate it. As the robot approaches an acceptable margin of error, it stops. Since odometry tracking is always on, the robot can always approximate where it is.

!!! tip "Build Your Own Functions!"
    We encourage you to add functions of your own! Study the methods and programming style used to build these functions. You must understand the **mathematical proof** and **logic** behind the algorithms to build your own. Contact the author (Lam) if you have questions!

---

## Features

- **Arc odometry** for accurate position tracking on straight and curved paths
- **PID-based control** for heading correction and distance management
- **Pure Pursuit** path-following algorithm for smooth multi-waypoint trajectories
- **Concurrent odometry** via Pybricks `multitask` — position is always up to date
- **Flexible movement**: encoder-based or gyro-based straight movement, point turns, swing turns
- **Coordinate-based navigation**: drive to any (x, y) coordinate on the field

---

## Getting Started

Head to the [Quick Start](quickstart.md) guide to set up Hyperlib on your robot.

---

## Contributors

- 👤 Author: [@Lamcao2109](https://github.com/Lamcao2109)
- 👥 Contributors: [@DuCpHancm](https://github.com/DuCpHancm), [@haing2811](https://github.com/haing2811/Concainit123)

## License

Hyperlib is released under the [MIT License](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/LICENSE).
