# Hyperlib — LEGO SPIKE Prime Motion Control Library

<p align="center">
  <img src="Media\Hyperlib_horizontal.png">
</p>

A MicroPython motion-control library for a differential-drive robot built on the **LEGO SPIKE Prime** hub, using [Pybricks](https://pybricks.com/). Hyperlib provides odometry, PID-based control, and a full suite of movement functions for autonomous navigation.

---

## The overall idea

Odometry is not magic; it cannot pinpoint exactly where your robot tracks every single minute movement of both wheels. It is just an approximation, and the error keeps increasing as the robot travels. Therefore, odometry in the World Robot Olympiad program is just a method to cut corners, to travel to approximate positions. If you want exact positions, you must plan waypoints (walls, black line to follow, colors on the map,...) which the robot can track using other kinds of sensors

These functions can NEVER make the robot arrive at the desired position/heading with 100% accuracy. The idea behind these functions is that the robot keeps steering to its target to MINIMIZE the error between the robot's and the desired position/heading, not completely EFFACING it. Thus, if you want the robot to be 100% on the target, it will run forever and can never complete this task. Instead, as the robot approaches an acceptable margin of error, it will stop. But don't worry! Since odometry tracking is always on, the robot will ALWAYS be able to approximate where it is and where it isn't. 

## IMPORTANT

WE ENCOURAGE YOU TO ADD FUNCTIONS OF YOUR OWN TO THE PROGRAM! Many functions may be essential to you that we have not built yet! So, don't hesitate to study the method and programming style used to build these functions to construct your own! Also, you must firmly understand the MATHEMATICAL PROOF and LOGIC under the algorithms and functions to build your own programmes. Contact the author (Lam) if you have any question to ask!

---

## 📁 Repository Structure

```
├── Versions/
│   ├── Hyperlib_V1_4/
│   ├── Hyperlib_V1_5/
│   └── Hyperlib_V1_6/          ← Current version
│       ├── Hyperlib_v1_6.py    ← Main library
│       └── main_program.py     ← Your program goes here
├── Hyperlib V1.3.py
└── README.md
```

---

## ⚡ Quick Start

1. Place [`Hyperlib_v1_6.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py) and [`main_program.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/main_program.py) in the same directory on your SPIKE Prime hub.
2. Write your autonomous routine inside `main_program.py`:

```python
from Hyperlib_v1_6 import *
import Hyperlib_v1_6 as lib

async def my_mission():
    await OdomIni(0, 0, 0)        # Start at origin, facing 0°
    await TurnToPoint(-300, 300)  # Face target coordinate
    await MoveToPoint(-300, 300)  # Drive to it
    lib.PArray = [[0,0],[200,300],[400,0]]
    await PurePursuit()           # Follow the path

async def odometry_task():
    while True:
        await OdomUpdate()        # Keep position tracking running
        await wait(LoopTime)

async def main():
    await multitask(my_mission(), odometry_task())  # Run both concurrently

run_task(main())
```

---

## 🤖 Hardware Configuration

| Parameter    | Value      | Description                        |
|--------------|------------|------------------------------------|
| Left Motor   | Port A     | Reversed (CounterClockwise)        |
| Right Motor  | Port E     | Standard                           |
| `WheelD`     | based on your robot     | Wheel diameter                     |
| `Axle`       | based on your robot     | Distance between wheels            |
| `LoopTime`   | 10 ms      | Control loop update frequency      |
| `MinSpeed`   | input your motors' min dps (°/s)  | Minimum motor speed                |
| `MaxSpeed`   | input your motors' max dps (°/s)  | Maximum motor speed                |
| `MaxAcc`     | input your motors' max acceleration (°/s²) | Maximum acceleration               |

---

## 🎛️ PID Gains

| Constant             | Value | Purpose                                       |
|----------------------|-------|-----------------------------------------------|
| `GKp`                | 20    | Proportional gain for gyro heading correction |
| `GKd`                | 250   | Derivative gain for gyro heading correction   |
| `ENCKp`              | 1     | Proportional gain for encoder straightening   |
| `ENCKd`              | 10    | Derivative gain for encoder straightening     |
| `DistanceKp`         | 4     | Proportional gain for distance-based speed    |
| `GyroSingleTurnKp`   | 20    | Gain for gyro-based swing turns               |
| `ENCSingleTurnKp`    | 12    | Gain for encoder-based swing turns            |
| `DoubleTurnKp`       | 8     | Gain for two-motor point turns                |

Set `OdometrySwitch = False` to disable automatic odometry updates inside movement functions.

---

## 📖 API Reference

### Odometry

#### [`OdomIni(x, y, theta)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L131)
Initializes the robot's position and heading. Resets the IMU to `theta`, sets coordinates to `(x, y)`, and zeroes motor encoders.

#### [`OdomUpdate()`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L149)
Updates `(odomX, odomY)` using **arc odometry** — accurate for both straight paths and curves. Uses wheel encoder deltas combined with IMU heading. Returns `(odomX, odomY, current_theta)`.

---

### Straight Movement

#### [`MoveMM(Distance, MaxSpeed, MinSpeed, Forward, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L194)
Moves straight for a set distance (mm) using only **wheel encoders** for straightness correction (PD-controller on left/right encoder difference). No gyro required.

#### [`MoveGyro(Distance, Theta, MinSpeed, MaxSpeed, Forward, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L492)
Moves straight for a set distance (mm) while maintaining a **target heading** using the IMU gyro (PD-controller on heading error).

---

### Turning

#### [`TurnGyro(Theta, MaxSpeed, MinSpeed, EarlyExit, Forward, Direction, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L538)
Point turn using both motors to reach a target heading via the IMU.

- `Theta` — target heading in degrees
- `EarlyExit` — acceptable heading error to stop (default `5°`)
- `Forward` — face forward (`True`) or backward (`False`)
- `Direction` — `0` = shortest path, `1` = force clockwise, `2` = force counterclockwise

#### [`TurnMM(turn_mm, MaxSpeed, MinSpeed, EarlyExit, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L251)
Point turn using both motors, controlled purely by **wheel encoders**. Each wheel travels `turn_mm` in opposite directions. Positive = clockwise.

#### [`TurnToPoint(TargetX, TargetY, MaxSpeed, MinSpeed, EarlyExit, Forward, Direction, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L593)
Turns in place to face a target coordinate `(TargetX, TargetY)` using odometry and the IMU.

---

### Swing Turns (One Motor)

#### [`SwingGyro(Side, Theta, EarlyExit, MaxSpeed, MinSpeed, Forward, Direction, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L410) 
Pivot turn using one motor while the other is held, controlled by the **IMU gyro**.

- `Side = 1` → left motor moves, right held
- `Side` ≠ 1 → right motor moves, left held
- `Theta` — target heading in degrees

#### [`SwingMM(Side, turn_mm, EarlyExit, MaxSpeed, MinSpeed, Direction, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L322)
Pivot turn using one motor while the other is held, controlled by the **moving wheel's encoder**.

- `turn_mm` — distance the moving wheel travels (mm). Positive = clockwise.

#### [`SwingToPoint(Side, TargetX, TargetY, EarlyExit, MaxSpeed, MinSpeed, Forward, Direction, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L652)
Swing turn to face a specific field coordinate using one motor and the IMU gyro.

---

### Coordinate-Based Movement

#### [`MoveToPoint(targetX, targetY, MaxSpeed, MinSpeed, ArrivalThreshold, Forward, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py#L700)
Drives to a target coordinate `(targetX, targetY)` using a P-controller for distance-based speed and a PD-controller for heading correction. Stops within `ArrivalThreshold` mm of the target.

#### [`MoveToPoRo(targetX, targetY, targetAngle, offsetDistance, finalDistance, offsetAngle, MaxSpeed, MinSpeed, ArrivalThreshold, minRatio, Forward, DKp, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py#L754)
Drives to a coordinate while controlling the **final approach angle** (Position + Orientation). Places a virtual waypoint behind the target and smoothly blends the robot's heading toward `targetAngle` as it gets closer. Ideal for docking or alignment tasks.

| Parameter          | Default | Description                                 |
|--------------------|---------|---------------------------------------------|
| `targetAngle`      | —       | Desired heading when arriving (degrees)     |
| `offsetDistance`   | 250 mm  | Distance behind target for virtual waypoint |
| `offsetAngle`      | 20°     | Angular offset of virtual waypoint          |
| `minRatio`         | 0.8     | Minimum blend ratio during final approach   |
| `ArrivalThreshold` | 25 mm   | Distance considered as "arrived"            |

#### [`PurePursuit(MaxSpeed, MinSpeed, ArrivalThreshold, Lookahead, Forward, DKp, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py#L848)
Follows a sequence of waypoints stored in `PArray` using the **Pure Pursuit** algorithm. The robot continuously steers toward a "lookahead point" — a point on the path a fixed distance ahead of the robot — instead of aiming directly at each waypoint. This produces smooth, curved trajectories through all the waypoints.

Before calling `PurePursuit`, set `PArray` to a list of `[x, y]` waypoints (in order):

```python
import Hyperlib_v1_6 as lib
lib.PArray = [[0, 0], [200, 300], [400, 100], [600, 0]]
await PurePursuit(MaxSpeed=900)
```

| Parameter          | Default | Description                                              |
|--------------------|---------|----------------------------------------------------------|
| `MaxSpeed`         | 900     | Maximum motor speed (dps)                                |
| `MinSpeed`         | 250     | Minimum motor speed for final approach (dps)             |
| `ArrivalThreshold` | 25 mm   | Distance to waypoint considered "reached"                |
| `Lookahead`        | 50 mm   | Lookahead radius — larger = smoother but less precise    |
| `Forward`          | `True`  | `True` = drive forward, `False` = drive backward        |
| `Stopping`         | `True`  | Whether to brake after reaching the final waypoint       |

**How it works:**

The robot treats `PArray` as a path made of connected line segments. Each control loop iteration:

1. For the current segment, it finds where a circle of radius `Lookahead` centered on the robot intersects the segment.
2. It steers toward that intersection point ("the lookahead point"), not the waypoint itself.
3. When the robot gets within `ArrivalThreshold` of the next waypoint, it advances to the next segment.
4. Once all segments are traversed, it calls `MoveToPoint` to precisely arrive at the final waypoint.

This gives the robot smooth, continuous motion through a path — sharp corners are rounded naturally by the lookahead radius.

---

### Multitask and Odometry

Starting from v1.6, odometry runs as a **concurrent background task** using Pybricks' `multitask`. This means the robot's position is always up to date during movement, including inside `PurePursuit`.

The recommended program structure is:

```python
async def my_mission():
    # Your movement code here
    ...

async def odometry_task():
    while True:
        await OdomUpdate()
        await wait(LoopTime)

async def main():
    await multitask(my_mission(), odometry_task())

run_task(main())
```

`multitask` runs both coroutines interleaved on the same thread — `odometry_task` calls `OdomUpdate` on every loop tick while `my_mission` executes movements. All movement functions yield control with `await wait(LoopTime)`, which gives the odometry task time to run.

> **Note:** `multitask` is not true parallelism — it is cooperative multitasking. Both tasks share the same CPU and take turns. The `await wait(...)` calls inside movement functions are the yield points.

---

### Utility

#### [`Stop()`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L886)
Brake both motors for 40 ms, then hold them in place.

#### [`DegToMM(deg)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L53)
Converts motor encoder degrees to millimeters of travel using the configured wheel diameter.

#### [`ThetaErr(angle)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L58)
Returns the signed heading error in degrees (range −180° to 180°) between the current IMU heading and a target angle.

---

## 📝 Notes

- **⚠️ First run:** Before running any program, update your robot's physical specs at the top of [`Hyperlib_v1_6.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py#L33) to match your actual hardware:
  ```python
  WheelD = 62     # ← your wheel diameter in mm
  Axle   = 170    # ← your axle width in mm (wheel center to wheel center)
  ```
  Incorrect values will cause all distance and odometry calculations to be wrong.
- All movement functions are `async` and must be called with `await`, or composed using Pybricks' `multitask` / `run_task`.
- `PurePursuit` requires odometry to be running. Always pair it with `odometry_task()` inside `multitask` (see Quick Start).
- The **Bluetooth button** is configured as the stop button to prevent accidental program termination during a run.
- For debugging, use the `view()` function in `main_program.py` to continuously print the current IMU heading to the console.
- Hyperlib uses **arc odometry** in `OdomUpdate` for accurate position tracking on both straight and curved paths.


---

## 🗂️ Version History

| Version | File | Notes |
|---------|------|-------|
| v1.6 | [`Hyperlib_v1_6.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py) | Added Pure Pursuit path following, concurrent odometry via `multitask` |
| v1.5 | [`Hyperlib_v1_5.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py) | Added arc odometry, encoder-based turns (`TurnMM`, `SwingMM`), improved PID gains |
| v1.4 | [`Hyperlib_V1_4/`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_4/) | Previous stable version |
| v1.3 | [`Hyperlib V1.3.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Hyperlib%20V1.3.py) | Initial release |

### v1.5 → v1.6 Changes

**Pure Pursuit (`PurePursuit`)**
- New function for following multi-waypoint paths smoothly. Replaces chaining multiple `MoveToPoint` calls when you need the robot to travel through a sequence of coordinates without stopping at each one.
- Uses a lookahead circle to find the next steering target on the path, producing naturally curved trajectories.
- Path is defined by setting `PArray` (a list of `[x, y]` waypoints) before calling `PurePursuit`.

**Concurrent Odometry via `multitask`**
- Odometry is now separated into its own `odometry_task()` coroutine that runs concurrently with the mission code using Pybricks' `multitask`.
- In v1.5, odometry was called inside each movement function. In v1.6, it runs continuously in the background, so position is always current regardless of which movement is executing.
- The new program structure splits `main()` into `my_mission()` + `odometry_task()` composed with `multitask`.

---

## 📏 Odometry and its mathematical proof

Odometry is a method to estimate the robot's position. There are two main odometry methods: linear odometry and arc odometry. During straight movements, linear odometry is best suited for use, whereas during curved moment, arc odometry is the optimal method

Here is the mathematical proof of it, you can try to look for symmetries in these graph and the code to better understand odometry:

<p align="center">
  <img src="Media\Arc Odometry Proof.png" width="1000">
  <img src="Media\Linear Odometry Proof.png" width="1000">
</p>

---

## 📐 Pure Pursuit and its mathematical proof

Pure Pursuit is a path-tracking algorithm. Instead of driving directly toward each waypoint and stopping, the robot steers toward a **lookahead point** — a point on the path a fixed lookahead distance `L` ahead of the robot. This produces smooth, continuous curves through the entire path.

Here is the mathematical proof of it, you can try to look for symmetries in this graph and the code to better understand Pure Pursuit:

<p align="center">
  <img src="Media\Pure Pursuit Proof.png" width="1000">
</p>

---

## Our partners 🤗

Mr. Phong and his colleagues have been working with us since day one of robotics, and I want to dedicate a section of this library to honor their contributions to building our knowledge and curiosity, which have allowed us to build this library.
<p align="center">
  <img src="Media\TPS.png">
</p>

---

## :handshake: Contributors
:bust_in_silhouette:author: @Lamcao2109
:busts_in_silhouette:contributors: @DuCpHancm
