# Hyperlib — LEGO SPIKE Prime Motion Control Library

A MicroPython motion control library for a differential drive robot built on the **LEGO SPIKE Prime** hub using [Pybricks](https://pybricks.com/). Hyperlib provides odometry, PID-based control, and a full suite of movement functions for autonomous navigation.

---

## 📁 Repository Structure

```
├── Versions/
│   ├── Hyperlib_V1_4/
│   └── Hyperlib_V1_5/
│       ├── Hyperlib_v1_5.py   ← Main library
│       └── main_program.py    ← Your program goes here
├── Hyperlib V1.3.py
└── README.md
```

---

## ⚡ Quick Start

1. Place [`Hyperlib_v1_5.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py) and [`main_program.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/main_program.py) in the same directory on your SPIKE Prime hub.
2. Write your autonomous routine inside `main_program.py`:

```python
from Hyperlib_v1_5 import *

async def main():
    await OdomIni(0, 0, 0)        # Start at origin, facing 0°
    await SwingMM(2, -300)        # Swing turn 300 mm on right wheel
    await TurnGyro(0)             # Turn back to face 0°

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

#### [`MoveToPoint(targetX, targetY, MaxSpeed, MinSpeed, ArrivalThreshold, Forward, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L733)
Drives to a target coordinate `(targetX, targetY)` using a P-controller for distance-based speed and a PD-controller for heading correction. Stops within `ArrivalThreshold` mm of the target.

#### [`MoveToPoRo(targetX, targetY, targetAngle, offsetDistance, finalDistance, offsetAngle, MaxSpeed, MinSpeed, ArrivalThreshold, minRatio, Forward, DKp, Stopping)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L789)
Drives to a coordinate while controlling the **final approach angle** (Position + Orientation). Places a virtual waypoint behind the target and smoothly blends the robot's heading toward `targetAngle` as it gets closer. Ideal for docking or alignment tasks.

| Parameter          | Default | Description                                 |
|--------------------|---------|---------------------------------------------|
| `targetAngle`      | —       | Desired heading when arriving (degrees)     |
| `offsetDistance`   | 250 mm  | Distance behind target for virtual waypoint |
| `offsetAngle`      | 20°     | Angular offset of virtual waypoint          |
| `minRatio`         | 0.8     | Minimum blend ratio during final approach   |
| `ArrivalThreshold` | 25 mm   | Distance considered as "arrived"            |

---

### Utility

#### [`Stop()`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L886)
Brakes both motors for 40 ms, then holds them in place.

#### [`DegToMM(deg)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L53)
Converts motor encoder degrees to millimeters of travel using the configured wheel diameter.

#### [`ThetaErr(angle)`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L58)
Returns the signed heading error in degrees (range −180° to 180°) between the current IMU heading and a target angle.

---

## 📝 Notes

- **⚠️ First run:** Before running any program, update your robot's physical specs at the top of [`Hyperlib_v1_5.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py#L33) to match your actual hardware:
  ```python
  WheelD = 62     # ← your wheel diameter in mm
  Axle   = 170    # ← your axle width in mm (wheel center to wheel center)
  ```
  Incorrect values will cause all distance and odometry calculations to be wrong.
- All movement functions are `async` and must be called with `await`, or composed using Pybricks' `multitask` / `run_task`.
- The **Bluetooth button** is configured as the stop button to prevent accidental program termination during a run.
- For debugging, use the `view()` function in `main_program.py` to continuously print the current IMU heading to the console.
- Hyperlib uses **arc odometry** in `OdomUpdate` for accurate position tracking on both straight and curved paths.


---

## 🗂️ Version History

| Version | File | Notes |
|---------|------|-------|
| v1.5 | [`Hyperlib_v1_5.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_5/Hyperlib_v1_5.py) | Added arc odometry, encoder-based turns (`TurnMM`, `SwingMM`), improved PID gains |
| v1.4 | [`Hyperlib_V1_4/`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_4/) | Previous stable version |
| v1.3 | [`Hyperlib V1.3.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Hyperlib%20V1.3.py) | Initial release |

---

## :handshake: Contributor
:bust_in_silhouette:author: @Lamcao2109
:busts_in_silhouette:contributors: @DuCpHancm
