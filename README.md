# HYPERLIB
A library for pybrick use in the World Robot Olympiad 
newest version: V1.5
---

## Requirements

- [Pybricks](https://pybricks.com/) firmware on a LEGO SPIKE Prime hub
- Motors connected to **Port A** (left, reversed) and **Port E** (right)
- The hub's built-in IMU is used for heading data

---

## Robot Configuration

| Parameter   | unit | Description                          |
|-------------|-------|--------------------------------------|
| `WheelD`    | mm | Wheel diameter                       |
| `Axle`      | mm | Distance between wheels              |
| `LoopTime`  | ms | Control loop update frequency        |
| `MaxSpeed`  | °/s | Maximum motor speed              |
| `MinSpeed`  | °/s  | Minimum motor speed              |
| `MaxAcc`    | °/s²| Maximum acceleration            |

---

## PID Gains

| Constant        | Purpose                                      |
|-----------------|----------------------------------------------|
| `GKp`           | Proportional gain for heading correction     |
| `GKd`           | Derivative gain for heading correction       |
| `DistanceKp`    | Proportional gain for distance-based speed   |
| `SingleTurnKp`  | Gain for single-motor swing turns            |
| `DoubleTurnKp`  | Gain for two-motor turns                     |

---

## API Reference

### Helper Functions

#### `DegToMM(deg)`
Converts motor encoder degrees to millimeters of linear travel using the configured wheel diameter.

#### `ThetaErr(angle)`
Returns the signed heading error (in degrees, range −180 to 180) between the current IMU heading and a target angle.

---

### Motor Control

#### `LeftControl(speed, err)` / `RightControl(speed, err)`
Async functions that drive each motor individually. They enforce acceleration ramp limits (`MaxAcc`) and clamp speed within `[MinSpeed, MaxSpeed]`. The `err` parameter adds a correction offset (used for heading PID).

---

### Odometry

#### `OdomIni(x, y, theta)`
Initializes the robot's position and heading. Resets the IMU to `theta`, sets odometry coordinates to `(x, y)`, and zeroes the motor encoders.

#### `OdomUpdate()`
Incrementally updates the robot's `(odomX, odomY)` position using wheel encoder deltas and the average IMU heading between updates (midpoint integration). Returns `(odomX, odomY, current_theta)`.

Set `OdometrySwitch = False` to disable automatic odometry updates inside movement functions.

---

### Movement Functions

#### `SwingGyro(Side, Theta, EarlyExit, MaxSpeed, MinSpeed, Forward, Stopping)`
Performs a **swing (pivot) turn** using one motor while holding the other stationary.

- `Side = 1` → left motor moves, right held
- `Side` ≠ 1 → right motor moves, left held
- `Theta` — target heading in degrees
- `EarlyExit` — acceptable heading error to stop (degrees, default `1`)
- `Forward` — face forward (`True`) or backward (`False`) direction
- `Stopping` — brake and hold after completion

#### `GyroTurn(Theta, EarlyExit, MaxSpeed, MinSpeed, Forward, Stopping)`
Performs a **point turn** using both motors spinning in opposite directions to reach a target heading. More precise than `SwingGyro` for heading-only corrections.

#### `TurnToPoint(targetX, targetY, ...)` *(truncated in source)*
Turns the robot in place to face a target coordinate `(targetX, targetY)` using odometry.

#### `MoveToPoint(targetX, targetY, MaxSpeed, MinSpeed, ArrivalThreshold, Forward, Stopping)`
Drives the robot to a target coordinate using a **P-controller** for distance-based speed and a **PD-controller** for heading correction. Stops when within `ArrivalThreshold` mm of the target.

#### `MoveToPoRo(targetX, targetY, targetAngle, offsetDistance, finalDistance, offsetAngle, ...)`
Drives to a point while also controlling the **final approach angle** (Position + Orientation). A virtual waypoint is placed behind the target at `offsetDistance`; the robot aims for it from far away, then smoothly blends toward the final `targetAngle` as it gets closer. Useful for docking or aligning the robot at a specific orientation.

Key parameters:
- `targetAngle` — desired heading when arriving at the target
- `offsetDistance` — how far behind the target the virtual waypoint is placed (mm)
- `offsetAngle` — angular offset of the virtual waypoint
- `minRatio` — minimum blending ratio during final approach

#### `Stop()`
Brakes both motors for 40 ms, then holds them in place.

---

## Entry Point

```python
async def main():
    await OdomIni(0, 0, 0)        # Start at origin, facing 0°
    await TurnToPoint(500, 500)   # Turn to face point (500, 500)
    await MoveToPoRo(500, 500, 0, DistanceKp=3)  # Drive there, arrive facing 0°

run_task(main())
```

---

## Notes

- All movement functions are `async` and must be called with `await` or composed using `multitask` / `run_task`.
- The Bluetooth button is set as the stop button to prevent accidental program termination.
- The `view()` function is a debug utility that continuously prints the IMU heading to the console.
