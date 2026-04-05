# Turning API

## `TurnGyro(Theta, MaxSpeed, MinSpeed, EarlyExit, Forward, Direction, Stopping)`

Point turn using both motors to reach a target heading via the IMU.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Theta` | float | — | Target heading (degrees) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `EarlyExit` | float | `5` | Acceptable heading error to stop (degrees) |
| `Forward` | bool | `True` | Face forward (`True`) or backward (`False`) |
| `Direction` | int | `0` | `0` = shortest path, `1` = force clockwise, `2` = force counterclockwise |
| `Stopping` | bool | `True` | Whether to brake after completion |

## `TurnMM(turn_mm, MaxSpeed, MinSpeed, EarlyExit, Stopping)`

Point turn using both motors, controlled purely by **wheel encoders**. Each wheel travels `turn_mm` in opposite directions. Positive = clockwise.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `turn_mm` | float | — | Distance each wheel travels (mm). Positive = clockwise |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `EarlyExit` | float | `5` | Acceptable error to stop |
| `Stopping` | bool | `True` | Whether to brake after completion |

## `TurnToPoint(TargetX, TargetY, MaxSpeed, MinSpeed, EarlyExit, Forward, Direction, Stopping)`

Turns in place to face a target coordinate `(TargetX, TargetY)` using odometry and the IMU.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `TargetX` | float | — | Target X coordinate (mm) |
| `TargetY` | float | — | Target Y coordinate (mm) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `EarlyExit` | float | `5` | Acceptable heading error (degrees) |
| `Forward` | bool | `True` | Face forward or backward toward target |
| `Direction` | int | `0` | `0` = shortest, `1` = clockwise, `2` = counterclockwise |
| `Stopping` | bool | `True` | Whether to brake after completion |
