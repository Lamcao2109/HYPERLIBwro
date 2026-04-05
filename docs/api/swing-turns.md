# Swing Turns API

Swing turns use **one motor** while the other is held in place, creating a pivot turn.

## `SwingGyro(Side, Theta, EarlyExit, MaxSpeed, MinSpeed, Forward, Direction, Stopping)`

Pivot turn using one motor while the other is held, controlled by the **IMU gyro**.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Side` | int | — | `1` = left motor moves (right held), other = right motor moves (left held) |
| `Theta` | float | — | Target heading (degrees) |
| `EarlyExit` | float | `5` | Acceptable heading error (degrees) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `Forward` | bool | `True` | Face forward or backward |
| `Direction` | int | `0` | `0` = shortest, `1` = clockwise, `2` = counterclockwise |
| `Stopping` | bool | `True` | Whether to brake after completion |

## `SwingMM(Side, turn_mm, EarlyExit, MaxSpeed, MinSpeed, Direction, Stopping)`

Pivot turn using one motor while the other is held, controlled by the **moving wheel's encoder**.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Side` | int | — | `1` = left motor moves, other = right motor moves |
| `turn_mm` | float | — | Distance the moving wheel travels (mm). Positive = clockwise |
| `EarlyExit` | float | `5` | Acceptable error |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `Direction` | int | `0` | Turn direction |
| `Stopping` | bool | `True` | Whether to brake after completion |

## `SwingToPoint(Side, TargetX, TargetY, EarlyExit, MaxSpeed, MinSpeed, Forward, Direction, Stopping)`

Swing turn to face a specific field coordinate using one motor and the IMU gyro.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Side` | int | — | `1` = left motor moves, other = right motor moves |
| `TargetX` | float | — | Target X coordinate (mm) |
| `TargetY` | float | — | Target Y coordinate (mm) |
| `EarlyExit` | float | `5` | Acceptable heading error (degrees) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `Forward` | bool | `True` | Face forward or backward toward target |
| `Direction` | int | `0` | Turn direction |
| `Stopping` | bool | `True` | Whether to brake after completion |
