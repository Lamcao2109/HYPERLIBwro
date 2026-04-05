# Straight Movement API

## `MoveMM(Distance, MaxSpeed, MinSpeed, Forward, Stopping)`

Moves straight for a set distance (mm) using only **wheel encoders** for straightness correction (PD-controller on left/right encoder difference). No gyro required.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Distance` | float | — | Distance to travel (mm) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `Forward` | bool | `True` | `True` = forward, `False` = backward |
| `Stopping` | bool | `True` | Whether to brake after completion |

## `MoveGyro(Distance, Theta, MinSpeed, MaxSpeed, Forward, Stopping)`

Moves straight for a set distance (mm) while maintaining a **target heading** using the IMU gyro (PD-controller on heading error).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `Distance` | float | — | Distance to travel (mm) |
| `Theta` | float | — | Target heading to maintain (degrees) |
| `MinSpeed` | float | — | Minimum motor speed (°/s) |
| `MaxSpeed` | float | — | Maximum motor speed (°/s) |
| `Forward` | bool | `True` | `True` = forward, `False` = backward |
| `Stopping` | bool | `True` | Whether to brake after completion |
