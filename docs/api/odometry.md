# Odometry API

## `OdomIni(x, y, theta)`

Initializes the robot's position and heading. Resets the IMU to `theta`, sets coordinates to `(x, y)`, and zeroes motor encoders.

| Parameter | Type | Description |
|-----------|------|-------------|
| `x` | float | Initial X coordinate (mm) |
| `y` | float | Initial Y coordinate (mm) |
| `theta` | float | Initial heading (degrees) |

## `OdomUpdate()`

Updates `(odomX, odomY)` using **arc odometry** — accurate for both straight paths and curves. Uses wheel encoder deltas combined with IMU heading.

**Returns:** `(odomX, odomY, current_theta)`

!!! info
    This function should run continuously in a background task. See [Odometry Concepts](../concepts/odometry.md) for the recommended setup.
