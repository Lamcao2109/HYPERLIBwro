# Coordinate-Based Movement API

## `MoveToPoint(targetX, targetY, MaxSpeed, MinSpeed, ArrivalThreshold, Forward, Stopping)`

Drives to a target coordinate `(targetX, targetY)` using a P-controller for distance-based speed and a PD-controller for heading correction. Stops within `ArrivalThreshold` mm of the target.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `targetX` | float | — | Target X coordinate (mm) |
| `targetY` | float | — | Target Y coordinate (mm) |
| `MaxSpeed` | float | `900` | Maximum motor speed (°/s) |
| `MinSpeed` | float | `250` | Minimum motor speed (°/s) |
| `ArrivalThreshold` | float | `25` | Distance considered "arrived" (mm) |
| `Forward` | bool | `True` | `True` = forward, `False` = backward |
| `Stopping` | bool | `True` | Whether to brake on arrival |

## `MoveToPoRo(targetX, targetY, targetAngle, offsetDistance, finalDistance, offsetAngle, MaxSpeed, MinSpeed, ArrivalThreshold, minRatio, Forward, DKp, Stopping)`

Drives to a coordinate while controlling the **final approach angle** (Position + Orientation). Places a virtual waypoint behind the target and smoothly blends the robot's heading toward `targetAngle` as it gets closer. Ideal for docking or alignment tasks.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `targetX` | float | — | Target X coordinate (mm) |
| `targetY` | float | — | Target Y coordinate (mm) |
| `targetAngle` | float | — | Desired heading when arriving (degrees) |
| `offsetDistance` | float | `250` | Distance behind target for virtual waypoint (mm) |
| `finalDistance` | float | — | Final approach distance |
| `offsetAngle` | float | `20` | Angular offset of virtual waypoint (degrees) |
| `MaxSpeed` | float | `900` | Maximum motor speed (°/s) |
| `MinSpeed` | float | `250` | Minimum motor speed (°/s) |
| `ArrivalThreshold` | float | `25` | Distance considered "arrived" (mm) |
| `minRatio` | float | `0.8` | Minimum blend ratio during final approach |
| `Forward` | bool | `True` | Drive forward or backward |
| `DKp` | float | — | Distance proportional gain |
| `Stopping` | bool | `True` | Whether to brake on arrival |

## `PurePursuit(MaxSpeed, MinSpeed, ArrivalThreshold, Lookahead, Forward, DKp, Stopping)` {#purepursuit}

Follows a sequence of waypoints stored in `PArray` using the **Pure Pursuit** algorithm. The robot continuously steers toward a lookahead point on the path, producing smooth curved trajectories.

Before calling `PurePursuit`, set `PArray` to a list of `[x, y]` waypoints:

```python
import Hyperlib_v1_6 as lib
lib.PArray = [[0, 0], [200, 300], [400, 100], [600, 0]]
await PurePursuit(MaxSpeed=900)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `MaxSpeed` | float | `900` | Maximum motor speed (°/s) |
| `MinSpeed` | float | `250` | Minimum motor speed for final approach (°/s) |
| `ArrivalThreshold` | float | `25` | Distance to waypoint considered "reached" (mm) |
| `Lookahead` | float | `50` | Lookahead radius — larger = smoother but less precise (mm) |
| `Forward` | bool | `True` | `True` = forward, `False` = backward |
| `DKp` | float | — | Distance proportional gain |
| `Stopping` | bool | `True` | Whether to brake after reaching the final waypoint |

!!! tip
    See [Pure Pursuit Concepts](../concepts/pure-pursuit.md) for a detailed explanation of the algorithm.
