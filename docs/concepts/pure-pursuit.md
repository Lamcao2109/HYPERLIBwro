# Pure Pursuit

Pure Pursuit is a path-tracking algorithm. Instead of driving directly toward each waypoint and stopping, the robot steers toward a **lookahead point** — a point on the path a fixed lookahead distance `L` ahead of the robot. This produces smooth, continuous curves through the entire path.

## How It Works

The robot treats the waypoint array (`PArray`) as a path made of connected line segments. Each control loop iteration:

1. For the current segment, it finds where a circle of radius `Lookahead` centered on the robot intersects the segment.
2. It steers toward that intersection point (the "lookahead point"), not the waypoint itself.
3. When the robot gets within `ArrivalThreshold` of the next waypoint, it advances to the next segment.
4. Once all segments are traversed, it calls `MoveToPoint` to precisely arrive at the final waypoint.

This gives the robot smooth, continuous motion through a path — sharp corners are rounded naturally by the lookahead radius.

## Mathematical Proof

Look for symmetries between this diagram and the code to better understand Pure Pursuit:

![Pure Pursuit Proof](https://github.com/Lamcao2109/HYPERLIBwro/raw/main/Media/Pure%20Pursuit%20Proof.png)

## Usage Example

```python
import Hyperlib_v1_6 as lib

lib.PArray = [[0, 0], [200, 300], [400, 100], [600, 0]]
await PurePursuit(MaxSpeed=900)
```

See the full [PurePursuit API reference](../api/coordinate-movement.md#purepursuit) for all parameters.
