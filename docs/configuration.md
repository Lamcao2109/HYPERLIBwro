# Configuration

All configuration is done at the top of `Hyperlib_v1_6.py`. You **must** update the hardware parameters to match your robot.

## Hardware Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Left Motor | Port A | Reversed (CounterClockwise) |
| Right Motor | Port E | Standard |
| `WheelD` | based on your robot | Wheel diameter (mm) |
| `Axle` | based on your robot | Distance between wheels (mm) |
| `LoopTime` | 10 ms | Control loop update frequency |
| `MinSpeed` | input your motors' min °/s | Minimum motor speed |
| `MaxSpeed` | input your motors' max °/s | Maximum motor speed |
| `MaxAcc` | input your motors' max °/s² | Maximum acceleration |

!!! warning
    Incorrect `WheelD` and `Axle` values will cause **all** distance and odometry calculations to be wrong. Measure carefully!

## PID Gains

| Constant | Value | Purpose |
|----------|-------|---------|
| `GKp` | 20 | Proportional gain for gyro heading correction |
| `GKd` | 250 | Derivative gain for gyro heading correction |
| `ENCKp` | 1 | Proportional gain for encoder straightening |
| `ENCKd` | 10 | Derivative gain for encoder straightening |
| `DistanceKp` | 4 | Proportional gain for distance-based speed |
| `GyroSingleTurnKp` | 20 | Gain for gyro-based swing turns |
| `ENCSingleTurnKp` | 12 | Gain for encoder-based swing turns |
| `DoubleTurnKp` | 8 | Gain for two-motor point turns |

## Odometry Switch

Set `OdometrySwitch = False` to disable automatic odometry updates inside movement functions. This is useful if you are running odometry as a separate background task (recommended in v1.6+).
