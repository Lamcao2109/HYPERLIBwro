# Odometry

Odometry is a method to estimate the robot's position using wheel encoder data and an IMU (Inertial Measurement Unit). Hyperlib supports two odometry methods: **linear odometry** and **arc odometry**.

## Linear Odometry

Linear odometry is best suited for straight movements. It assumes the robot travels in a straight line between encoder readings and updates position using simple trigonometry.

## Arc Odometry

Arc odometry is the optimal method for curved movement. Instead of assuming straight-line travel, it models the robot's path as an arc, which is more accurate when the robot is turning while moving.

Hyperlib uses arc odometry by default in `OdomUpdate()` for accurate position tracking on both straight and curved paths.

## Mathematical Proof

Below are the mathematical derivations behind both odometry methods. Look for symmetries between these diagrams and the code to better understand how odometry works:

### Arc Odometry Proof

![Arc Odometry Proof](https://github.com/Lamcao2109/HYPERLIBwro/raw/main/Media/Arc%20Odometry%20Proof.png)

### Linear Odometry Proof

![Linear Odometry Proof](https://github.com/Lamcao2109/HYPERLIBwro/raw/main/Media/Linear%20Odometry%20Proof.png)

## How It Works in Practice

Odometry runs as a **concurrent background task** using Pybricks' `multitask`. This means the robot's position is always up to date during any movement. The recommended structure is:

```python
async def odometry_task():
    while True:
        await OdomUpdate()
        await wait(LoopTime)

async def main():
    await multitask(my_mission(), odometry_task())
```

!!! note
    `multitask` is not true parallelism — it is cooperative multitasking. Both tasks share the same CPU and take turns. The `await wait(...)` calls inside movement functions are the yield points.
