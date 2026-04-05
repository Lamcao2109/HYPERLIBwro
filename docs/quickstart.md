# Quick Start

## Repository Structure

```
├── Versions/
│   ├── Hyperlib_V1_4/
│   ├── Hyperlib_V1_5/
│   └── Hyperlib_V1_6/          ← Current version
│       ├── Hyperlib_v1_6.py    ← Main library
│       └── main_program.py     ← Your program goes here
├── Hyperlib V1.3.py
└── README.md
```

## Setup

1. Place [`Hyperlib_v1_6.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/Hyperlib_v1_6.py) and [`main_program.py`](https://github.com/Lamcao2109/HYPERLIBwro/blob/main/Versions/Hyperlib_V1_6/main_program.py) in the same directory on your SPIKE Prime hub.

2. Write your autonomous routine inside `main_program.py`:

```python
from Hyperlib_v1_6 import *
import Hyperlib_v1_6 as lib

async def my_mission():
    await OdomIni(0, 0, 0)        # Start at origin, facing 0°
    await TurnToPoint(-300, 300)  # Face target coordinate
    await MoveToPoint(-300, 300)  # Drive to it
    lib.PArray = [[0,0],[200,300],[400,0]]
    await PurePursuit()           # Follow the path

async def odometry_task():
    while True:
        await OdomUpdate()        # Keep position tracking running
        await wait(LoopTime)

async def main():
    await multitask(my_mission(), odometry_task())

run_task(main())
```

!!! warning "First Run"
    Before running any program, update your robot's physical specs at the top of `Hyperlib_v1_6.py` to match your actual hardware:

    ```python
    WheelD = 62     # ← your wheel diameter in mm
    Axle   = 170    # ← your axle width in mm (wheel center to wheel center)
    ```

    Incorrect values will cause all distance and odometry calculations to be wrong.

## Important Notes

- All movement functions are `async` and must be called with `await`.
- `PurePursuit` requires odometry to be running — always pair it with `odometry_task()` inside `multitask`.
- The **Bluetooth button** is configured as the stop button to prevent accidental program termination.
- For debugging, use the `view()` function in `main_program.py` to print the current IMU heading.
- Hyperlib uses **arc odometry** in `OdomUpdate` for accurate position tracking.
