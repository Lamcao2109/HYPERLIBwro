#This is where your main program goes

from Hyperlib_v1_4 import* # import your desired library

async def main():
    global PArray
    """Main Program."""
    await OdomIni(0,0,0)
    await TurnToPoint(300,300)
    await MoveToPoint(300,300)
    await TurnToPoint(0,0)
    await MoveToPoint(0,0)
    await TurnGyro(0)

async def view():
    """Debug function: continuously print current heading."""
    while True:
        print(hub.imu.heading())

# Run the main program
run_task(main())