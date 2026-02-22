#This is where your main program goes

from Hyperlib_v1_5 import* # import your desired library

async def main():
    global PArray
    """Main Program."""
    await OdomIni(0,0,0)
    await SwingMM(2,-300)
    await TurnGyro(0)

async def view():
    """Debug function: continuously print current heading."""
    while True:
        print(hub.imu.heading())

# Run the main program
run_task(main())