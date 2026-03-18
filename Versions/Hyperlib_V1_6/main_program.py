from Hyperlib_v1_6 import*
import Hyperlib_v1_6 as lib

async def my_mission():
    """Your main mission programming goes here"""
    global PArray
    await OdomIni(0,0,0)      
    await TurnToPoint(-300,300)
    await MoveToPoint(-300,300)
    await MoveToPoint(0,0,Forward=False)                         
    await SwingToPoint(1,600,600,Stopping=False)
    lib.PArray = [[600,600],[200,500],[0,400],[0,0]]   
    await PurePursuit(MaxSpeed=900)                  
    await TurnGyro(0) 

async def odometry_task():
    while True:
        await OdomUpdate()                              #Calculate changes in coordinate
        await wait(LoopTime)                            #Wait for a certain amount of time to not overwhelm the processing system

async def main():
    await multitask(my_mission(), odometry_task())      #concurrently execute odometry and your task

async def view():
    """Debug function: continuously print current heading."""
    while True:
        print(hub.imu.heading())  

run_task(main())                                        #Run the concurrent programming