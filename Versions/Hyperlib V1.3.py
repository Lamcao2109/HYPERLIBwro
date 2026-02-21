from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task, StopWatch
from pybricks.iodevices import PUPDevice
from umath import sin, cos, radians, atan2, degrees,exp

# ---------- Global Variables ----------
global Steep, MaxDomain, MinSpeed, MaxSpeed, MaxAcc, LoopTime, lastMM, lastTheta, GKp, odomX, odomY, OdometrySwitch

# Initialize hub
hub = PrimeHub()
# Set Bluetooth button as stop button (prevents accidental program stop)
hub.system.set_stop_button(Button.BLUETOOTH)

# ---------- Constants ----------
# PID controller constants for various movements
GKp = 20          # Proportional gain for heading correction during movement
GKd = 250         # Derivative gain for heading correction (currently disabled)
EncTurnKp = 10    # Gain for encoder-based turning (not used in current code)
DistanceKp = 5    # Gain for distance control
SingleTurnKp = 20 # Gain for single motor swing turns
DoubleTurnKp = 7  # Gain for two-motor turns
OdometrySwitch = True  # Enable/disable odometry updates during movements

# ---------- Robot Geometry ----------
# Motor setup with proper direction configuration
LeftMotor = Motor(Port.A,Direction.COUNTERCLOCKWISE)  # Left motor reversed
RightMotor = Motor(Port.E)                            # Right motor standard
WheelD = 62        # Wheel diameter in mm
Axle = 170         # Axle length (distance between wheels) in mm

# --------- Motor Control ---------
LoopTime = 12      # Control loop time in milliseconds (update frequency)
MaxAcc = 30000    # Maximum acceleration in degrees per second squared
MinSpeed = 200     # Minimum motor speed in degrees per second
MaxSpeed = 1350    # Maximum motor speed in degrees per second

# --------- Odometry Variables ---------
# Track robot position and orientation for navigation
odomX = 0          # Current X position in mm
odomY = 0          # Current Y position in mm
lastMM = 0         # Last measured average wheel distance (for delta calculation)
lastTheta = 0      # Last measured heading angle in degrees

# ---------- Helper Functions ----------

def DegToMM(deg):
    """Convert motor degrees to millimeters traveled based on wheel diameter."""
    # Formula: (degrees / 360) * circumference = (degrees / 360) * π * diameter
    return deg / 360 * 3.14159 * WheelD

def ThetaErr(angle):
    """Calculate heading error with proper wrapping to range [-180, 180] degrees."""
    # Get compass-style difference between current heading and target
    compass = (((hub.imu.heading() - angle) % 360) + 360) % 360
    # Convert to signed error (-180 to 180)
    zeta = compass if compass <= 180 else compass - 360
    return zeta

# ---------- Motor Control Functions ----------

async def LeftControl(speed, err):
    """Control left motor with acceleration limits and error correction."""
    global MaxAcc, LoopTime, MinSpeed, MaxSpeed
    
    current_speed = LeftMotor.speed()
    
    # Check acceleration limits
    speed_change = speed - current_speed
    max_change = MaxAcc * (LoopTime / 1000)  # Max change allowed in this time step
    
    if abs(speed_change) > max_change:
        # Limit acceleration to max_change
        if speed_change > 0:
            drive_speed = current_speed + max_change
        else:
            drive_speed = current_speed - max_change
    else:
        drive_speed = speed
    
    # Enforce speed limits
    if abs(drive_speed) < MinSpeed and drive_speed != 0:
        # Ensure minimum speed for motor movement
        drive_speed = MinSpeed if drive_speed > 0 else -MinSpeed
    elif drive_speed > MaxSpeed:
        drive_speed = MaxSpeed
    elif drive_speed < -MaxSpeed:
        drive_speed = -MaxSpeed
    
    # Apply error correction (err adjusts for heading/position correction)
    LeftMotor.run(drive_speed + err)

async def RightControl(speed, err):
    """Control right motor with acceleration limits and error correction."""
    global MaxAcc, LoopTime, MinSpeed, MaxSpeed
    
    current_speed = RightMotor.speed()
    
    # Check acceleration limits
    speed_change = speed - current_speed
    max_change = MaxAcc * (LoopTime / 1000)
    
    if abs(speed_change) > max_change:
        # Limit acceleration to max_change
        if speed_change > 0:
            drive_speed = current_speed + max_change
        else:
            drive_speed = current_speed - max_change
    else:
        drive_speed = speed
    
    # Enforce speed limits
    if abs(drive_speed) < MinSpeed and drive_speed != 0:
        drive_speed = MinSpeed if drive_speed > 0 else -MinSpeed
    elif drive_speed > MaxSpeed:
        drive_speed = MaxSpeed
    elif drive_speed < -MaxSpeed:
        drive_speed = -MaxSpeed
    
    # Apply error correction
    RightMotor.run(drive_speed + err)

# ---------- Odometry & Movements ----------

async def OdomIni(x, y, theta):
    """Initialize odometry with starting position and heading."""
    global odomX, odomY, lastMM, lastTheta
    
    # Reset IMU to specified heading
    hub.imu.reset_heading(theta)
    odomX = x
    odomY = y
    
    # Reset motor encoders to zero
    LeftMotor.reset_angle(0)
    RightMotor.reset_angle(0)
    
    # Initialize tracking variables
    lastMM = (DegToMM(LeftMotor.angle()) + DegToMM(RightMotor.angle())) / 2
    lastTheta = hub.imu.heading()
    

async def OdomUpdate():
    """Calculate and update robot position using wheel encoders and IMU."""
    global odomX, odomY, lastMM, lastTheta
    
    # Calculate current average wheel distance traveled
    left_mm = DegToMM(LeftMotor.angle())
    right_mm = DegToMM(RightMotor.angle())
    current_mm = (left_mm + right_mm) / 2
    
    # Get current heading and convert to radians for trig functions
    current_theta = hub.imu.heading()
    theta_rad = radians(current_theta)
    
    # Calculate distance traveled since last update
    delta_mm = current_mm - lastMM
    
    # Calculate the change in heading
    delta_theta = current_theta - lastTheta
    delta_theta_rad = radians(delta_theta)

    theta_rad = radians((lastTheta + current_theta) / 2)
    delta_x = delta_mm * sin(theta_rad)  # X component of movement
    delta_y = delta_mm * cos(theta_rad)  # Y component of movement
    
    # Update global position
    odomX += delta_x
    odomY += delta_y
    
    # Update last values for next iteration
    lastMM = current_mm
    lastTheta = current_theta
    
    # Return updated position for optional use
    return odomX, odomY, current_theta

async def SwingGyro(Side, Theta, EarlyExit=1, MaxSpeed=1200 , MinSpeed = 400, Forward = True, Stopping = True):
    """
    Swing turn using only one motor (pivot turn).
    Side: 1 = left motor moves (right locked), else = right motor moves (left locked)
    Theta: Target heading angle in degrees
    EarlyExit: Acceptable error range to stop (degrees)
    Forward: True = face forward direction, False = face backward direction
    Stopping: True = apply brakes after completion
    """
    if Side == 1: # Turn using left motor, right motor held stationary
        while True:
            if OdometrySwitch:
                await OdomUpdate()
            # Check if we've reached target heading
            if abs(ThetaErr(Theta)) < EarlyExit:
                break
            OneEighty = 0 if Forward else 180  # Adjust for forward/backward facing
            sign = abs(ThetaErr(Theta+OneEighty))/ThetaErr(Theta+OneEighty)
            print(ThetaErr(OneEighty + Theta))
            if abs(ThetaErr(OneEighty + Theta)) < EarlyExit:
                break
            # Calculate speed based on error
            Speed = ThetaErr(Theta+OneEighty)*SingleTurnKp
            Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
            Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
            # Hold right motor, move left motor
            RightMotor.hold()
            await LeftControl(Speed*sign, 0)
            await wait(LoopTime)
    else: # Turn using right motor, left motor held stationary
        while True:
            if OdometrySwitch:
                await OdomUpdate()
            if abs(ThetaErr(Theta)) < EarlyExit:
                break
            OneEighty = 0 if Forward else 180
            sign = abs(ThetaErr(Theta+OneEighty))/ThetaErr(Theta+OneEighty)
            if abs(ThetaErr(OneEighty + Theta)) < EarlyExit:
                break
            Speed = ThetaErr(Theta+OneEighty)*SingleTurnKp
            Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
            Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
            # Hold left motor, move right motor
            LeftMotor.hold()
            await RightControl(Speed*sign, 0)
            await wait(LoopTime)
    if Stopping:
        await Stop()

async def MoveGyro(Distance, Theta, MinSpeed = 500, MaxSpeed = 1300, Forward = True, Stopping = True):
    """
    Move straight for specified distance while maintaining heading.
    Distance: Distance to travel in mm
    Theta: Heading to maintain during movement (degrees)
    Forward: True = move forward, False = move backward
    Stopping: True = apply brakes after completion
    """
    # Record starting position
    tempMM = (DegToMM(LeftMotor.angle()) + DegToMM(RightMotor.angle())) / 2
    lastErr = 0  # For derivative term (if GKd > 0)
    
    while True:
        if OdometrySwitch == True:
            await OdomUpdate()
        
        # Calculate current distance traveled
        currentMM = (DegToMM(LeftMotor.angle()) + DegToMM(RightMotor.angle())) / 2
        
        # Determine direction (forward or backward)
        Dir = 1 if Forward else -1
        
        # Calculate speed based on remaining distance
        Speed = (Distance*Dir - (currentMM - tempMM))*DistanceKp
        Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
        Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
        
        # Calculate heading correction (PID)
        Prop = ThetaErr(Theta) * GKp  # Proportional term
        Deri = (ThetaErr(Theta) - lastErr)*GKd  # Derivative term
        err = Prop + Deri  # Total correction
        lastErr = ThetaErr(Theta)
        
        # Apply correction: left motor gets -err, right motor gets +err
        await LeftControl(Speed, -err)
        await RightControl(Speed, err)
        
        # Check if target distance reached
        if abs(currentMM - tempMM) >= abs(Distance):
            break
        
        await wait(LoopTime)
    
    if Stopping:
        await Stop()

async def TurnGyro(Theta, MaxSpeed = 1000, MinSpeed = 300, EarlyExit = 4, Forward = True, Direction = 0, Stopping = True):
    """
    Turn in place using both motors.
    Theta: Target heading angle in degrees
    EarlyExit: Acceptable error range to stop (degrees)
    Forward: True = face forward, False = face backward after turn
    Direction: 0 = shortest path, 1 = force clockwise, 2 = force counterclockwise
    Stopping: True = apply brakes after completion
    """
    # Record starting motor positions for encoder-based correction
    tempLeft = LeftMotor.angle()
    tempRight = RightMotor.angle()
    
    while True:
        if OdometrySwitch:
            await OdomUpdate()
        
        # Calculate encoder difference for additional correction
        dLeft = LeftMotor.angle() - tempLeft
        dRight = RightMotor.angle() - tempRight
        Error = dLeft + dRight  # Encoder-based error term
        
        OneEighty = 0 if Forward else 180
        
        # Check if target heading reached
        if abs(ThetaErr(Theta)+OneEighty) < EarlyExit:
            break
        
        # Calculate turn speed based on heading error
        Speed = ThetaErr(Theta+OneEighty)*DoubleTurnKp
        Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
        Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
        
        # Apply forced direction if specified
        if Direction == 0:
            Speed = Speed  # Use calculated speed (could be positive or negative)
        elif Direction == 1:
            Speed = -abs(Speed)  # Force clockwise (negative)
        elif Direction == 2:
            Speed = abs(Speed)   # Force counterclockwise (positive)
        
        # Apply turn: left motor goes opposite direction of right motor
        await LeftControl(-Speed, -Error)
        await RightControl(Speed, Error)
        
        await wait(LoopTime)
    
    if Stopping:
        await Stop()

async def TurnToPoint(TargetX, TargetY, MaxSpeed=800, MinSpeed = 500, EarlyExit = 5, Forward = True, Direction = 0, Stopping = True):
    """
    Turn to face a specific point (x,y) in the field.
    TargetX, TargetY: Coordinates of point to face (mm)
    EarlyExit: Acceptable angular error to stop (degrees)
    Forward: True = face point with front, False = face point with back
    Direction: 0 = shortest path, 1 = force clockwise, 2 = force counterclockwise
    Stopping: True = apply brakes after completion
    """
    global odomX, odomY
    
    # Calculate vector to target
    dx = TargetX - odomX
    dy = TargetY - odomY
    
    # Record starting encoder positions for correction
    tempLeft = LeftMotor.angle()
    tempRight = RightMotor.angle()
    
    while True:
        if OdometrySwitch:
            await OdomUpdate()
        
        # Calculate encoder-based correction
        dLeft = LeftMotor.angle() - tempLeft
        dRight = RightMotor.angle() - tempRight
        Error = dLeft + dRight
        
        OneEighty = 0 if Forward else 180
        
        # Calculate required heading to face target point
        target_angle = degrees(atan2(dx, dy))
        
        # Check if facing target
        if abs(OneEighty + ThetaErr(target_angle)) < EarlyExit:
            break
        
        # Calculate turn speed
        Speed = ThetaErr(OneEighty + target_angle)*DoubleTurnKp
        Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
        Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
        
        # Apply forced direction if specified
        if Direction == 0:
            Speed = Speed
        elif Direction == 1:
            Speed = -abs(Speed)  # Force clockwise
        elif Direction == 2:
            Speed = abs(Speed)   # Force counterclockwise
        
        # Execute turn
        await LeftControl(-Speed, -Error)
        await RightControl(Speed, Error)
        
        await wait(LoopTime)
    
    if Stopping:
        await Stop()

async def SwingToPoint(Side, TargetX, TargetY, EarlyExit=3, MaxSpeed=1200 , MinSpeed = 500, Forward = True, Stopping = True):
    """
    Swing turn to face a specific point using one motor.
    Side: 1 = pivot on right wheel, else = pivot on left wheel
    TargetX, TargetY: Coordinates of point to face (mm)
    EarlyExit: Acceptable angular error to stop (degrees)
    Forward: True = face point with front, False = face point with back
    Stopping: True = apply brakes after completion
    """
    dx = TargetX - odomX
    dy = TargetY - odomY
    
    if Side == 1:  # Pivot on right wheel, left motor moves
        while True:
            if OdometrySwitch:
                await OdomUpdate()
            
            # Determine direction sign
            sign = abs(ThetaErr(degrees(atan2(dx,dy))))/ThetaErr(degrees(atan2(dx,dy)))
            OneEighty = 0 if Forward else 180
            
            # Check if facing target
            if abs(ThetaErr(OneEighty + degrees(atan2(dx,dy)))) < EarlyExit:
                break
            
            # Calculate swing speed
            Speed = ThetaErr(OneEighty + degrees(atan2(dx,dy)))*SingleTurnKp
            Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
            Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
            print(ThetaErr(OneEighty + degrees(atan2(dx,dy))))
            
            # Hold right motor, move left motor
            RightMotor.hold()
            await LeftControl(Speed*sign, 0)
            await wait(LoopTime)
    else:  # Pivot on left wheel, right motor moves
        while True:
            if OdometrySwitch:
                await OdomUpdate()
            
            sign = abs(ThetaErr(degrees(atan2(dx,dy))))/ThetaErr(degrees(atan2(dx,dy)))
            OneEighty = 0 if Forward else 180
            
            if abs(OneEighty + ThetaErr(degrees(atan2(dx,dy)))) < EarlyExit:
                break
            
            Speed = ThetaErr(degrees(atan2(dx,dy)))*SingleTurnKp
            Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
            Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
            
            # Hold left motor, move right motor
            LeftMotor.hold()
            await RightControl(Speed*-sign, 0)
            await wait(LoopTime)
    
    if Stopping:
        await Stop()

async def MoveToPoint(targetX, targetY, MaxSpeed=1300, MinSpeed=400, ArrivalThreshold=15, Forward = True, Stopping = True):
    """
    Drive to a specific point (x,y) while maintaining proper heading.
    targetX, targetY: Destination coordinates (mm)
    ArrivalThreshold: Distance threshold to consider arrival (mm)
    Forward: True = move forward to point, False = move backward to point
    Stopping: True = apply brakes after completion
    """
    global odomX, odomY, GKp, GKd, LoopTime
    
    lastErr = 0  # For derivative term
    
    while True:
        if OdometrySwitch == True:
            await OdomUpdate()
        
        # Calculate vector to target
        dx = targetX - odomX
        dy = targetY - odomY
        
        # Calculate remaining distance
        distanceToGo = (dx**2 + dy**2)**0.5
        
        # Check if arrived at target
        if distanceToGo < ArrivalThreshold:
            break
            
        OneEighty = 0 if Forward else 180
        
        # Calculate required heading to face target while moving
        target_theta = degrees(atan2(dx, dy)) + OneEighty
        error = ThetaErr(target_theta)
        
        # Calculate speed based on distance (P-controller)
        Speed = distanceToGo * DistanceKp
        if abs(Speed) > MaxSpeed: Speed = MaxSpeed * (Speed/abs(Speed))
        if abs(Speed) < MinSpeed: Speed = MinSpeed * (Speed/abs(Speed))
        
        # Determine direction sign
        dSign = 1 if Forward else -1
        
        # Calculate heading correction (PID)
        prop = error * GKp
        deri = (error - lastErr) * GKd
        correction = prop + deri
        lastErr = error
        
        # Apply movement with heading correction
        await LeftControl(dSign*Speed, -correction)
        await RightControl(dSign*Speed, correction)
        
        await wait(LoopTime)
        
    if Stopping:
        await Stop()

async def MoveToPoRo(targetX, targetY, targetAngle, offsetDistance=250, finalDistance=50, offsetAngle=20,
                     MaxSpeed=1000, MinSpeed=300, ArrivalThreshold=20, minRatio=0.8,
                     Forward=True, DistanceKp=DistanceKp, Stopping=True):
    """
    Move to a point with a curved approach (Position + Orientation control).
    This creates a smooth curved path that approaches the target from a specific direction.
    
    Args:
        targetX, targetY: Destination coordinates (mm)
        targetAngle: Desired final heading at destination (degrees)
        offsetDistance: Distance from target to place virtual waypoint (mm)
        finalDistance: Distance at which to start final approach (mm)
        offsetAngle: Angular offset for virtual waypoint placement (degrees)
        MaxSpeed: Maximum movement speed (deg/s)
        MinSpeed: Minimum movement speed (deg/s)
        ArrivalThreshold: Distance to consider arrived (mm)
        minRatio: Minimum blending ratio for angle interpolation
        Forward: True = move forward, False = move backward
        DistanceKp: Proportional gain for distance-based speed control
        Stopping: True = apply brakes after completion
    
    How it works:
    1. Creates a virtual point offset from target at specified angle/distance
    2. While far away (> offsetDistance), robot heads toward virtual point
    3. As robot gets closer, smoothly blends between the heading of the target point
       and desired final heading
    4. This creates a curved path that approaches target from correct orientation
    """
    global odomX, odomY, GKp, GKd, LoopTime
    
    lastErr = 0  # For derivative term in heading correction
    OneEighty = 0 if Forward else 180  # Adjust for forward/backward movement
    finalDistance = min(offsetDistance, finalDistance)  # Ensure final distance isn't larger than offset

    while True:
        if OdometrySwitch:
            await OdomUpdate()  # Update current position

        # Calculate virtual point position (offset from target)
        totalThetaOffset = radians(targetAngle + 180 + offsetAngle)
        virtualX = targetX + offsetDistance * sin(totalThetaOffset)
        virtualY = targetY + offsetDistance * cos(totalThetaOffset)

        # Calculate vectors to target and virtual point
        dx = targetX - odomX
        dy = targetY - odomY
        dVx = virtualX - odomX
        dVy = virtualY - odomY

        # Calculate remaining distance to target
        distanceToGo = (dx**2 + dy**2)**0.5
        
        # Determine target heading based on distance
        if distanceToGo > offsetDistance:
            # Far away: aim for virtual point
            target_theta = degrees(atan2(dVx, dVy)) + OneEighty
        else:
            # Getting close: blend between the desired heading and heading to the desired position
            # The closer we get, the more weight on final heading
            distanceRatio = max(distanceToGo / offsetDistance, minRatio)
            target_theta = (targetAngle * distanceRatio + 
                           degrees(atan2(dx, dy)) * (1 - distanceRatio) + 
                           OneEighty)

        # Calculate heading error
        error = ThetaErr(target_theta)

        # Check if we've arrived
        if distanceToGo < ArrivalThreshold:
            break

        # Calculate speed based on remaining distance
        Speed = distanceToGo * DistanceKp
        if abs(Speed) > MaxSpeed: 
            Speed = MaxSpeed * (Speed/abs(Speed))
        if abs(Speed) < MinSpeed: 
            Speed = MinSpeed * (Speed/abs(Speed))
        
        # Determine direction sign (forward or backward)
        dSign = 1 if Forward else -1
        
        # Calculate heading correction (PID)
        prop = error * GKp
        deri = (error - lastErr) * GKd
        correction = prop + deri
        lastErr = error
        
        # Apply movement with heading correction
        await LeftControl(dSign * Speed, -correction)
        await RightControl(dSign * Speed, correction)
        
        await wait(LoopTime)

    if Stopping:
        await Stop()

async def Stop():
    """Stop both motors smoothly with braking and final hold."""
    timer = StopWatch()
    timer.reset()
    
    # Apply brakes for 40ms to smoothly stop
    while timer.time() < 40:
        OdomUpdate()  # Update odometry during stop
        LeftMotor.brake()
        RightMotor.brake() 
    # Finally hold position
    LeftMotor.hold()
    RightMotor.hold()


async def main():
    """Main Program."""
    await OdomIni(0,0,0)
    await TurnToPoint(500,500)
    await MoveToPoRo(600,600,0, DistanceKp= 3)
    await TurnToPoint(200,-300,Forward=False)
    await MoveToPoint(200,-300,Forward=False)
    await TurnToPoint(0,0)
    await MoveToPoint(0,0)
    await TurnGyro(0)

async def view():
    """Debug function: continuously print current heading."""
    while True:
        print(hub.imu.heading())

# Run the main program
run_task(main())
