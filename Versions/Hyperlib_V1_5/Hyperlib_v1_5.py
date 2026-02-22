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
GKp = 12          # Proportional gain for heading correction during movement
GKd = 170         # Derivative gain for heading correction (currently disabled)
ENCKp = 1         # Proportional gain for ENC-based correction 
ENCKd = 10        # Derivative gain for ENC-based correction
EncTurnKp = 1     # Proportional gain for ENC-based correction while turning
EncTurnKd = 0     # Derivative gain for ENC-based correction while turning
DistanceKp = 4    # Gain for distance control
GyroSingleTurnKp = 20 # Gain for single motor gyro-based swing turns
ENCSingleTurnKp = 12  # Gain for single motor ENC-based swing turns
DoubleTurnKp = 8  # Gain for two-motor turns
OdometrySwitch = True  # Enable/disable odometry updates during movements

# ---------- Robot Geometry ----------
# Motor setup with proper direction configuration
LeftMotor = Motor(Port.A,Direction.COUNTERCLOCKWISE)  # Left motor reversed
RightMotor = Motor(Port.E)                            # Right motor standard
WheelD = 62        # Wheel diameter in mm
Axle = 170         # Axle length (distance between wheels) in mm

# --------- Motor Control ---------
LoopTime = 10      # Control loop time in milliseconds (update frequency)
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
    """Calculate and update robot position using wheel encoders and IMU.
    Uses arc odometry for accurate position estimation."""
    global odomX, odomY, lastMM, lastTheta
    
    # Calculate current average wheel distance traveled
    left_mm = DegToMM(LeftMotor.angle())
    right_mm = DegToMM(RightMotor.angle())
    current_mm = (left_mm + right_mm) / 2
    
    # Get current heading
    current_theta = hub.imu.heading()
    
    # Calculate distance traveled since last update
    delta_mm = current_mm - lastMM
    
    # Calculate the change in heading
    delta_theta = current_theta - lastTheta
    delta_theta_rad = radians(delta_theta)
    
    # Arc odometry - accurate for both straight and curved paths
    if abs(delta_theta_rad) > 0.17:  # Small threshold to avoid division by zero
        # radius = arc length / angle
        radius = delta_mm / delta_theta_rad
        
        # Calculate position change using arc geometry with current heading
        # The robot ends at angle current_theta, having started at lastTheta
        delta_x = radius * (sin(radians(current_theta)) - sin(radians(lastTheta)))
        delta_y = radius * (cos(radians(lastTheta)) - cos(radians(current_theta)))
    else:
        # Nearly straight line movement
        delta_x = delta_mm * sin(radians(current_theta))
        delta_y = delta_mm * cos(radians(current_theta))
    
    # Update global position
    odomX += delta_x
    odomY += delta_y
    
    # Update last values for next iteration
    lastMM = current_mm
    lastTheta = current_theta
    
    # Return updated position for optional use
    return odomX, odomY, current_theta

async def MoveMM(Distance, MaxSpeed=1200, MinSpeed=400,
                  Forward=True, Stopping=True):
    """
    Move straight for a specified distance using only wheel encoders.
    Straightness is maintained by adjusting motor powers based on the
    difference between left and right wheel displacements (P‑controller).

    Distance:   Distance to travel in mm (positive)
    MaxSpeed:   Maximum motor speed in deg/s
    MinSpeed:   Minimum motor speed in deg/s
    Forward:    True = move forward, False = move backward
    Stopping:   True = apply brakes after completion
    """
    # Record starting position
    tempMM = (DegToMM(LeftMotor.angle()) + DegToMM(RightMotor.angle())) / 2
    lastErr = 0  # For derivative term (if ENCKd > 0)
    last_left_mm = DegToMM(LeftMotor.angle())
    last_right_mm = DegToMM(RightMotor.angle())

    while True:
        if OdometrySwitch == True:
            await OdomUpdate()
        
        # Calculate current distance traveled
        currentMM = (DegToMM(LeftMotor.angle()) + DegToMM(RightMotor.angle())) / 2
        dLeft = DegToMM(LeftMotor.angle()) - last_left_mm
        dRight = DegToMM(RightMotor.angle()) - last_right_mm

        # Determine direction (forward or backward)
        Dir = 1 if Forward else -1
        
        # Calculate speed based on remaining distance
        Speed = (Distance*Dir - (currentMM - tempMM))*DistanceKp
        Speed = Speed if abs(Speed) <= MaxSpeed else MaxSpeed*(Speed/abs(Speed))
        Speed = Speed if abs(Speed) >= MinSpeed else MinSpeed*(Speed/abs(Speed))
        
        # Calculate ENC - based correction (PID)
        Err = dLeft - dRight
        Prop = Err*ENCKp
        Deri = (Err-lastErr)*ENCKd
        Sum = Prop + Deri  # Total correction
        lastErr = Err
        
        # Apply correction: left motor gets -err, right motor gets +err
        await LeftControl(Speed, -Sum)
        await RightControl(Speed, Sum)
        
        # Check if target distance reached
        if abs(currentMM - tempMM) >= abs(Distance):
            break
        
        await wait(LoopTime)
    
    if Stopping:
        await Stop()


async def TurnMM(turn_mm, MaxSpeed=1000, MinSpeed=300, EarlyExit=3,
                  Stopping=True):
    """
    Turn in place using both motors, controlled by wheel encoders.
    Both wheels move equal distances in opposite directions until each
    has traveled the specified millimeter amount.

    turn_mm:    Distance each wheel should travel (mm). Positive = clockwise.
    MaxSpeed:   Maximum motor speed in deg/s
    MinSpeed:   Minimum motor speed in deg/s
    EarlyExit:  Acceptable error to stop (mm) – difference from target
    Stopping:   True = apply brakes after completion
    """
    # Record starting encoder distances (in mm)
    start_left_mm = DegToMM(LeftMotor.angle())
    start_right_mm = DegToMM(RightMotor.angle())

    # For a point turn, left and right move opposite directions.
    # Positive target = clockwise => left backward, right forward.
    target_left = start_left_mm + turn_mm
    target_right = start_right_mm - turn_mm

    lastError = 0

    while True:
        if OdometrySwitch:
            await OdomUpdate()

        # Current positions
        left_mm = DegToMM(LeftMotor.angle())
        right_mm = DegToMM(RightMotor.angle())

        # Remaining errors
        error_left = target_left-left_mm
        error_right = target_right-right_mm

        # Distance moved so far
        d_left = left_mm - start_left_mm
        d_right = right_mm - start_right_mm

        # Translation error (should be zero in ideal turn)
        translation_error = d_left + d_right
        P = translation_error * EncTurnKp
        D = (translation_error - lastError) * EncTurnKd
        correction = P + D
        lastError = translation_error

        # Stop when both wheels are within EarlyExit
        if abs(error_left) < EarlyExit and abs(error_right) < EarlyExit:
            break

        # Average error gives turn rate demand
        avg_error = (error_right-error_left) / 2
        speed = avg_error * DoubleTurnKp

        # Clamp speed
        if abs(speed) > MaxSpeed:
            speed = MaxSpeed if speed > 0 else -MaxSpeed
        elif abs(speed) < MinSpeed:
            speed = MinSpeed if speed > 0 else -MinSpeed

        # Apply turn: left backward, right forward for positive speed (clockwise)
        await LeftControl(-speed, -correction)
        await RightControl(speed, correction)

        await wait(LoopTime)

    if Stopping:
        await Stop()


async def SwingMM(Side, turn_mm, EarlyExit=5, MaxSpeed=1200, MinSpeed=500,
                  Direction=0, Stopping=True):
    """
    Swing turn using only one motor, controlled by the moving wheel's encoder.
    The moving wheel travels the specified millimeter distance while the other
    wheel is held stationary.

    Side:       1 = pivot on right wheel (left motor moves)
                else = pivot on left wheel (right motor moves)
    turn_mm:    Distance the moving wheel should travel (mm). Positive = clockwise.
    EarlyExit:  Acceptable error to stop (mm) – difference from target
    MaxSpeed:   Maximum motor speed in deg/s
    MinSpeed:   Minimum motor speed in deg/s
    Direction:  0 = use sign of 'turn_mm', 1 = force clockwise, 2 = force counterclockwise
    Stopping:   True = apply brakes after completion
    """
    # Record starting encoder distance of the moving wheel
    if Side == 1:          # left motor moves, right motor held
        start_mm = DegToMM(LeftMotor.angle())
    else:                  # right motor moves, left motor held
        start_mm = DegToMM(RightMotor.angle())

    # Determine target displacement (positive = clockwise)
    if Direction == 0:
        target = turn_mm
    elif Direction == 1:
        target = abs(turn_mm)      # force clockwise
    else:  # Direction == 2
        target = -abs(turn_mm)     # force counterclockwise

    # For clockwise (target > 0):
    # - If left motor moves (Side==1), it must go backward → displacement = -target
    # - If right motor moves (Side!=1), it must go forward → displacement = +target
    if Side == 1:
        target_mm = start_mm + target
    else:
        target_mm = start_mm + target

    while True:
        if OdometrySwitch:
            await OdomUpdate()

        if Side == 1:
            current_mm = DegToMM(LeftMotor.angle())
        else:
            current_mm = DegToMM(RightMotor.angle())

        error = target_mm - current_mm
        if abs(error) < EarlyExit:
            break

        # Proportional speed
        raw_speed = error * ENCSingleTurnKp
        mag = abs(raw_speed)
        if mag > MaxSpeed:
            mag = MaxSpeed
        elif mag < MinSpeed:
            mag = MinSpeed

        # Determine speed sign based on Direction (if forced) or error sign
        if Direction == 0:
            speed = raw_speed
        elif Direction == 1:  # force clockwise
            # Clockwise requires:
            # - left motor (Side==1) to go backward → negative speed
            # - right motor (Side!=1) to go forward → positive speed
            if Side == 1:
                speed = -mag
            else:
                speed = mag
        else:  # Direction == 2, force counterclockwise
            if Side == 1:
                speed = mag
            else:
                speed = -mag

        if Side == 1:
            RightMotor.hold()
            await LeftControl(speed, 0)
        else:
            LeftMotor.hold()
            await RightControl(speed, 0)

        await wait(LoopTime)

    if Stopping:
        await Stop()

async def SwingGyro(Side, Theta, EarlyExit=1, MaxSpeed=1200, MinSpeed=400,
                    Forward=True, Direction=0, Stopping=True):
    """
    Swing turn using only one motor (pivot turn).
    Side: 1 = left motor moves (right locked), else = right motor moves (left locked)
    Theta: Target heading angle in degrees
    EarlyExit: Acceptable error range to stop (degrees)
    Forward: True = face forward direction, False = face backward direction
    Direction: 0 = shortest path, 1 = force clockwise, 2 = force counterclockwise
    Stopping: True = apply brakes after completion
    """
    OneEighty = 0 if Forward else 180
    target_angle = Theta + OneEighty

    if Side == 1:  # Turn using left motor, right motor held
        while True:
            if OdometrySwitch:
                await OdomUpdate()

            error = ThetaErr(target_angle)
            if abs(error) < EarlyExit:
                break

            # Compute speed magnitude from error
            raw_speed = error * GyroSingleTurnKp
            mag = abs(raw_speed)
            if mag > MaxSpeed:
                mag = MaxSpeed
            elif mag < MinSpeed:
                mag = MinSpeed

            # Determine sign based on Direction and Side
            if Direction == 0:
                sign = 1 if error >= 0 else -1
            elif Direction == 1:  # clockwise
                # Left motor positive -> clockwise
                sign = 1
            elif Direction == 2:  # counterclockwise
                # Left motor negative -> counterclockwise
                sign = -1

            speed = sign * mag

            RightMotor.hold()
            await LeftControl(speed, 0)
            await wait(LoopTime)

    else:  # Turn using right motor, left motor held
        while True:
            if OdometrySwitch:
                await OdomUpdate()

            error = ThetaErr(target_angle)
            if abs(error) < EarlyExit:
                break

            raw_speed = error * GyroSingleTurnKp
            mag = abs(raw_speed)
            if mag > MaxSpeed:
                mag = MaxSpeed
            elif mag < MinSpeed:
                mag = MinSpeed

            # Determine sign based on Direction and Side
            if Direction == 0:
                sign = 1 if error >= 0 else -1
            elif Direction == 1:  # clockwise
                # Right motor negative -> clockwise
                sign = -1
            elif Direction == 2:  # counterclockwise
                # Right motor positive -> counterclockwise
                sign = 1

            speed = sign * mag

            LeftMotor.hold()
            await RightControl(speed, 0)
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

async def TurnGyro(Theta, MaxSpeed = 1000, MinSpeed = 300, EarlyExit = 5, Forward = True, Direction = 0, Stopping = True):
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
    lastError = 0
    
    while True:
        if OdometrySwitch:
            await OdomUpdate()
        
        # Calculate encoder difference for additional correction
        dLeft = LeftMotor.angle() - tempLeft
        dRight = RightMotor.angle() - tempRight
        Error = dLeft + dRight  # Encoder-based error term
        Proportional = Error * EncTurnKp
        Derivative = (Error-lastError) * EncTurnKd
        Sum = Proportional + Derivative
        lastError = Error 
        
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

async def TurnToPoint(TargetX, TargetY, MaxSpeed=1000, MinSpeed = 400, EarlyExit = 3, Forward = True, Direction = 0, Stopping = True):
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

async def SwingToPoint(Side, TargetX, TargetY, EarlyExit=3, MaxSpeed=1200, MinSpeed=500,
                       Forward=True, Direction=0, Stopping=True):
    """
    Swing turn to face a specific point using one motor.
    Side: 1 = pivot on right wheel (left motor moves), else = pivot on left wheel (right motor moves)
    TargetX, TargetY: Coordinates of point to face (mm)
    EarlyExit: Acceptable angular error to stop (degrees)
    Forward: True = face point with front, False = face point with back
    Direction: 0 = shortest path, 1 = force clockwise, 2 = force counterclockwise
    Stopping: True = apply brakes after completion
    """
    global odomX, odomY

    # Vector to target
    dx = TargetX - odomX
    dy = TargetY - odomY
    target_angle = degrees(atan2(dx, dy))
    OneEighty = 0 if Forward else 180
    desired = target_angle + OneEighty

    if Side == 1:  # Pivot on right wheel, left motor moves
        while True:
            if OdometrySwitch:
                await OdomUpdate()

            error = ThetaErr(desired)
            if abs(error) < EarlyExit:
                break

            raw_speed = error * GyroSingleTurnKp
            mag = abs(raw_speed)
            if mag > MaxSpeed:
                mag = MaxSpeed
            elif mag < MinSpeed:
                mag = MinSpeed

            if Direction == 0:
                sign = 1 if error >= 0 else -1
            elif Direction == 1:  # clockwise
                sign = 1   # left motor positive -> clockwise
            elif Direction == 2:  # counterclockwise
                sign = -1  # left motor negative -> counterclockwise

            speed = sign * mag

            RightMotor.hold()
            await LeftControl(speed, 0)
            await wait(LoopTime)

    else:  # Pivot on left wheel, right motor moves
        while True:
            if OdometrySwitch:
                await OdomUpdate()

            error = ThetaErr(desired)
            if abs(error) < EarlyExit:
                break

            raw_speed = error * GyroSingleTurnKp
            mag = abs(raw_speed)
            if mag > MaxSpeed:
                mag = MaxSpeed
            elif mag < MinSpeed:
                mag = MinSpeed

            if Direction == 0:
                sign = 1 if error >= 0 else -1
            elif Direction == 1:  # clockwise
                sign = -1  # right motor negative -> clockwise
            elif Direction == 2:  # counterclockwise
                sign = 1   # right motor positive -> counterclockwise

            speed = sign * mag

            LeftMotor.hold()
            await RightControl(speed, 0)
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

async def MoveToPoRo(targetX, targetY, targetAngle, offsetDistance=250, finalDistance=70, offsetAngle=20,
                     MaxSpeed=1000, MinSpeed=300, ArrivalThreshold=25, minRatio=0.8,
                     Forward=True, DKp=DistanceKp, Stopping=True):
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
        DKp: Proportional gain for distance-based speed control
        Stopping: True = apply brakes after completion
    
    How it works:
    1. Creates a virtual point offset from target at specified angle/distance
    2. While far away (> offsetDistance), robot heads toward virtual point
    3. As robot gets closer, smoothly blends between the heading of the target point
       and desired final heading
    4. This creates a curved path that approaches target from correct orientation
    """
    global odomX, odomY, GKp, GKd, LoopTime, DistanceKp
    
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
        Speed = distanceToGo * DKp
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


