# control.action
# Functions to generate commands
import math
    
# Motor angles, starting from the front left, going anticlockwise
MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]

# Utilities
def _normalise_speeds(speeds):
    maxspeed = max(speeds)
    return map( lambda x: (x/maxspeed)*x, speeds )        

def _percentage_speed(speed):
    # We need the speed to be an integer between 100 and -100
    speed = int(round(100.0 * speed))
    speed = max(-100, min(speed, 100))
    return speed

def _calc_motor_speed(motor, angle):
    return (math.cos(angle) * motor[0] - math.sin(angle) * motor[1])
    
    
# Actions
def move(angle, scale):
    """Move the robot in the angle (radians) from the front, with speed -1 to +1"""
    motor_speeds = [ _calc_motor_speed(motor, angle) for motor in MOTORS ]
    motor_speeds = _normalise_speeds(motor_speeds)
    motor_speeds = [ x*scale for x in motor_speeds ]
    motor_speeds = map(_percentage_speed, motor_speeds)
    
    return [ Command("RUN", motor_speeds) ]

def turn(speed):
    """Turn the robot in the given direction, clockwise/anticlockwise"""
    speed = int(speed)
    return [ Command("RUN", [speed, speed, -speed]) ]

def stop():
    """Stooooppp!!"""
    return [ Command("RUN", []) ]

# Kicking
def kick(scale=100):
    """Kick the kicker"""
    return [ Command("KICK", [scale]) ]

def catch(scale=100):
    """Catch the ball (hopefully)"""
    return [ Command("CATCH", [scale]) ]

# Utility
def initialiseMpu(current_rotation=0.0):
    """Setup the gyroscope - don't move the robot!"""
    return [ MpuCommand("INIT"),
                MpuCommand("STABL"),
                MpuCommand("SETHOME", [current_rotation]) ]

def ping():
    """Make sure the robot is still alive"""    
    return [ Command("PING") ]
