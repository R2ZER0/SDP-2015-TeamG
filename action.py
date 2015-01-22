# movement.py
import math

class Action():
    """Provides an interface to movement and other robot actions"""
  
    TURN_CLOCKWISE = 'c'
    TURN_ANTICLOCKWISE = 'a'
    
    # Motor angles, starting from the front left, going anticlockwise
    MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    def __init__(self, serial):
        self.serial = serial;
    
    # Public methods #
    
    # Movement
    def move(self, angle, speed):
        """Move the robot in the angle (radians) from the front"""
        motor_speeds = [ _calc_motor_speed(motor, angle, speed) for motor in MOTORS ]
        
        self._send_run(motor_speeds)
  
    def turn(self, direction, speed):
        """Turn the robot in the given direction, clockwise/anticlockwise"""
        motor_speed = 0.0
        
        if(direction == TURN_CLOCKWISE):
            motor_speed = speed
        elif(direction == TURN_ANTICLOCKWISE):
            motor_speed = -speed
      
        self._send_run([motor_speed, motor_speed, motor_speed])
  
    def stop(self):
        motor_speeds = [ 0.0, 0.0, 0.0 ]
        self._send_run(motor_speeds);
  
    # Kicking
    def kick(self):
        pass
    
    def catch(self):
        pass
    
    
    # Private Methods #
    
    # Commands
    def _send_run(self, speeds):
        self._send_command("RUN", speeds);
    
    # TODO: decide on the kick protocol etc.
  
    # Utility
    def _calc_motor_speed(motor, angle, linear_speed):
        return linear_speed * (math.cos(angle) * motor[0] - math.sin(angle) * motor[1] )
    
    def _get_command_string(command, args):
        commstr = str(command)
        
        for arg in args:
            commstr = commstr + " " + str(arg)
            
        return commstr
    
    def _send_command(command, args):
        self.serial.write( _get_command_string(command, args) )
    
    
    