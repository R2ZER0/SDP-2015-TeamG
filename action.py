# movement.py
import math

class Action():
    """Provides an interface to movement and other robot actions"""
  
    TURN_CLOCKWISE = 'c'
    TURN_ANTICLOCKWISE = 'a'
    
    # Motor angles, starting from the front left, going anticlockwise
    # MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
    MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    def __init__(self, comm):
        self.comm = comm
    
    # Public methods #
    
    # Movement
    def move(self, angle, speed):
        """Move the robot in the angle (radians) from the front, with speed -1 to +1"""
        motor_speeds = [ self._calc_motor_speed(motor, angle, speed) for motor in self.MOTORS ]
        
        self._send_run(motor_speeds)
  
    def turn(self, speed):
        """Turn the robot in the given direction, clockwise/anticlockwise"""      
        motor_speed = self._normalise_speed(speed)
        self._send_run([motor_speed, motor_speed, motor_speed])
  
    def stop(self):
        motor_speeds = [ 0.0, 0.0, 0.0 ]
        self._send_run(motor_speeds)
  
    # Kicking
    def kick(self):
        self._send_command("KICK",[])
    
    def catch(self):
        self._send_command("CATCH",[])
    
    
    # Private Methods #
    
    # Commands
    def _send_run(self, speeds):
        self._send_command("RUN", speeds)
    
    # TODO: decide on the kick protocol etc.
  
    # Utility
    def _normalise_speed(self, speed):
        # We need the speed to be an integer between 100 and -100
        speed = int(round(100.0 * speed))
        speed = max(-100, min(speed, 100))
        return speed
    
    def _calc_motor_speed(self, motor, angle, linear_speed):
        speed = linear_speed * (math.cos(angle) * motor[0] - math.sin(angle) * motor[1] )
        return self._normalise_speed(speed)
    
    def _get_command_string(self, command, args):
        commstr = str(command)
        
        for arg in args:
            commstr = commstr + " " + str(arg)
            
        commstr = commstr + "\r\n"            
        return commstr
    
    def _send_command(self, command, args):
        commstr = self._get_command_string(command, args)
        print("Writing command: " + commstr)
        if self.comm is not None:
            self.comm.write(commstr)
            self.comm.flush()
    
    
    
