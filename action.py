# movement.py
import math

class Action():
    """Provides an interface to movement and other robot actions"""

    Debug = False # set Action.Debug = True in your application to enable
    
    # Motor angles, starting from the front left, going anticlockwise
    # MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
    MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    def __init__(self, comm):
        self.comm = comm
    
    # Public methods #
    
    # Movement
    def move(self, angle, scale):
        """Move the robot in the angle (radians) from the front, with speed -1 to +1"""
        motor_speeds = [ Action._calc_motor_speed(motor, angle) for motor in self.MOTORS ]
        motor_speeds = Action._normalise_speeds(motor_speeds)
        motor_speeds = map(lambda x: x*scale, motor_speeds)
        motor_speeds = map(Action._percentage_speed, motor_speeds)
        
        self._send_run(motor_speeds)
  
    def turn(self, speed):
        """Turn the robot in the given direction, clockwise/anticlockwise"""
        speed = int(speed)
        self._send_run([-speed, speed, speed])
  
    def stop(self):
        motor_speeds = [ 0.0, 0.0, 0.0 ]
        self._send_run(motor_speeds)
  
    # Kicking
    def kick(self, scale=100):
        self._send_command("KICK",[scale])
    
    def catch(self, scale=100):
        self._send_command("CATCH",[scale])

    # Utility
    def ping(self):
        self._send_command("PING", [])
    
    
    # Private Methods #
    
    # Commands
    def _send_run(self, speeds):
        self._send_command("RUN", speeds)
    
    # TODO: decide on the kick protocol etc.
  
    # Utility
    @staticmethod
    def _normalise_speeds(speeds):
        maxspeed = max(speeds)
        return map( lambda x: (x/maxspeed)*x, speeds )        
    
    @staticmethod
    def _percentage_speed(speed):
        # We need the speed to be an integer between 100 and -100
        speed = int(round(100.0 * speed))
        speed = max(-100, min(speed, 100))
        return speed
    
    @staticmethod
    def _calc_motor_speed(motor, angle):
        return (math.cos(angle) * motor[0] - math.sin(angle) * motor[1])
    
    @staticmethod
    def _get_command_string(command, args):
        commstr = str(command)
        
        for arg in args:
            commstr = commstr + " " + str(arg)
            
        commstr = commstr + "\r\n"            
        return commstr
    
    def _send_command(self, command, args):
        commstr = Action._get_command_string(command, args)
        if self.Debug:
            print "Sending command: " + commstr
        if self.comm is not None:
            self.comm.write(commstr)
            self.comm.flush()
    
    
    
