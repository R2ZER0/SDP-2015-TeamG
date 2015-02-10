# movement.py
import math

class Action():
    """Provides an interface to movement and other robot actions"""

    Debug = False # set Action.Debug = True in your application to enable
  
    TURN_CLOCKWISE = 'c'
    TURN_ANTICLOCKWISE = 'a'
    
    # Motor angles, starting from the front left, going anticlockwise
    # MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
    MOTOR_ANGLES = [ math.pi*5.0/3.0, math.pi/2.0, math.pi*4/3.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    def __init__(self, comm):
        self.comm = comm
    
    # Public methods #
    
    # Movement
    def move(self, angle, scale):
        """Move the robot in the angle (radians) from the front, with speed -1 to +1"""
        #motor_speeds = [ Action._calc_motor_speed(motor, angle) for motor in self.MOTORS ]
        motor_speeds = Action._calc_motor_speed(angle)
        motor_speeds = Action._normalise_speeds(motor_speeds)
        motor_speeds = map(lambda x: x*(-100), motor_speeds)
        #motor_speeds = map(Action._percentage_speed, motor_speeds)
        print motor_speeds
        self._send_run([int(motor_speeds[0]), int(motor_speeds[1]), int(motor_speeds[2])])
  
    def turn(self, speed):
        """Turn the robot in the given direction, clockwise/anticlockwise"""      
        #motor_speed = _normalise_speed(speed)
        self._send_run([speed, speed, speed])
  
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
        maxspeed = max([abs(s) for s in speeds])
        return map( lambda x: (x/maxspeed), speeds )        
    
    @staticmethod
    def _percentage_speed(speed):
        # We need the speed to be an integer between 100 and -100
        speed = int(round(100.0 * speed))
        speed = max(-100, min(speed, 100))
        return speed
    
    @staticmethod
    def _calc_motor_speed( angle):
        m = [0,0,0]
        m[0] = math.cos(angle) - math.sin(angle)/(2+math.sqrt(3))
        m[1] = 2*math.sin(angle)/(2+math.sqrt(3))
        m[2] = -math.cos(angle) - math.sin(angle)/(2+math.sqrt(3))
        return m

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
    
    
    
