
import math
import motor_calibration


class Action():
    '''Provides an interface between desired movement and the Robot. Accepts
    a communication object to write to.

    Attributes:
        Debug           Setting True enables verbose output of commands.
        MOTOR_ANGLES    The angles of our wheels relative to the x-axis.
                        These begin from front-left and move anticlockwise.
        MOTORS          Trigonometric calculations for each motor, used for 
                        holonomic movement
    ''' 

    Debug = False

    MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    def __init__(self, comm):
        '''Initialises an Action instance with the given communication object.

        Attributes:
            comm    A pySerial serial object for communication
        '''
        self.comm = comm
    
    # Public methods #

    def move(self, angle, scale):
        """Moves the robot in the given angle with given scale as power value.

        Attributes:
            angle   The direction of movement, in radians, relative to x-axis 
                    of robot
            scale   Power-scale for this movement, range: -1 to 1
        """

        motor_speeds = [ Action._calc_motor_speed(motor, angle) for motor in self.MOTORS ]
        motor_speeds = Action._normalise_speeds(motor_speeds)
        motor_speeds = map(lambda x: x*scale, motor_speeds)

        # Scale motor speeds to be in the range [-100,100]
        motor_speeds = map(Action._percentage_speed, motor_speeds)
        
        self._send_run(motor_speeds)
  
    def turn(self, speed):
        """Turn the robot with the given power.

        Attributes:
            speed   The speed of the turn, range is [-100,100]. Positive values
                    result in clockwise rotation, negative in anticlockwise.
        """

        # Speed must be an integer, otherwise movement may fail
        speed = int(speed)

        # Motor 3 requires negation due to fitting
        self._send_run([speed, speed, -speed])
  
    def stop(self):
        """Stops the robot's movement by setting all motors to zero.
        """
        self._send_run([0,0,0])
  
    def kick(self, scale=100):
        """Sends the kick command to the robot. Performs no checking of the 
        kicker's state prior to kicking.

        Attributes:
            scale   Power scale for this kick. Range [0,100]. Default 100.
        """
        # Note, scale must be in list form
        self._send_command("KICK",[scale])
    
    def catch(self, scale=100):
        """Sends the catch command to the robot. Performns no checking of the 
        catcher's position prior to catching.

        Attributes:
            scale   Power scale for this catch. Range [0,100]. Default 100.
        """
        self._send_command("CATCH",[scale])

    def open_catcher(self, scale=100):
        """Sends the command to open the catcher to the robot. Currently 
        implemented simply as a kick.

        Attributes:
            scale   Power scale for the catch. Range [0,100]. Default 100.
        """
        self.kick(scale)

    # Utility
    def ping(self):
        """Sends the PING command to the Robot. Robot should return "PONG" via
        the serial connection if correctly functioning.
        """
        self._send_command("PING", [])
    
    # Private Methods #
    
    # Commands
    def _send_run(self, speeds):
        """Convenience function for sending the given motor speeds to the Robot.

        Attributes:
            speeds  A list of motor speeds for each motor. Range [-100,100].
        """
        self._send_command("RUN", speeds)

    # Utility
    @staticmethod
    def _normalise_speeds(speeds):
        maxspeed = max([abs(s) for s in speeds])
        return map( lambda x: (x/maxspeed), speeds )        
    
    @staticmethod
    def _percentage_speed(speed):
        """Constrains speed to be within the range [-100,100].

        Attributes:
            speed   A speed in the range [-1,1]
        """
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
    def _get_command_string(command, *args):
        """Constructs a suitable syntax command string for sending the given
        command to the Robot with supplied arguments.

        Attributes:
            command     A command string
            args        None, single argument, multiple argument, or a list.
        """
        commstr = str(command)
        
        # Append each argument with a space
        for arg in args:
            commstr = commstr + " " + str(arg)
            
        # Terminate with correct newline syntax
        return commstr + "\r\n"            
    
    def _send_command(self, command, *args):
        """Sends the given command to the robot, with arguments.

        Attributes:
            command     A command string
            args        None, single argument, multiple argument, or a list.
        """
        commstr = Action._get_command_string(command, args)

        if self.Debug:
            print "Sending command: " + commstr
        
        if self.comm is not None:
            self.comm.write(commstr)
            self.comm.flush()
    
    
    
