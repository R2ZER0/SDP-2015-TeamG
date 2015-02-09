# movement.py
import math


# Don't create these directly, instead use Action.xyz()
class Command():
    """A simple command wrapper class"""
    @classmethod
    def fromCommandString(cls, commandString):
        parts = commandString.split(" ")
        return cls(parts[0], parts[1:])
    
    def __init__(self, command, args=[]):
        self.command = command
        self.args = args
        
        if not self.isValid():
            raise Exception("Attempt to create invalid command" + self)
        
    def getCommandString(self):
        return self.command + " " + (" ".join(self.args))
    
    def isValid(self):
        return (self.command in ["RUN", "TURN", "LED", "KICK", "CATCH", "MPU"])
    
    def __unicode__(self):
        return self.getCommandString()
    
    def __eq__(self, other):
        return (self.command == other.command) and (self.args == other.args)
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
class MpuCommand(Command):
    """Wrapper around MPU commands"""
    def __init__(self, subcmd, args=[]):
        super(MpuCommand, self).__init__("MPU", [subcmd] + args)
        
    def isValid(self):
        return ((super(MpuCommand, self).isValid()) and
               (self.args[0] in ["INIT", "SETHOME", "GETYAW", "STABL", "CALIB"]))


class Commander():
    """Deals directly with the robot, to cater for all your robot commanding needs"""
    def __init__(self, comm):
        self.comm = comm
        
    def executeCommandList(self, commands):            
        """Takes a list of Command objects, and executes them in order"""
        commands = Commander._optimiseCommandList(commands)
        results = map(lambda cmd: self.executeCommand(cmd), commands)
        return results
        
    def executeCommand(self, command):
        """Execute a single command"""
        commandString = command.getCommandString()
        comm.write(commandString + "\r\n")
        comm.flush()
        
        success = False
        result = []
        # wait for output
        while True:
            line = comm.readline().rstrip()
            
            if(line == "DONE"):
                success = True
                break
            elif(line == "FAILED"):
                success = False
                break
            
            result.append(line)
            
        if success:
            return (command, result)
        
        else:
            raise Exception("Command execution failed: " + cmd + " With result: " + " ".join(result))
                
    
    @staticmethod
    def _optimiseCommandList(commands):
        """Simple optimisation on the list of commands"""
        
        commands_to_keep = []
        
        # Filter out all but the last run command
        has_run = False
        for cmd in reversed(commands):
            
            if(cmd.command == "RUN"):
                if(has_run):
                    continue
                has_run = True
                
            commands_to_keep.append(cmd)
            
        return commands_to_keep[::-1]
        

class Action():
    """Higher-level robot activities"""
    
    # Motor angles, starting from the front left, going anticlockwise
    # MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
    MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
    MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]
    
    # Public methods #
    
    # Movement
    @classmethod
    def move(cls, angle, scale):
        """Move the robot in the angle (radians) from the front, with speed -1 to +1"""
        motor_speeds = [ cls._calc_motor_speed(motor, angle) for motor in cls.MOTORS ]
        motor_speeds = cls._normalise_speeds(motor_speeds)
        motor_speeds = map(lambda x: x*scale, motor_speeds)
        motor_speeds = map(lambda x: cls._percentage_speed(x), motor_speeds)
        
        return [ Command("RUN", motor_speeds) ]
  
    @classmethod
    def turn(cls, speed):
        """Turn the robot in the given direction, clockwise/anticlockwise"""
        speed = int(speed)
        return [ Command("RUN", [speed, speed, -speed]) ]
  
    @classmethod
    def stop(cls):
        """Stooooppp!!"""
        return [ Command("RUN", []) ]

    # Kicking
    @classmethod
    def kick(cls, scale=100):
        """Kick the kicker"""
        return [ Command("KICK", [scale]) ]
    
    @classmethod
    def catch(cls, scale=100):
        """Catch the ball (hopefully)"""
        return [ Command("CATCH", [scale]) ]

    # Utility
    @classmethod
    def initialiseMpu(cls, current_rotation=0.0):
        """Setup the gyroscope - don't move the robot!"""
        return [ MpuCommand("INIT"),
                 MpuCommand("STABL"),
                 MpuCommand("SETHOME", [current_rotation]) ]

    @classmethod
    def ping(cls):
        """Make sure the robot is still alive"""    
        return [ Command("PING") ]
    
    # Private Methods #
      
    # Utility
    @classmethod
    def _normalise_speeds(cls, speeds):
        maxspeed = max(speeds)
        return map( lambda x: (x/maxspeed)*x, speeds )        
    
    @classmethod
    def _percentage_speed(cls, speed):
        # We need the speed to be an integer between 100 and -100
        speed = int(round(100.0 * speed))
        speed = max(-100, min(speed, 100))
        return speed
    
    @classmethod
    def _calc_motor_speed(cls, motor, angle):
        return (math.cos(angle) * motor[0] - math.sin(angle) * motor[1])
    
    
