# control.command
# Contains the Command class

VALID_COMMANDS = ["RUN", "TURN", "KICK", "CATCH", "RELEASE", "MPU"]
VALID_MPU_COMMANDS = ["INIT", "SETHOME", "GETYAW", "STABL", "CALIB"]
    
# Don't create these directly, instead use action.xyz()
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
        return (self.command in VALID_COMMANDS)
    
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
               (self.args[0] in VALID_MPU_COMMANDS))