# control.commander

from control.command import Command

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
    
    # Command tracking
    _next_tracking_id = 1

    def getNextTrackingID(self):
        """Get next available ID for command tracking"""
        idx = cls._next_tracking_id
        cls._next_tracking_id += 1
        return idx
    
    class TrackedCommand(object):
        """Combines a command and a tracking ID"""
        def __init__(self, command, trid=):