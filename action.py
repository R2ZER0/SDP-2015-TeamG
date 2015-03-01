# action.py

class ActionHandle(object):
    """A handle on the result of commands"""
    def __init__(self, idx, cmd):
        self._finished = False
        self._completed = False
        self._running = False
        self.idx = idx
        self.cmd = cmd

    @property
    def finished(self):
        return self._finished
    
    @property
    def completed(self):
        return self._completed
    
    @property
    def running(self):
        return self._running
    
    # Helpers to update the state of this handle
    def _onCancel(self):
        self._completed = True
        self._running = False
        
    def _onFinish(self):
        self._finished = True
        self._running = False
        self._completed = True
        
    def _onRunning(self):
        self._running = True
        
    def _onNextCommand(self):
        if(not self._completed):
            self._onCancel()
            
class MovementActionHandle(ActionHandle):
    """A handle to a movement-related command"""
    def __init__(self, idx, cmd, dir, spd):
        super(MovementActionHandle, self).__init__(idx, cmd)
        self.dir = dir
        self.spd = spd
        
class KickerActionHandle(ActionHandle):
    """A handle to kicker commands"""
    def __init__(self, idx, cmd, spd):
        super(KickerActionHandle, self).__init__(idx, cmd)
        self.spd = spd
        
class CatcherActionHandle(ActionHandle):
    """A handle to catcher commands"""
    def __init__(self, idx, cmd, spd):
        super(CatcherActionHandle, self).__init__(idx, cmd)
        self.spd = spd

# A simple (approx) encoding for floats
def i2f(i):
    return float(i)/1024.0

def f2i(f):
    return int(f*1024)

def mkangle(a):
    while a > math.pi:
        a -= math.pi
    while a < (0 - math.pi):
        a += math.pi
    return a

class Action():
    """Deals directly with the robot, to cater for all your robot commanding needs"""
    def __init__(self, comm):
        self.comm = comm    
        
        # Command handles
        self.move = MovementActionHandle(1, 'S', 
        self.kick = None
        self.catch = None
        
        # Last known MPU output
        self.curr_dir = 0.0
        
    # Movement commands
    def _cmd_movement(self, cmd, angle, scale):
        self.move._onNextCommand()
        self.move = MovementActionHandle(self.move.idx+1, cmd, angle, scale)
        return self.move
    
    def move(self, angle, scale=100):
        return self._cmd_movement('M', angle, scale)
        
    def turnBy(self, angle, scale=100):
        return self._cmd_movement('T', mkangle(self.curr_dir + angle), scale)
        
    def stop(self):
        return self._cmd_movement(self.move.idx+1, 'S', 0, 0)
        
     # Kicker command
    def kick(self, scale=100):
        self.kick._onNextCommand()
        self.kick = KickerActionHandle(self.kick.idx+1, 'K', scale)
        return self.kick
    
    # Catcher commands
    def _cmd_catcher(self, cmd, scale):
        self.catch._onNextCommand()
        self.catch = CatcherActionHandle(self.catch.idx+1, cmd, scale)
        return self.catch
    
    def catch(self, scale=100):
        return self._cmd_catcher('C', scale)
    
    def open_catcher(self, scale=100):
        return self._cmd_catcher('R', scale)

