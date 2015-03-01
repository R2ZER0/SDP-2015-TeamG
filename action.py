# action.py
import re
import threading
import atexit
import time
import signal

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
        
    def _onComplete(self):
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
    def __init__(self, idx, spd):
        super(KickerActionHandle, self).__init__(idx, 'K')
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
    
    # Regex for parsing state messages
    msg_re = re.compile('\((\d+) ([SMT]) (\d+) (1|0) (\d+) (1|0) (\d+) ([CRI]) (1|0) (\d+)\)')
    
    def __init__(self, comm):
        self.comm = comm    
        
        # Command handles
        self.move = MovementActionHandle(1, 'S', 0, 0)
        self.kick = KickerActionHandle(0, 0)
        self.catch = CatcherActionHandle(1, 'I', 0)
        
        # Last known MPU output
        self.curr_dir = 0.0
        
        # Comms threads
        self._exit = False
        def set_exit():
            self._exit = True
        atexit.register(set_exit)
        
        self.prev_handler = None
        def handler(signum, frame):
            self._exit = True
            self.prev_handler(signum, frame)
        self.prev_handler = signal.signal(2, handler)
        
        self.recv_thread = threading.Thread(target=lambda: self.run_state_processor())
        self.send_thread = threading.Thread(target=lambda: self.run_state_sender())
        
        self.recv_thread.start()
        self.send_thread.start()
    
    def exit(self):
        self._exit = True
        
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
    
    # State processing
    def process_message(self, message):
        """Parse and process the state message from the arduino"""
        #'\((\d+) ([SMT]) (\d+) (1|0) (\d+) (1|0) (\d+) ([CRI]) (1|0) (\d+)\)'
        #    mID   mCMD   mDIR  mFIN   kID  kFIN   cID   cCMD    cFIN  sDIR
        #     1      2      3     4     5     6     7      8      9     10
        res = Action.msg_re.match(message)
        
        if res is not None:
            move_id = int(res.group(1))
            move_cmd = res.group(2)
            move_dir = i2f(int(res.group(3)))
            move_fin = (1 == int(res.group(4)))
            
            kick_id = int(res.group(5))
            kick_fin = (1 == int(res.group(6)))
            
            catch_id = int(res.group(7))
            catch_cmd = res.group(8)
            catch_fin = (1 == int(res.group(9)))
            
            state_dir = i2f(int(res.group(10)))
            self.curr_dir = state_dir
            
            # Detect if the command is running, finished etc
            if move_id == self.move.idx:
                if not self.move.running:
                    self.move._onRunning()
                    
                if move_fin and not self.move.finished:
                    self.move._onComplete()
            
            if kick_id == self.kick.idx:
                if not self.kick.running:
                    self.kick._onRunning()
                    
                if kick_fin and not self.kick.finished:
                    self.kick._onComplete()
                    
            if catch_id == self.catch.idx:
                if not self.catch.running:
                    self.catch._onRunning()
                    
                if catch_fin and not self.catch.finished:
                    self.catch._onComplete()
                
    def run_state_processor(self):
        """Processes incoming state messages"""
        while not self._exit:
            self.comm.timeout = 0.1
            line = self.comm.readline()
            self.process_message(line)
            
    def run_state_sender(self):
        """Sends out state messages"""
        while not self._exit:
            time.sleep(1/20)
            message = "({0} {1} {2} {3} {4} {5} {6} {7} {8} {9})".format(
                self.move.idx, self.move.cmd, f2i(self.move.dir), self.move.spd,
                self.kick.idx, self.kick.cmd, self.kick.spd,
                self.catch.idx, self.catch.cmd, self.catch.spd
            )
            
            self.comm.write(message)
