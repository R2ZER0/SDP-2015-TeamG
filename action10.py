# action.py
import re
import threading
import atexit
import time
import signal
import math

class ActionHandle(object):
    """A handle on the result of commands"""
    def __init__(self, idx, cmd, arg1, arg2):
        self._finished = False
        self._completed = False
        self._running = False
        self.idx = idx
        self.cmd = cmd
        self.arg1 = arg1
        self.arg2 = arg2

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
        self._running = False
        if(not self._completed):
            self._onCancel()

class Action10():
    """Deals directly with the robot, to cater for all your robot commanding needs"""
    
    # Regex for parsing state messages
    msg_re = re.compile('\((\d+) ([SMT]) (1|0) (\d+) ([KI]) (1|0) (\d+) ([CRI]) (1|0)\)')
    
    def __init__(self, comm):
        self.comm = comm    
        
        # Command handles
        self.move_handle = MovementActionHandle(100, 'S', 0, 0)
        self.kick_handle = KickerActionHandle(100, 'I', 0)
        self.catch_handle = CatcherActionHandle(100, 'I', 0)
        
        # Last known MPU output
        self.curr_dir = 0.0
        
        # Number of messages received
        self.num_messages_recvd = 0
        
	if self.comm:
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
        
    # Use these to send commands
    def setMovementCommand(self, cmd, arg1, arg2):
        self.move_handle._onNextCommand()
        self.move_handle = ActionHandle(self.move_handle.idx+1, cmd, arg1, arg2)
        return self.move_handle
    
    def setKickerCommand(self, cmd, arg1):
        self.kick_handle._onNextCommand()
        self.kick_handle = ActionHandle(self.kick_handle.idx+1, cmd, arg1, None)
        return self.kick_handle

    def setCatcherCommand(self, cmd):
        self.catch_handle._onNextCommand()
        self.catch_handle = ActionHandle(self.catch_handle.idx+1, cmd, None, None)
        return self.catch_handle

    # Put here extra methods to make sending commands easier
    # e.g. .move(...), .kick(...), .catch(...)
    
    # State processing
    def process_message(self, message):
        """Parse and process the state message from the arduino"""
        #'\((\d+) ([SMT]) (1|0) (\d+) ([KI]) (1|0) (\d+) ([CRI]) (1|0)\)'
        #    mID   mCMD    mFIN   kID   kCMD  kFIN   cID   cCMD   cFIN
        #     1      2       3     4     5      6     7      8     9  
        res = Action.msg_re.match(message)
        
        if res is not None:
            #print "Got msg " + str(self.num_messages_recvd)
            self.num_messages_recvd += 1
            
            move_id = int(res.group(1))
            move_cmd = res.group(2)
            move_fin = (1 == int(res.group(3)))
            
            kick_id = int(res.group(4))
            kick_cmd = res.group(5)
            kick_fin = (1 == int(res.group(6)))
            
            catch_id = int(res.group(7))
            catch_cmd = res.group(8)
            catch_fin = (1 == int(res.group(9)))
            
            # Detect if the command is running, finished etc
            if move_id == self.move_handle.idx:
                if not self.move_handle.running:
                    self.move_handle._onRunning()
                    
                if move_fin and not self.move_handle.finished:
                    self.move_handle._onComplete()
            
            if kick_id == self.kick_handle.idx:
                if not self.kick_handle.running:
                    self.kick_handle._onRunning()
                    
                if kick_fin and not self.kick_handle.finished:
                    self.kick_handle._onComplete()
                    
            if catch_id == self.catch_handle.idx:
                if not self.catch_handle.running:
                    self.catch_handle._onRunning()
                    
                if catch_fin and not self.catch_handle.finished:
                    self.catch_handle._onComplete()
        else:
            #print "# " + message
            pass
        
        
    def run_state_processor(self):
        """Processes incoming state messages"""
        while not self._exit:
	
            self.comm.timeout = 0.1
            line = self.comm.readline()
            line = line.rstrip()
	    #if not line.startswith('dist'):
	    #	print line
            if line != "":
                self.process_message(line)
            
    def run_state_sender(self):
        """Sends out state messages"""
        while not self._exit:
            time.sleep(0.120)
            
            if self.num_messages_recvd > 10:
                message = "({0} {1} {2} {3} {4} {5} {6} {7} {8} {9})".format(
                    self.move_handle.idx, self.move_handle.cmd, self.move_handle.arg1, self.move_handle.arg2,
                    self.kick_handle.idx, self.kick_handle.cmd, self.kick_handle.arg1,
                    self.catch_handle.idx, self.catch_handle.cmd, self.catch_handle.arg1
                )
                
                self.comm.write(message)
