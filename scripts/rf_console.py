# Provides a handy console for configuring the SRF stick
import serial
import sys
import zmq
from threading import Thread, Event
import time
import re

SERIAL_PORT = "/dev/ttyACM0"

if len(sys.argv) >= 2:
    SERIAL_PORT = sys.argv[1]


ZMQ_ENDPOINT = "inproc://cms"
ctx = zmq.Context()

class CommandModeSerial(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._stop = Event()
        
        self.comm = serial.Serial(SERIAL_PORT, 115200)
        self.rep = ctx.socket(zmq.REP)
        self.rep.connect(ZMQ_ENDPOINT)
        
    def stop(self):
        self._stop.set()
        
    def _readline(self, dodebug=True):
        message = ""
        char = ""
        while True:
            char = self.comm.read()
            
            if char == "\r":
                break
            
            message = message + str(char)
            
        return message
    
    def _read_response(self):
        """Get lines of response, until OK or ERR"""
        lines = []
        while True:
            line = self._readline()
            lines.append(line)
            
            regex = re.compile('(OK|ERR)')
            if regex.match(line) is not None:
                break
            
        return "\n".join(lines)
    
    def _get_response_lines(self):
        pass
        
    
    def run(self):
        # Remove all existing chars
        self.comm.flush()
        while self.comm.inWaiting() > 0:
            self.comm.read()
        
        # Enter command mode
        self.comm.write("~~~")
        self.comm.flush()
        self._read_response()
        
        # Enter command loop
        while not self._stop.is_set():
            # Send keepalive
            self.comm.write("AT\r")
            self.comm.flush()
            self._read_response()
            
            events = self.rep.poll(timeout=3000)
            if events > 0:
                # We got a command! pass it onto the serial comm
                message = str(self.rep.recv_string())
                response = ""
                
                if message == "?":
                    if self.comm.inWaiting() > 0:
                        response = self._readline()
                    else:
                        response = "!Buffer Empty"
                        
                elif message == "exit":
                    self.comm.write("ATDN\r")
                    self.comm.flush()
                    self.stop()
                    
                else:
                    self.comm.write(message + "\r\n")
                    self.comm.flush()
                    # Wait for a response, and return it to the terminal
                    response = self._read_response()
                
                self.rep.send_string(response)
                



class CommandModeTerminal():
    def __init__(self):
        self.req = ctx.socket(zmq.REQ)
        self.req.bind(ZMQ_ENDPOINT)
        
    def run(self):
        print 'Running CMT'
        
        while True:
            message = raw_input("> ")
            message.rstrip()
            
            if len(message) < 1:
                continue
            
            self.req.send_string(message)
            
            if message == "exit":
                break
            
            response = str(self.req.recv_string())
            print response
    
if __name__ == '__main__':
    cmt = CommandModeTerminal()
    cms = CommandModeSerial()
    
    cms.start()
    cmt.run()

    cms.stop()