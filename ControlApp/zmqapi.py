# Exposes the Action API via a zeromq socket
import zmq
import json

class ActionZMQ():
    context = zmq.Context()
    
    def __init__(self, action, endpoint, bind=False):
        
        self.action = action
        
        # Initialise our ZMQ socket
        self.socket = self.context.socket(zmq.REQ)
        
        if bind:
            self.socket.bind(endpoint)
        else:
            self.socket.connect(endpoint)
            
    def poll(self):
        """Check for messages on the wire and process them"""
        while True:
            try:
                command = json.loads( self.socket.recv(zmq.NOBLOCK) )
                self._execute(command)
                self.socket.send("ok")
            except zmq.ZMQError:
                break
        
    def _execute(self, command):
        """Execute the command given as a dictionary"""
        if command["cmd"] == "move":
            angle = command["angle"] or 0.0
            speed = command["speed"] or 0.0
            
            self.action.move(angle, speed)
            
        elif command["cmd"] == "turn":
            direction = command["direction"] or 'c'
            speed = command["speed"] or 0.0
            
            self.action.move(direction, speed)
            
        else:
            pass # Unknown command

