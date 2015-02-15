# action.py

import control.action as action
from control.commander import Commander

class Action(object):
    """A wrapper around control.* for compatability with the old API"""
    def __init__(self, comm):
        self.comm = comm
        self.commander = Commander(comm) 
        
    def move(self, angle, scale):
        pass
    
    def turn(self, speed):
        pass
    
    def stop(self):
        pass
    
    def kick(self, scale=100):
        pass
    
    def catch(self, scale=100):
        pass
    
    def open_catcher(self, scale=100):
        pass
    
    def ping(self):
        pass
