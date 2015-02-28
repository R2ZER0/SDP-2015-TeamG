# control.state

class KickerState(object):
    """Represents the state of the kicker"""
    # Actions
    ACTION_STOPPED = "STOPPED"
    ACTION_KICKING = "KICKING"
    ACTION_RETURNING = "RETURNING"
    
    @classmethod
    def parse(cls, string):
        parts = string.split(" ")
        assert(parts[0] == "KICKER")
        
        action = 
    
    def __init__(self, action=STATE_STOPPED):
        self._current_action = action
        
    @property
    def action(self):
        return self._current_action

class CatcherState(object):
    pass

class MovementState(object):
    pass