import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *

class Planner:

    def __init__(self, world, robot, role, passing):
        # Initialisation
        assert role in ['attacker','defender']
        assert passing in [True, False]

        self._world = world
        self._robot = robot
        self._role = role
        self.passing = passing

        # Encode states
        self.INITIAL_STATE = 'INITIAL_STATE'
        self.ACQUIRING_BALL_STATE = 'ACQUIRING_BALL_STATE'
        self.MOVING_TO_PT_STATE = 'MOVING_TO_PT_STATE'
        self.IDLE_STATE = 'IDLE_STATE'
        self.PASSING_STATE = 'PASSING_STATE'

        self._current_state=self.INITIAL_STATE
        self._current_task=None
    
    def plan(self):
        '''Catch-all function for the Planner, uses our role, state, and the world model
        to determine what we should be doing next.
        '''
        
        ball = self._world.ball

        our_attacker = self._world.our_attacker
        our_defender = self._world.our_defender
        their_attacker = self._world.their_attacker
        their_defender = self._world.their_defender

        our_robot = our_attacker if self._role == 'attacker' else our_defender


        world = self._world
        role = self._role
        robot = self._robot
        
        state = self._current_state

        our_zone = self._world.pitch.zones[our_robot.zone]

        if self.passing:
            if state == self.INITIAL_STATE:
                if our_zone.isInside(ball.x, ball.y):       # fine for stationary ball starting in our zone
                    self._current_state = self.ACQUIRING_BALL_STATE
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()
                else:
                    return 

            elif state == self.ACQUIRING_BALL_STATE:

                if not our_zone.isInside(ball.x, ball.y):
                    robot.stop()
                    self._current_state = self.INITIAL_STATE
                    self._current_task = None
                    return

                if isinstance(self._current_task, AcquireBall):
            if self._current_task.complete:
                if role == 'attacker':
                    teammate = our_defender
                elif role == 'defender':
                    teammate = our_attacker
                (pass_x_dest, pass_y_dest) = get_clear_forward_passing_pos(world, our_robot, teammate, their_attacker) # their attacker?
                print pass_x_dest, pass_y_dest
                self._current_state = self.MOVING_TO_PT_STATE
                self._current_task = MoveToPoint(world, robot, role, pass_x_dest, pass_y_dest)
                self._current_task.execute()
            else:
                self._current_task.execute()
                else:
            self._current_task = AcquireBall(world, robot, role)
                    

            elif state == self.MOVING_TO_PT_STATE:
                if abs(our_robot.get_displacement_to_point(self._current_task.x, self._current_task.y)) > 30:
                    self._current_task.execute()
                else:
                    self._current_state = self.IDLE_STATE
                    robot.stop()
                    self._current_task = None

            elif state == self.IDLE_STATE:
                if abs(our_defender.y - our_attacker.y) < 30:
                    self._current_state = self.PASSING_STATE
                    if role == 'attacker':
                        self._current_task = KickToPoint(world, robot, role, our_defender.x, our_defender.y)    
                    elif role == 'defender':
                        self._current_task = KickToPoint(world, robot, role, our_attacker.x, our_attacker.y) 
                    self._current_task.execute()
                else:
                    return

            elif state == self.PASSING_STATE:
                if not self._current_task.complete:
                    self._current_task.execute() 
        else:
		self._current_state = 'RECEIVING'
		if self._current_task == None:
			if role == 'attacker':
				self._current_task = MirrorObject(world, robot, role, world.our_defender)
	   		else:
				self._current_task = MirrorObject(world, robot, role, world.our_attacker)               
		else:
			self._current_task.execute() 
                

