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
	our_partner =  our_attacker if self._role != 'attacker' else our_defender

        world = self._world
        role = self._role
        robot = self._robot
        
        state = self._current_state

        our_zone = self._world.pitch.zones[our_robot.zone]

	if state == self.INITIAL_STATE:
		if our_zone.isInside(ball.x, ball.y):
		    self._current_state = self.ACQUIRING_BALL_STATE
		    self._current_task = AcquireBall(world, robot, role)
		    self._current_task.execute()
		    return
		elif self._world.pitch.zones[our_partner.zone].isInside(ball.x, ball.y):
		    self._current_state = self.RECEIVING_STATE
		    return
		else:
		    self._current_state = self.MIRROR_BALL
		    return

	elif state == self.ACQUIRING_BALL_STATE:
		if not our_zone.isInside(ball.x, ball.y):
		    robot.stop()
		    self._current_state = self.INITIAL_STATE
		    self._current_task = None
		    return
		if isinstance(self._current_task, AcquireBall):
			if self._current_task.complete:
				self._current_state = self.SHOOT
				self._current_task = None
			else:
				self._current_task.execute()
		else:
			self._current_task = AcquireBall(world, robot, role)

	elif state == self.RECEIVING:
		if not self._world.pitch.zones[our_partner.zone].isInside(ball.x, ball.y):
		    robot.stop()
		    self._current_state = self.INITIAL_STATE
		    self._current_task = None
		    return
		if isinstance(self._current_task, MirrorObject):
			self._current_task.execute()
		else:
			self._current_task = MirrorObject(world, robot, role, our_partner)
	

	elif state == self.MIRROR_BALL:
		if isinstance(self._current_task, MirrorObject):
			self._current_task.execute()
		else:
			self._current_task = MirrorObject(world, robot, role, ball)
			self._current_task.execute()
		self._current_state = self.INITIAL_STATE
		return

	elif state == self.SHOOT:
		if our_robot.get_displacement_to_point(ball.x, ball.y)) > 30:
			robot.stop()
			self._current_state = self.INITIAL_STATE
			self._current_task = None
			return
		if self._current_task == None:
			(x,y) = choose_attacker_destination(world)
			self._current_task = MoveToPoint(world, robot, role, x, y)
		if isinstance(self._current_task, MoveToPoint) and self._current_task.complete:
			if abs(their_goal.y + 30 - their_defender.y) > abs(their_goal.y - 30 - their_defender.y):
        			y = their_goal.y + 20
    			else:
       				y = their_goal.y - 20
			self._current_task = KickToPoint(world, robot, role, their_goal.x, y)
			self._current_task.execute()
		if isinstance(self._current_task, KickToPoint) and self._current_task.complete:
			robot.stop()
			self._current_state = self.INITIAL_STATE
			self._current_task = None

	else:
		robot.stop()
		self._current_state = self.INITIAL_STATE
		self._current_task = None
		

		

