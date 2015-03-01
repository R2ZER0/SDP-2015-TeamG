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

        world = self._world
        role = self._role
        robot = self._robot
        
        state = self._current_state

        our_zone = self._world.pitch.zones[robot.zone]

        self._current_state = INITIAL_STATE
        self._current_task = None

        if passing:
            if state == INITIAL_STATE:
                if our_zone.isInside(ball.x, ball.y):       # fine for stationary ball starting in our zone
                    self._current_state = ACQUIRING_BALL_STATE
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()
                else:
                    return 

            elif state == ACQUIRING_BALL_STATE:
                if not robot.has_ball(ball):
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

                else:
                    if role == 'attacker':
                        tm_zone = our_defender.zone
                        teammate = our_defender
                    elif role == 'defender':
                        tm_zone = our_attacker
                        zone = our_attacker.zone
                    (pass_x_dest, pass_y_dest) = get_clear_forward_passing_pos(tm_zone, robot, teammate, their_attacker) # their attacker?
                    self._current_state = MOVING_TO_PT_STATE
                    self._current_task = MoveToPoint(world, robot, role, pass_x_dest, pass_y_dest)
                    self.execute()

            elif state == MOVING_TO_PT_STATE:
                if abs(robot.get_displacement_to_point(self._current_task.x, self._current_task.y)) > 30:
                    self._current_task.execute()

                else:
                    self._current_state = IDLE_STATE
                    sel._current_task = None

            elif state == IDLE_STATE:
                if abs(our_defender.y - our_attacker.y)) < 30:
                    self._current_state = PASSING_STATE
                    if role == 'attacker':
                        self._current_task = KickToPoint(world, robot, role, our_defender.x, our_defender.y)    
                    elif role == 'defender':
                        self._current_task = KickToPoint(world, robot, role, our_attacker.x, our_attacker.y) 
                    self._current_task.execute()
                else:
                    return

            elif state == PASSING_STATE:
                if not self._current_task.kicked:
                    self._current_task.execute()

                else:
                    self._current_state = INITIAL_STATE
                    self._current_task = None   
        else:
            if state == INITIAL_STATE:
                self._current_state = MOVING_TO_PT_STATE
                if role == 'attacker':
                    self._current_task = MoveToPoint(world, robot, role, robot.x, our_defender.y)
                elif role == 'defender':
                    self._current_task = MoveToPoint(world, robot, role, robot.x, our_attacker.y)
                self._current_task.execute()

            elif state == MOVING_TO_PT_STATE:
                ## Should we use prediction instead since this way is probably too slow in term of reaction time?
                ## Violates the passing protocol.
                ## Perhaps is unnecessary work and should just leave as is - receiving robot doesn't *need* to actually
                ## catch the ball 
                if not our_zone.isInside(ball.x, ball.y):   
                    if role == 'attacker':
                        self._current_task = MoveToPoint(world, robot, role, robot.x, our_defender.y)
                    elif role == 'defender':
                        self._current_task = MoveToPoint(world, robot, role, robot.x, our_attacker.y)
                    self._current_task.execute()

                else:
                    self._current_state = ACQUIRING_BALL_STATE
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

            elif state == ACQUIRING_BALL_STATE:
                if not robot.has_ball(ball):
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

                else:
                    self._current_state = INITIAL_STATE
                    self._current_task = None
                
                
