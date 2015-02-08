import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *

class Planner:

    def __init__(self, world, robot, role):
        # Initialisation
        assert role in ['attacker','defender']
        self._world = world
        self._robot = robot
        self._role = role

        # Encode states
        self.INITIAL_STATE = 'INITIAL_STATE'
        self.ACQUIRING_BALL_STATE = 'ACQUIRING_BALL_STATE'
        self.MOVING_TO_POINT_STATE = 'MOVING_TO_POINT_STATE'
        self.SHOOTING_STATE = 'SHOOTING_STATE'
        self.REVERTING_TO_IDLE_STATE = 'REVERTING_TO_IDLE_STATE'
        self.REVERTING_TO_IDLE_ROTATE = 'REVERTING_TO_IDLE_ROTATE'
        self.CLEARING_STATE = 'CLEARING_STATE'

        self._current_state=self.INITIAL_STATE
        self._current_task=None
    
    @property
    def attacker_strat_state(self):
        return self._attacker_current_strategy.current_state

    @property
    def defender_strat_state(self):
        return self._defender_current_strategy.current_state

    @property
    def attacker_state(self):
        return self._attacker_state

    @attacker_state.setter
    def attacker_state(self, new_state):
        assert new_state in ['defence', 'attack']
        self._attacker_state = new_state

    @property
    def defender_state(self):
        return self._defender_state

    @defender_state.setter
    def defender_state(self, new_state):
        assert new_state in ['defence', 'attack']
        self._defender_state = new_state

    def plan(self):
        ball = self._world.ball
        our_attacker = self._world.our_attacker
        our_defender = self._world.our_defender
        their_attacker = self._world.their_attacker

        world = self._world
        role = self._role
        robot = self._robot
        
        state = self._current_state

        #pdb.set_trace()
        if self._role == 'attacker':
            (idle_x, idle_y) = world.pitch.zones[our_attacker.zone].center()

            """Attacker state machine"""
            if state == self.INITIAL_STATE:
                if self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):
                    self._current_state = self.ACQUIRING_BALL_STATE
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

            elif state == self.REVERTING_TO_IDLE_STATE:
                if not(our_attacker.get_displacement_to_point(self._current_task.x, self._current_task.y) == 0) and not(self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y)):
                    """
                    If the robot's displacement from the point we're trying to move to 
                    is't zero, we aren't there yet
                    """
                    self._current_task.execute()

                elif not(our_attacker.get_displacement_to_point(self._current_task.x, self._current_task.y) == 0) and self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):
                    self._current_state=self.ACQUIRING_BALL_STATE
                    self._current_task=AcquireBall(world, robot, role)
                    self._current_task.execute()

                elif our_attacker.get_displacement_to_point(self._current_task.x, self._current_task.y) == 0:
                    self._current_task=None
                    self._current_state=self.INITIAL_STATE

            elif state == self.ACQUIRING_BALL_STATE:
                #if self._current_task.complete:
                #    pdb.set_trace()

                if not(our_attacker.has_ball(ball)) and self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

                elif not(our_attacker.has_ball(ball)) and not(self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y)):
                    """
                    The ball hasn't been caught, and has left the zone; we can't possibly get it now
                    so we revert back to being idle in our zone
                    """
                    self._current_state = self.REVERTING_TO_IDLE_STATE
                    self._current_task = MoveToPoint(world, robot, role, idle_x, idle_y)
                    self._current_task.execute()

                elif our_attacker.has_ball(ball) and self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):
                    """
                    We have the ball in our grasp, proceed to next state and invoke the next task

                    Note: MoveWithBall is a placeholder, waiting for final set of tasks
                    """
                    # Note: choose_attacker_destination() currently just tries all possible
                    # points in the zone - inefficient. Could try probabilistic choice instead
                    # (Polygon.sample(rnd))

                    (x_dest, y_dest)= choose_attacker_destination(world)
                    self._current_state = self.MOVING_TO_POINT_STATE
                    self._current_task = MoveToPoint(world, robot, role, x_dest, y_dest)
                    self._current_task.execute()

            elif state == self.MOVING_TO_POINT_STATE:
                if abs(our_attacker.get_displacement_to_point(self._current_task.x, self._current_task.y)) > 30:
                    """
                    If the robot's displacement from the point we're trying to move to 
                    is't zero, we aren't there yet
                    """
                    self._current_task.execute()
                else:
                    self._current_state = self.SHOOTING_STATE
                    self._current_task = Shoot(world, robot, role)
                    self._current_task.execute()

            elif state == self.SHOOTING_STATE:
                if not self._current_task.kicked:
                    self._current_task.execute()
                else:
                    """
                    If we're in this state, we've already taken the shot (assuming the Shoot task fires the 
                    kicker as soon as it's execute() function is called), so revert back to the initial
                    starting state
                    """
                    self._current_state = self.REVERTING_TO_IDLE_STATE
                    self._current_task = MoveToPoint(world, robot, role, idle_x, idle_y)
                    self._current_task.execute()

        elif self._role=='defender':
            (idle_x, idle_y) = world.pitch.zones[our_defender.zone].center()

            """Defender state machine"""
            if state == self.INITIAL_STATE:
                if self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                    self._current_state = self.ACQUIRING_BALL_STATE
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

            elif state == self.REVERTING_TO_IDLE_STATE:

                if abs(our_defender.get_displacement_to_point(idle_x, idle_y)) > 20 and not(self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y)):
                    """
                    If the robot's displacement from the point we're trying to move to 
                    is't zero, we aren't there yet
                    """
                    self._current_task.execute()

                elif self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                    self._current_state=self.ACQUIRING_BALL_STATE
                    self._current_task=AcquireBall(world, robot, role)
                    self._current_task.execute()

                elif abs(our_defender.get_displacement_to_point(idle_x, idle_y)) < 20:
                    self._current_task=TurnToPoint(world, robot, role, idle_x*2, idle_y)
                    self._current_state=self.REVERTING_TO_IDLE_ROTATE
                    self._current_task.execute()

            elif state == self.REVERTING_TO_IDLE_ROTATE:

                if self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                    self._current_state=self.ACQUIRING_BALL_STATE
                    self._current_task=AcquireBall(world, robot, role)
                    self._current_task.execute()
                
                elif not self._current_task.complete:
                    self._current_task.execute()

                else:
                    self._current_task = None
                    self._current_state = self.INITIAL_STATE

            elif state == self.ACQUIRING_BALL_STATE:
                if not(our_defender.has_ball(ball)) and self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                    self._current_task = AcquireBall(world, robot, role)
                    self._current_task.execute()

                elif not(our_defender.has_ball(ball)) and not(self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y)):
                    """
                    The ball hasn't been caught, and has left the zone; we can't possibly get it now
                    so we revert back to being idle in our zone
                    """
                    self._current_state = self.REVERTING_TO_IDLE_STATE
                    self._current_task = MoveToPoint(world, robot, role, idle_x, idle_y)
                    self._current_task.execute()

                elif our_defender.has_ball(ball) and self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                    """
                    We have the ball in our grasp, proceed to next state and invoke the next task
                    """
                    # For milestone 2. Defender will move 2 widths' worth to the right before kicking away
                    if our_defender.x < world.pitch.zones[our_defender.zone].center()[0]:
                        (x_dest, y_dest) = (our_defender.x + our_defender.width, our_defender.y)
                        self._current_state = self.MOVING_TO_POINT_STATE
                        self._current_task = MoveToPoint(world, robot, role, x_dest, y_dest)
                        self._current_task.execute()
                    else:
                        self._current_state = self.CLEARING_STATE
                        zone_center = world.pitch.zones[our_attacker.zone].center()
                        self._current_task = KickToPoint(world, robot, role, zone_center[0], zone_center[1])
                        self._current_task.execute()

            elif state == self.MOVING_TO_POINT_STATE:
                if abs(our_defender.get_displacement_to_point(self._current_task.x, self._current_task.y)) > 20:
                    """
                    If the robot's displacement from the point we're trying to move to 
                    is't zero, we aren't there yet
                    """
                    self._current_task.execute()

                else:
                    self._current_state = self.CLEARING_STATE
                    zone_center = world.pitch.zones[our_attacker.zone].center()
                    self._current_task = KickToPoint(world, robot, role, zone_center[0], zone_center[1])
                    self._current_task.execute()

            elif state == self.CLEARING_STATE:
                if not self._current_task.kicked:
                    self._current_task.execute()
                else:
                    """
                    If we're in this state, we've already kicked the ball away (assuming the KickToPoint
                    task fires the kicker as soon as it's execute() function is called), so revert back to the 
                    initial starting state
                    """
                    self._current_state = self.REVERTING_TO_IDLE_STATE
                    self._current_task = MoveToPoint(world, robot, role, idle_x, idle_y)
                    self._current_task.execute()