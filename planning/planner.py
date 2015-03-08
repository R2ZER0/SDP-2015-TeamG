import pdb
from models import *
from collisions import *
from tasks import *
from utilities import *

class Planner:
<<<<<<< HEAD

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

=======
    
    # The planner assigns the robots a state and strategy
    #
    # Each robot (attacker, defender) has a state:
    # attacker: defence, grab, score, catch
    # defender: defence, grab, pass
    #
    # Based on these they choose one of a number of strategies from strategies.py
    # Decided in the (large) plan() method
    #
    # NOTE: Strategies have their own (sub-)state
    # This is how far through the strategy they are
    # e.g. when grabing ball, we must -prepare-, -go to the ball-, -grab- it, and be -finished-

    def __init__(self, our_side, pitch_num, task, is_obstacle):
        self._world = World(our_side, pitch_num)
        # LB: Magic numbers!
        # These should surely be constants in models.py?
        # Also need to make sure grabber area is consistent with our robot
        self._world.our_defender.catcher_area = {'width' : 25, 'height' : 18, 'front_offset' : 12} #10
        self._world.our_attacker.catcher_area = {'width' : 25, 'height' : 18, 'front_offset' : 14}

        self._attacker_strategies = { 'milestone3catch_obstacle' : [Milestone3CatchObstacle],
                                      'milestone3catch_no_obstacle' : [Milestone3CatchNoObstacle],
                                      'milestone3kick_obstacle' : [Milestone3KickObstacle],
                                      'milestone3kick_no_obstacle' : [Milestone3KickNoObstacle]}

        self._defender_strategies = { 'milestone3catch_obstacle' : [Milestone3CatchObstacle],
                                      'milestone3catch_no_obstacle' : [Milestone3CatchNoObstacle],
                                      'milestone3kick_obstacle' : [Milestone3KickObstacle],
                                      'milestone3kick_no_obstacle' : [Milestone3KickNoObstacle]}

        # self._attacker_strategies = {'defence' : [AttackerDefend],
        #                              'grab' : [AttackerGrab, AttackerGrabCareful],
        #                              'score' : [AttackerDriveByTurn, AttackerDriveBy, AttackerTurnScore, AttackerScoreDynamic],
        #                              'catch' : [AttackerPositionCatch, AttackerCatch]}

        # self._defender_strategies = {'defence' : [DefenderDefence, DefenderPenalty],
        #                              'grab' : [DefenderGrab],
        #                              'pass' : [DefenderBouncePass]}

        assert task in ['kick', 'catch']
        self.task = task
        self.is_obstacle = is_obstacle

        if task == 'kick':
            if is_obstacle:
                self._defender_state = 'milestone3kick_obstacle'
                self._attacker_state = 'milestone3kick_obstacle'
            else:
                self._defender_state = 'milestone3kick_no_obstacle'
                self._attacker_state = 'milestone3kick_no_obstacle'
        else:
            if is_obstacle:
                self._defender_state = 'milestone3catch_obstacle'
                self._attacker_state = 'milestone3catch_obstacle'
            else:
                self._defender_state = 'milestone3catch_no_obstacle'
                self._attacker_state = 'milestone3catch_no_obstacle'
            
        self._defender_current_strategy = self.choose_defender_strategy(self._world)
        self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

    # LB: Only chooses the first possible strategy? Is this correct?
    # Provisional. Choose the first strategy in the applicable list.
    def choose_attacker_strategy(self, world):
        next_strategy = self._attacker_strategies[self._attacker_state][0]
        return next_strategy(world)

    # Provisional. Choose the first strategy in the applicable list.
    def choose_defender_strategy(self, world):
        next_strategy = self._defender_strategies[self._defender_state][0]
        return next_strategy(world)

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
        # LB: assertion looks strange - state is set to things like "grab" at some points - check this
        assert new_state in ['defence', 'attack', 'milestone3catch_obstacle', 'milestone3catch_no_obstacle', 
            'milestone3kick_obstacle', 'milestone3kick_no_obstacle']
        self._attacker_state = new_state

    @property
    def defender_state(self):
        return self._defender_state

    @defender_state.setter
    def defender_state(self, new_state):
        # LB: assertion looks strange - state is set to things like "grab" at some points - check this
        assert new_state in ['defence', 'attack', 'milestone3catch_obstacle', 'milestone3catch_no_obstacle', 
            'milestone3kick_obstacle', 'milestone3kick_no_obstacle']
        self._defender_state = new_state

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    # LB: We could split up this big method
    # I also don't think robot should have a default - to avoid forgetting the parameter
    # But we need to check if it is used anywhere without the parameter
    def plan(self, robot='attacker'):
        assert robot in ['attacker', 'defender']
        assert self.task in ['kick', 'catch']
        
        if self.task == 'kick':
            if self.is_obstacle:
                state = 'milestone3kick_obstacle'
                strategy = Milestone3KickObstacle
            else:
                state = 'milestone3kick_no_obstacle'
                strategy = Milestone3KickNoObstacle
        else:
            if self.is_obstacle:
                state = 'milestone3catch_obstacle'
                strategy = Milestone3CatchObstacle
            else:
                state = 'milestone3catch_no_obstacle'
                strategy = Milestone3CatchNoObstacle

        if robot == 'defender':
            if not self._defender_state == state:
                self._defender_state = state
            if not isinstance(self._defender_current_strategy, strategy):
                self._defender_current_strategy = self.choose_defender_strategy(self._world)
            return self._defender_current_strategy.generate()

        if robot == 'attacker':
            if not self._attacker_state == state:
                self._attacker_state = state
            if not isinstance(self._attacker_current_strategy, strategy):
                self._attacker_current_strategy = self.choose_attacker_strategy(self._world)
            return self._attacker_current_strategy.generate()



        # Old Stuff
        our_defender = self._world.our_defender
>>>>>>> tbm-theirs
        our_attacker = self._world.our_attacker
        our_defender = self._world.our_defender
        their_attacker = self._world.their_attacker
        their_defender = self._world.their_defender

<<<<<<< HEAD
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
=======
        else:
            # If the ball is in their defender zone we defend:
            if self._world.pitch.zones[their_defender.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'defence':
                    self._attacker_state = 'defence'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)
                return self._attacker_current_strategy.generate()

            # If ball is in our attacker zone, then grab the ball and score:
            elif self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):

                # Check if we should switch from a grabbing to a scoring strategy.
                if self._attacker_state == 'grab' and self._attacker_current_strategy.current_state == 'GRABBED':
                    self._attacker_state = 'score'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                elif self._attacker_state == 'grab':
                    # Switch to careful mode if the ball is too close to the wall.
                    if abs(self._world.ball.y - self._world.pitch.height) < 0 or abs(self._world.ball.y) < 0:
                        if isinstance(self._attacker_current_strategy, Milestone2Attacker):
                            self._attacker_current_strategy = Milestone2AttackerCareful(self._world)
                    else:
                        if isinstance(self._attacker_current_strategy, Milestone2AttackerCareful):
                            self._attacker_current_strategy = Milestone2Attacker(self._world)

                # Check if we should switch from a defence to a grabbing strategy.
                elif self._attacker_state in ['defence', 'catch'] :
                    self._attacker_state = 'grab'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                elif self._attacker_state == 'score' and self._attacker_current_strategy.current_state == 'FINISHED':
                    self._attacker_state = 'grab'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

                return self._attacker_current_strategy.generate()
            # If the ball is in our defender zone, prepare to catch the passed ball:
            elif self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y) or \
                 self._attacker_state == 'catch':
                 # self._world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'catch':
                    self._attacker_state = 'catch'
                    self._attacker_current_strategy = self.choose_attacker_strategy(self._world)
                return self._attacker_current_strategy.generate()
>>>>>>> tbm-theirs
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
                

