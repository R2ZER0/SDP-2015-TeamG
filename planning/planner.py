from models import *
from collisions import *
from tasks import *
from utilities import *

# Note: Needs slight modification to work with final task set
class Planner:

    def __init__(self, our_side, pitch_num, world):
        #self._world = World(our_side, pitch_num)
        #self._world.our_defender.catcher_area = {'width' : 30, 'height' : 30, 'front_offset' : 12}
        #self._world.our_attacker.catcher_area = {'width' : 30, 'height' : 30, 'front_offset' : 14}

        # self._defender_defence_task = DefenderDefence(self._world)
        # self._defender_attack_task = DefaultDefenderAttack(self._world)

        self._attacker_tasks = {'defence' : [AttackerDefend],
                                     'grab' : [AttackerGrab, AttackerGrabCareful],
                                     'score' : [AttackerDriveByTurn, AttackerDriveBy, AttackerTurnScore, AttackerScoreDynamic],
                                     'catch' : [AttackerPositionCatch, AttackerCatch]}

        self._defender_tasks = {'defence' : [DefenderDefence, DefenderPenalty],
                                     'grab' : [DefenderGrab],
                                     'pass' : [DefenderBouncePass]}

        self._defender_state = 'defence'
        self._defender_current_task = self.choose_defender_task(world)

        self._attacker_state = 'defence'
        self._attacker_current_task = self.choose_attacker_task(world)

    # Provisional. Choose the first task in the applicable list.
    def choose_attacker_task(self, world):
        next_task = self._attacker_tasks[self._attacker_state][0]
        return next_task(world)

    # Provisional. Choose the first task in the applicable list.
    def choose_defender_task(self, world):
        next_task = self._defender_tasks[self._defender_state][0]
        return next_task(world)

    @property
    def attacker_task_state(self):
        return self._attacker_current_task.current_state

    @property
    def defender_task_state(self):
        return self._defender_current_task.current_state

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

    #def update_world(self, position_dictionary):
    #    self._world.update_positions(position_dictionary)

    def plan(self, world, robot='attacker'):
        assert robot in ['attacker', 'defender']
        our_defender = world.our_defender
        our_attacker = world.our_attacker
        their_defender = world.their_defender
        their_attacker = world.their_attacker
        ball = world.ball
        if robot == 'defender':
            # If the ball is in their attacker zone:
            if world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
                # If the bal is not in the defender's zone, the state should always be 'defend'.
                if not self._defender_state == 'defence':
                    self._defender_state = 'defence'
                    self._defender_current_task = self.choose_defender_task(world)
                return self._defender_current_task.generate()

            # We have the ball in our zone, so we grab and pass:
            elif world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
                # Check if we should switch from a grabbing to a scoring task.
                if  self._defender_state == 'grab' and self._defender_current_task.current_state == 'GRABBED':
                    self._defender_state = 'pass'
                    self._defender_current_task = self.choose_defender_task(world)

                # Check if we should switch from a defence to a grabbing task.
                elif self._defender_state == 'defence':
                    self._defender_state = 'grab'
                    self._defender_current_task = self.choose_defender_task(world)

                elif self._defender_state == 'pass' and self._defender_current_task.current_state == 'FINISHED':
                    self._defender_state = 'grab'
                    self._defender_current_task = self.choose_defender_task(world)

                return self._defender_current_task.generate()
            # Otherwise, chillax:
            else:

                return do_nothing()

        else:
            # If the ball is in their defender zone we defend:
            if world.pitch.zones[their_defender.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'defence':
                    self._attacker_state = 'defence'
                    self._attacker_current_task = self.choose_attacker_task(world)
                return self._attacker_current_task.generate()

            # If ball is in our attacker zone, then grab the ball and score:
            elif world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y):

                # Check if we should switch from a grabbing to a scoring task.
                if self._attacker_state == 'grab' and self._attacker_current_task.current_state == 'GRABBED':
                    self._attacker_state = 'score'
                    self._attacker_current_task = self.choose_attacker_task(world)

                elif self._attacker_state == 'grab':
                    # Switch to careful mode if the ball is too close to the wall.
                    if abs(world.ball.y - world.pitch.height) < 0 or abs(world.ball.y) < 0:
                        if isinstance(self._attacker_current_task, AttackerGrab):
                            self._attacker_current_task = AttackerGrabCareful(world)
                    else:
                        if isinstance(self._attacker_current_task, AttackerGrabCareful):
                            self._attacker_current_task = AttackerGrab(world)

                # Check if we should switch from a defence to a grabbing task.
                elif self._attacker_state in ['defence', 'catch'] :
                    self._attacker_state = 'grab'
                    self._attacker_current_task = self.choose_attacker_task(world)

                elif self._attacker_state == 'score' and self._attacker_current_task.current_state == 'FINISHED':
                    self._attacker_state = 'grab'
                    self._attacker_current_task = self.choose_attacker_task(world)

                return self._attacker_current_task.generate()
            # If the ball is in our defender zone, prepare to catch the passed ball:
            elif world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y) or \
                 self._attacker_state == 'catch':
                 # world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
                if not self._attacker_state == 'catch':
                    self._attacker_state = 'catch'
                    self._attacker_current_task = self.choose_attacker_task(world)
                return self._attacker_current_task.generate()
            else:
                return calculate_motor_speed(0, 0)
