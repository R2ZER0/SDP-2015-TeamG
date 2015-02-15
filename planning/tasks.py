from utilities import *
import math
from random import randint
from action import Action
import time

'''Task base class. Execute function must be implemented by inheriting
classes.'''
class Task(object):

    ATTACKER='attacker'
    DEFENDER='defender'

    '''Initialise this task.'''
    def __init__(self, world, robot, role):
        self.world = world
        self.robot = robot
        self.role = role
        if self.role == Task.ATTACKER:
            self.robot_info = self.world.our_attacker
        elif self.role == Task.DEFENDER:
            self.robot_info = self.world.our_defender

    '''Carries out this task. Should call the functions of the Action
    object to carry out the actions required to complete this task.'''
    def execute(self):
        pass

class AcquireBall(Task):
    '''Acquire Ball task. Rotate to face the ball, move to an appropriate
    distance from it and then attempt to catch it.'''

    #: Largest angle (0 - MAX_ANGLE_THRESHOLD) between the robot and the ball \
    #: before we need to turn to face the robot.
    ANGLE_THRESHOLD = 0.3

    #: Distance the robot must be from the ball in order to catch it
    CATCH_DISTANCE = 40

    #: Becomes True once this Task has caught the blal
    complete = False

    #: True indicates we were still turning on the last call
    turning = False

    def __init__(self,world,robot,role):
        super(AcquireBall,self).__init__(world,robot,role)

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Otherwise, if our catcher is closed, open it up.
        * Check if we're close enough to catch; if not, move forward.
        * Otherwise, attempt to catch the ball.
        '''

        if self.complete:
            return

        ball_x = self.world.ball.x
        ball_y = self.world.ball.y

        #Work out the angle between the robot's orientation and the ball.
        angle_to_turn = self.robot_info.get_rotation_to_point(ball_x,ball_y)
        
        # If the robot is not facing the point...
        if abs(angle_to_turn) > self.ANGLE_THRESHOLD:

            ''' This function maps the angle between the robot and the ball
            to a curve using the function log(x*10 + 1) / log(pi*10 + 1).
            The value is normalised to the range [60,100] because the motors
            don't function properly below ~60% power. Plot the curve on
            wolfram alpha to see what it looks like.'''
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60

            self.turning = True

            #Turn left or right.
            if angle_to_turn > 0:                
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)

        #Otherwise, move toward the ball.
        else:
            if self.turning:
                self.robot.stop()
                self.turning = False

            #Open the catcher.
            if self.robot_info.catcher == 'closed':
                self.robot.open_catcher()
                self.robot.open_catcher()
                self.robot_info.catcher = 'open'
                return

            #Find the distance to the ball.
            displacement = self.robot_info.get_displacement_to_point(ball_x,ball_y)

            #If we are too far away, move closer.
            if displacement > self.CATCH_DISTANCE:
                move_speed = 100
                self.robot.move(0,move_speed)

            #Otherwise, attempt to catch the ball.
            else:
                self.robot.stop()
                if self.robot_info.catcher == 'open':
                    self.robot.catch()
                    self.robot.catch()
                    self.robot_info.catcher = 'closed'
                    self.complete = True

class MoveToPoint(Task):
    '''Movement Task. Rotates our Robot to face the point (x,y) and travels
    to it within some threshold.
    '''

    #: Largest angle threshold that we allow
    MAX_ANGLE_THRESHOLD = 0.5

    #: Lower angle threshold as we get closer to the ball
    MIN_ANGLE_THRESHOLD = 0.5

    #: Distance threshold to the point before considering ourselves done
    DISP_TOLERANCE = 45

    #: Specified x-position to travel to
    x = 0
    
    #: Specified y-position to travel to
    y = 0

    #: Assigned True once we reach the point within the threshold
    complete = False

    def __init__(self,world,robot,role,x,y):
        '''Initialises this task. (x,y) represent the point on the pitch to
        move to.'''
        super(MoveToPoint,self).__init__(world,robot,role)

        self.x = x
        self.y = y

    def execute(self, custom_threshold=None, custom_rotation=None, custom_speed=None):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Check if we're close enough; if not, move forward.
        * Otherwise, we're done.
        '''
        # Find the distance to the point x,y.
        displacement = self.robot_info.get_displacement_to_point(self.x,self.y)

        # Angle threshold decreases as the robot nears the point (x,y).
        angle_threshold = min(1,(displacement / 100)) * (self.MAX_ANGLE_THRESHOLD - self.MIN_ANGLE_THRESHOLD) + self.MIN_ANGLE_THRESHOLD

        # Find out the angle between (x,y) and the robot's orientation.
        angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)
        
        # If the robot is not facing the point...
        if abs(angle_to_turn) > angle_threshold:

            # Rotation speed decreases as the angle required decreases.
            #rotation_speed = 40 * math.log10(abs(angle_to_turn))
            rotation_speed = 40
            print angle_to_turn
            #Turn left or right.
            if angle_to_turn > 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)

        # If the robot is facing the point...
        else:
            # If the robot is too far away...
            if displacement > self.DISP_TOLERANCE:
                move_speed = 100
                self.robot.move(0,move_speed)
            # Else the robot has completed the task.
            else:
                self.robot.stop()

class TurnToPoint(Task):
    '''Rotation Task; allows turning of the Robot to face a given point to
    within some threshold.
    '''

    #: Margin of error allowed in this angle, radians.
    ANGLE_THRESHOLD = 0.5
    
    #: Our target x-position
    x = 0
    
    #: Our target y-position
    y = 0
    
    #: Gets assigned True once the Robot has rotated to the angle within accepted range.
    complete = False

    def __init__(self,world,robot,role,x,y):
        super(TurnToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Otherwise, we're done.
        '''

        if self.complete:
            return

            # Find the angle between the robot's orientation and the point.
        angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)
        
        # If the robot is not facing the point...
        if abs(angle_to_turn) > self.ANGLE_THRESHOLD:

            # Rotation speed decreases as the angle required decreases.
            #rotation_speed = 40 * math.log10(abs(angle_to_turn))
            rotation_speed = 40
            print angle_to_turn
            #Turn left or right.
            if angle_to_turn > 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)

        else:
            self.complete = True
            self.robot.stop()

class KickToPoint(Task):
    '''Generic kick to point task. Rotate to face the point (x,y) and kick.'''

    #: Allowed margin of error between the desired angle and ours.
    ANGLE_THRESHOLD = 0.2
    
    #: The target x-point to kick to.
    x = 0
    
    #: The target y-point to kick to.
    y = 0

    #: Assigned True when the Task has kicked the ball
    kicked = False

    #: This increases through various values as we perform checks to ensure \
    #: we're on target to kick accurately.
    check_state = 0
    
    def __init__(self,world,robot,role,x,y):
        super(KickToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Work through check states; checking angle is still on point, \
          opening catcher, waiting, then kicking.
        '''

        if self.kicked:
            return

        # Find the angle between the robot's orientation and the point.
        angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)

        # If the robot is not facing the point...
        if abs(angle_to_turn) > self.ANGLE_THRESHOLD:

            # Rotation speed decreases as the angle required decreases.
            #rotation_speed = 40 * math.log10(abs(angle_to_turn))
            rotation_speed = 40
            #Turn left or right.
            if angle_to_turn > 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)

        #Else the robot is facing the point...
        else:
            if self.check_state <= 1:
                self.robot.stop()
                self.check_state += 1
                return
            elif self.check_state == 2:
                if abs(angle_to_turn) <= self.ANGLE_THRESHOLD:
                    self.check_state += 1
                else:
                    self.check_state = 0
                return
            else:
                self.robot.kick()
                self.kicked = True

class Shoot(KickToPoint):
    '''Simple Shoot task. Rotates the robot to face the goal and then kicks.'''
    
    def __init__(self,world,robot,role):
        x = world.their_goal.x
        y = world.their_goal.y
        super(Shoot,self).__init__(world,robot,role,x,y)
