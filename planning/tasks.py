from utilities import *
import math
from random import randint
from action import Action
import pdb

'''Task base class. Execute function must be implemented by inheriting
classes.'''
class Task(object):

    ATTACKER='attacker'
    DEFENDER='defender'

    #world      = None #The world state.
    #robot      = None #The Action object that represents the robot.
    #role       = None #The role (attacker/defender) of the robot.
    #robot_info = None #The robot info from the world state.

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

'''Acquire Ball task. Rotate to face the ball, move to an appropriate
distance from it and then attempt to catch it.'''
class AcquireBall(Task):

    '''The largest angle (0 - math.pi) between the robot and the ball
    before we need to turn to face the robot.'''
    ANGLE_THRESHOLD = 0.3

    '''The distance the robot must be from the ball in order to catch
    it.'''
    CATCH_DISTANCE = 30

    def __init__(self,world,robot,role):
        super(AcquireBall,self).__init__(world,robot,role)

        self.turning = False
        self.complete = False

    def execute(self):

        if self.complete:
            return

        ball_x = self.world.ball.x
        ball_y = self.world.ball.y

        #Work out the angle between the robot's orientation and the ball.
        angle_to_turn = self.robot_info.get_rotation_to_point(ball_x,ball_y)
        
        #If the angle is too large, rotate to face the ball.
        if abs(angle_to_turn) > self.ANGLE_THRESHOLD:
            ''' This function maps the angle between the robot and the ball
            to a curve using the function log(x*10 + 1) / log(pi*10 + 1).
            The value is normalised to the range [60,100] because the motors
            don't function properly below ~60% power. Plot the curve on
            wolfram alpha to see what it looks like.'''
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60
            self.turning = True

            #Turn left or right.
            if angle_to_turn < 0:
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
                self.robot_info.catcher = 'open'

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
                    self.robot_info.catcher = 'closed'
                    self.complete = True

'''Movement task. Rotate to face the point (x,y) and go to it.'''
class MoveToPoint(Task):

    '''The maximum and minimum values of the angle between the robot's
    orientation and the point. Threshold goes from MAX->MIN as
    displacement decreases.'''
    MIN_ANGLE_THRESHOLD = 0.05
    MAX_ANGLE_THRESHOLD = 0.3

    '''How close the robot must be to the point before we stop moving.'''
    DISP_TOLERANCE = 20

    '''The x value of the point.'''
    x = 0
    '''The y value of the point.'''
    y = 0

    '''Initialises this task. (x,y) represent the point on the pitch to
    move to.'''
    def __init__(self,world,robot,role,x,y):
        super(MoveToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y

    def execute(self):
        # Find the distance to the point x,y.
        displacement = self.robot_info.get_displacement_to_point(self.x,self.y)

        # Angle threshold decreases as the robot nears the point (x,y).
        angle_threshold = min(1,(displacement / 100)) * (self.MAX_ANGLE_THRESHOLD - self.MIN_ANGLE_THRESHOLD) + self.MIN_ANGLE_THRESHOLD

        # Find out the angle between (x,y) and the robot's orientation.
        angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)

        # If the robot is not facing the point...
        if angle_to_turn > angle_threshold:
            # Rotation speed decreases as the angle required decreases.
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60
            # Turn left or right.
            if angle_to_turn < 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)
        # If the robot is facing the point...
        else:
            # If the robot is too far away...
            if displacement > self.DISP_TOLERANCE:
                move_speed = 1
                self.robot.move(0,move_speed)
            # Else the robot has completed the task.
            else:
                self.robot.stop()

'''Generic kick to point task. Rotate to face the point (x,y) and kick.'''
class KickToPoint(Task):

    '''The largest angle (0 - math.pi) between the robot and the ball
    before we need to turn to face the robot.'''
    ANGLE_THRESHOLD = 0.2
    
    '''The x value of the point.'''
    x = 0
    '''The y value of the point.'''
    y = 0

    kicked = False
    
    def __init__(self,world,robot,role,x,y):
        super(KickToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y

    def execute(self):

        if self.kicked:
            return

        # Find the angle between the robot's orientation and the point.
        angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)

        # If the robot is not facing the point...
        if abs(angle_to_turn) > self.ANGLE_THRESHOLD:
            # Rotation speed decreases as the angle required decreases.
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60
            # Turn left or right.
            if angle_to_turn < 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)
        #Else the robot is facing the point...
        else:
            self.robot.kick()
            self.kicked = True

'''Shoot task. Rotate robot to face the goal and then kick.'''
class Shoot(KickToPoint):
    
    def __init__(self,world,robot,role):
        x = world.their_goal.x
        y = world.their_goal.y
        super(Shoot,self).__init__(world,robot,role,x,y)
