from utilities import *
import math
from random import randint
from action import Action
import pdb

class Task(object):

    ATTACKER='attacker'
    DEFENDER='defender'

    self.world      = None #The world state.
    self.robot      = None #The Action object that represents the robot.
    self.role       = None #The role (attacker/defender) of the robot.
    self.robot_info = None #The robot info from the world state.

    def __init__(self, world, robot, role):
        self.world = world
        self.robot = robot
        self.role = role
        if self.role == Task.ATTACKER:
            self.robot_info = self.world.our_attacker
        elif self.role == Task.DEFENDER:
            self.robot_info = self.world.our_defender

    def execute(self):
        pass

class AcquireBall(Task):

    '''The largest angle (0 - math.pi) between the robot and the ball
    before we need to turn to face the robot.'''
    ANGLE_THRESHOLD = 0.3

    '''The distance the robot must be from the ball in order to catch
    it.'''
    CATCH_DISTANCE = 35

    def __init__(self,world,robot,role):
        super(AcquireBall,self).__init__(world,robot,role)

    def execute(self):
        ball_x = self.world.ball.x
        ball_y = self.world.ball.y

        #Work out the angle between the robot's orientation and the ball.
        angle_to_turn = self.robot_info.get_rotation_to_point(ball_x,ball_y)
        
        #If the angle is too large, rotate to face the ball.
        if abs(angle_to_turn) > ANGLE_THRESHOLD:
            ''' This function maps the angle between the robot and the ball
            to a curve using the function log(x*10 + 1) / log(pi*10 + 1).
            The value is normalised to the range [60,100] because the motors
            don't function properly below ~60% power. Plot the curve on
            wolfram alpha to see what it looks like.'''
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60
            #Turn left or right.
            if angle_to_turn < 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)
        #Otherwise, move toward the ball.
        else:
            #Open the catcher.
            if self.robot_info.catcher == 'closed':
                self.robot.open_catcher()
                self.robot_info.catcher = 'open'
            #Find the distance to the ball.
            displacement = self.robot_info.get_displacement_to_point(ball_x,ball_y)
            #If we are too far away, move closer.
            if displacement > CATCH_DISTANCE:
                move_speed = 1
                self.robot.move(0,move_speed)
            #Otherwise, attempt to catch the ball.
            else:
                self.robot.stop()
                if self.robot_info.catcher == 'open':
                    self.robot.catch()
                    self.robot_info.catcher = 'closed'


