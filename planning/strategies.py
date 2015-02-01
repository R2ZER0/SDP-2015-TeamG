from utilities import *
import math
from random import randint
from action import Action
import pdb

class Strategy(object):

    ATTACKER='attacker'
    DEFENDER='defender'
    
    def __init__(self, world, robot, role):
        self.world = world
        self.robot = robot
        self.role = role

    def generate(self):
        return self.NEXT_ACTION_MAP[self.current_state]()

    def execute(self):
        pass

class AcquireBall(Strategy):

    def __init__(self,world,robot,role):
        super(AcquireBall,self).__init__(world,robot,role)
        #self.turning = False
        if self.role == Strategy.ATTACKER:
            self.robot_info = self.world.our_attacker
        elif self.role == Strategy.DEFENDER:
            self.robot_info = self.world.our_defender

    def execute(self):
        ball_x = self.world.ball.x
        ball_y = self.world.ball.y

        angle_to_turn = self.robot_info.get_rotation_to_point(ball_x,ball_y)
        
        if abs(angle_to_turn) > 0.3: # If the angle between the robot and the ball is above the arbitrary threshold
            ''' This function maps the angle between the robot and the ball
            to a curve using the function log(x*10 + 1) / log(pi*10 + 1).
            The value is normalised to the range [60,100] because the motors
            don't function properly below ~60% power. Plot the curve on
            wolfram alpha to see what it looks like.'''
            rotation_speed = (math.log10((abs(angle_to_turn) * 10) + 1) / math.log10((math.pi * 10) + 1)) * 40 + 60
            #self.turning = True
            if angle_to_turn < 0:
                self.robot.turn(rotation_speed)
            else:
                self.robot.turn(-rotation_speed)
        else:
            self.robot.kick()
            displacement = self.robot_info.get_displacement_to_point(ball_x,ball_y)
            if displacement > 35:
                move_speed = 1
                self.robot.move(0,move_speed)
            else:
                self.robot.stop()
                self.robot.catch()
            
