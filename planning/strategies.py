from utilities import *
import math
from random import randint

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
        super(AcquireBall,self)

    def execute(self):
        ball_x = self.world.ball.x()
        ball_y = self.world.ball.y()

        if role == ATTACKER:
            robot_x = self.world.our_attacker.x()
            robot_y = self.world.our_attacker.y()
            robot_angle = self.world.our_defender.angle()
        elif role == DEFENDER:
            robot_x = self.world.our_defender.x()
            robot_y = self.world.our_defender.y()
            robot_angle = self.world.our_defender.angle()

        dist_x = robot_x - ball_x
        dist_y = robot_y - ball_y

        theta = atan2(dist_x,dist_y)
        
        angle_to_turn = theta - robot_angle

        if abs(angle_to_turn) > 10: 
            if angle_to_turn < 0:
                robot.turn(-100)
            else:
                robot.turn(100)
        else:
            robot.stop()
