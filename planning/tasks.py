from utilities import *
from action import Action
import time
import math

'''Task base class. Execute function must be implemented by inheriting
classes.'''
class Task(object):

    '''Initialise this task.'''
    def __init__(self, world, robot, role='attacker'):
        self.complete = False
        self.world = world
        self.robot = robot
        self.role = role
        if role == 'attacker':
            self.robot_info = self.world.our_attacker
        elif role == 'defender':
            self.robot_info = self.world.our_defender

    '''Carries out this task. Should call the functions of the Action
    object to carry out the actions required to complete this task.'''
    def execute(self):
        pass

class TurnToPoint(Task):
    '''Rotation Task; allows turning of the Robot to face a given point to
    within some threshold.
    '''

    def __init__(self,world,robot,role,x,y):
        super(TurnToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y
        self.turnHandle = None

    def check(self):
        return abs(self.robot_info.get_rotation_to_point(self.x,self.y)) < 0.1

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Otherwise, we're done.
        '''
        if self.complete or self.check():
            self.complete = True
            return

        if self.turnHandle == None:
            print '[TurnToPoint]: No TurnHandle assigned, generating new movement.'
            angle_to_turn = self.robot_info.get_rotation_to_point(self.x,self.y)


            print '[TurnToPoint]: New Angle To Turn: %f' % (angle_to_turn)
            self.turnHandle = self.robot.turnBy(angle_to_turn)

        else:
            if self.turnHandle.completed:
                print '[TurnToPoint]: TurnHandle completed.'

                if self.check():

                    print '[TurnToPoint]: Within tolerance of angle, complete is True.'
                    self.complete = True
                else:

                    print '[TurnToPoint]: Outwith tolerance, resetting TurnHandle.'
                    self.turnHandle = None

            elif self.turnHandle.finished:
                self.pitch_object = pitch_object
                self.move_task = None

    def should_move(self):
        return abs(self.pitch_object.y - self.robot_info.y) > 2

class MoveToPoint(Task):
    '''Movement Task. Travels to point(x,y) within some threshold.
    '''
    def __init__(self, world, robot, role, x, y, speed = 40, tolerance = 20, vtolerance = 2):
        super(MoveToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y
        self.speed = speed
        self.motionHandle = None
        self.DISP_TOLERANCE = tolerance
        self.VELOCITY_TOLERANCE = vtolerance
        self.TRAJECTORY_CONTROL = 0.1
        self.last_displacement = 0
        self.last_displacement_time = int(round(time.time() * 1000))
        self.displacement_update_time = 50
        self.displacement = self.robot_info.get_displacement_to_point(self.x,self.y)
        self.stopping = False

    
    def check(self):
        return (self.robot_info.get_displacement_to_point(self.x,self.y) < self.DISP_TOLERANCE)

    def angle_tolerance(self):
        if self.robot_info.get_displacement_to_point(self.x,self.y) > 100:
            return self.DISP_TOLERANCE
        else:
            return self.DISP_TOLERANCE + 20 / self.robot_info.get_displacement_to_point(self.x,self.y)
    
    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Otherwise, we're done.
        '''

        print '[MoveToPoint]: Entered execute()'

        if self.complete:
            print '[MoveToPoint]: Completed, returning.'
            return
        
        if int(round(time.time() * 1000)) - self.last_displacement_time > self.displacement_update_time:
            print '[MoveToPoint]: Updating last displacement.'
            self.last_displacement = self.displacement
            self.last_displacement_time = int(round(time.time() * 1000))
            self.displacement = self.robot_info.get_displacement_to_point(self.x,self.y)

        if not self.stopping:
            print '[MoveToPoint]: Not Stopping'
            if not self.check():
                print '[MoveToPoint]: Not within distance'
                if self.displacement - self.last_displacement > 0:
                    print '[MoveToPoint]: Moving away from target, resetting'
                    self.motionHandle = self.robot.stop()
                    self.stopping = True
            else:
                print '[MoveToPoint]: Within tolerance, stopping'
                self.motionHandle = self.robot.stop()
                self.stopping = True
        else:
            print '[MoveToPoint]: Stopping'
            if self.robot_info.velocity < self.VELOCITY_TOLERANCE:
                print '[MoveToPoint]: Velocity within stopping range'
                if not self.check():
                    print '[MoveToPoint]: Not within target, resetting'
                    self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
                    self.motionHandle = self.robot.move(self.angle, scale = self.speed)
                    self.stopping = False
                else:
                    print '[MoveToPoint]: Within target, complete'
                    self.complete = True

class AcquireBall(Task):
    '''Acquire Ball task. Rotate to face the ball, move to an appropriate
    distance from it and then attempt to catch it.'''

    def __init__(self,world,robot,role):
        super(AcquireBall,self).__init__(world,robot,role)
        self.move_task = None
        self.turn_task = None
        self.catch = None
        self.complete = False
        self.open_catch_handle = None

    def updateTurn(self):
        if self.turn_task == None:
            return
        elif abs(self.robot_info.get_rotation_to_point(self.world.ball.x, self.world.ball.y)) > math.pi/8 and self.turn_task.complete:
            self.turn_task = None

    def updateMove(self):
        scale = 30 / self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y)
        target_x = self.world.ball.x - (self.world.ball.x - self.robot_info.x)*scale
        target_y = self.world.ball.y - (self.world.ball.y - self.robot_info.y)*scale
        if self.move_task == None:
            return
        elif self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y) < 70 and self.move_task.speed == 60:
            self.move_task = None
        elif hypot(self.move_task.x - target_x, self.move_task.y - target_y) > 20:
            self.move_task = None

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Otherwise, if our catcher is closed, open it up.
        * Check if we're close enough to catch; if not, move forward.
        * Otherwise, attempt to catch the ball.
        '''

        if self.complete:
            print '[AcquireBall]: Task complete, returning.'
            return

        if self.robot_info.can_catch_ball(self.world.ball):
            print '[AcquireBall]: Can catch ball, stopping.'
            if self.catch == None or not self.catch.finished:
                print '[AcquireBall]: Catching Ball'
                self.catch = self.robot.catch()
                self.robot_info.catcher = 'closed'
                self.complete = True
            elif self.catch.completed:
                print '[AcquireBall]: Done!'
                self.complete = True
                self.robot_info.catcher = 'closed'
            self.robot.stop()
            return
        else:
            self.catch = None
            print '[AcquireBall]: Ball not in catcher area.'
            self.updateTurn()

            if self.turn_task == None:
                print '[AcquireBall]: Generating new turn to point task.'
                self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.world.ball.x, self.world.ball.y)
                return
            elif not self.turn_task.complete:
                print '[AcquireBall]: Executing TurnToPoint task.'
                self.turn_task.execute()
                return

            print '[AcquireBall]: Opening catcher, upating move.'
            self.updateMove()

            if self.move_task == None:
                
                if self.robot_info.catcher == 'closed':
                    self.robot.open_catcher()
                    self.robot_info.catcher = 'open'
                print '[AcquireBall]: Move task is None, generating new.'
                scale = 30 / self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y)
                target_x = self.world.ball.x - (self.world.ball.x - self.robot_info.x)*scale
                target_y = self.world.ball.y - (self.world.ball.y - self.robot_info.y)*scale
                if self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y) > 70:
                    speed = 40
                else:
                    speed = 30

                self.move_task = MoveToPoint(self.world, self.robot, self.role, target_x, target_y, speed = speed)

            elif not self.move_task.complete:
                print '[AcquireBall]: Executing move to point %f,%f.' % (self.move_task.x, self.move_task.y)
                self.move_task.execute()

            else:
                print 'Shouldn''t reach!'

class MirrorObject(Task):
    '''Mirrors the y-movement of a robot or the ball.
    '''

    def __init__(self,world,robot,role,obj):
        super(MirrorObject,self).__init__(world,robot,role)
        self.DISP_TOLERANCE = 30
        self.obj = obj
        self.angle = self.robot_info.get_rotation_to_point(self.robot_info.x, obj.y)
        self.motion_task = None
        last_speed = 0
        self.turn_task = None
    
    def check(self):
        return self.robot_info.get_displacement_to_point(self.robot_info.x, self.obj.y) < self.DISP_TOLERANCE
    
    def execute(self):
        '''Executes another round of this Task.
        '''

        if self.turn_task == None:
            self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.obj.x, self.robot_info.y)
        if not self.turn_task.complete:
            self.turn_task.execute()
        else:
            if self.motion_task == None:
                self.motion_task = MoveToPoint(self.world, self.robot, self.role, self.robot_info.x, self.obj.y, speed = self.calc_speed())
            if self.check():
                self.robot.stop()
                if abs(self.robot_info.get_rotation_to_point(self.obj.x, self.robot_info.y)) > math.pi/8:
                    print 'Turn task is none!'
                    self.turn_task = None
                return
            if math.hypot(0, self.obj.y - self.motion_task.y) > 10:
                self.motion_task = None
                return
            if abs(self.calc_speed() - self.motion_task.speed) > 10:
                self.motion_task = None
                return
            self.motion_task.execute()

    def calc_speed(self):
        speed = math.sin(self.obj.angle) * self.obj.velocity
        speed = min(70, max(60, speed))
        if abs(self.robot_info.y - self.obj.y) > 40:
            speed += 10
        return speed


class KickToPoint(Task):
    '''Generic kick to point task. Rotate to face the point (x,y) and kick.'''

    def __init__(self,world,robot,role,x,y):
        super(KickToPoint,self).__init__(world,robot,role)
        self.x = x
        self.y = y
        self.turn_task = None
        self.kickHandle = None
        self.complete = False
        self.robot.catch()

    def execute(self):
        '''Executes another round of this Task. Performs as follows:

        * Check if our angle is on target; if not, adjust by rotating.
        * Work through check states; checking angle is still on point, \
          opening catcher, waiting, then kicking.
        '''
        if self.complete:
            return

        if self.turn_task is None:
            print '[KickToPoint]: Construct new TurnToPoint'
            self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.x, self.y)

        if not self.turn_task.complete:
            print '[KickToPoint]: Executing TurnToPoint'
            self.turn_task.execute()
            return
        
        if self.kickHandle == None:
            print '[KickToPoint]: Opening catcher and kicking'
            self.robot.open_catcher()
            self.kickHandle = self.robot.kick()

        if self.kickHandle != None and not self.kickHandle.completed:
            print '[KickToPoint]: Kicking not completed, passing'
            pass
        else:
            print '[KickToPoint]: Kicking completed.'
            self.complete = True

class Shoot(KickToPoint):
    '''Simple Shoot task. Rotates the robot to face the goal and then kicks.'''
    
    def __init__(self,world,robot,role):
        x = world.their_goal.x
        y = world.their_goal.y
        super(Shoot,self).__init__(world,robot,role,x,y)





