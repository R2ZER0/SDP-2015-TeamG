import math


class Task(object):
    """Task base class. Execute function must be implemented by inheriting
    classes."""

    ATTACKER = 'attacker'
    DEFENDER = 'defender'

    #: Default value of motor speed to start at
    BASE_MOTOR_SPEED = 70

    #: Gets assigned True once the Robot has rotated to the angle within accepted range.
    complete = False

    def __init__(self, world, robot, role):
        """Initialise this task."""
        self.world = world
        self.robot = robot
        self.role = role
        if self.role == Task.ATTACKER:
            self.robot_info = self.world.our_attacker
        elif self.role == Task.DEFENDER:
            self.robot_info = self.world.our_defender

    def angle_to_point(self, x, y):
        """Get the angle between the robot and a particular point."""
        return -self.robot_info.get_rotation_to_point(x, y)

    def displacement_to_point(self, x, y):
        return self.robot_info.get_displacement_to_point(x, y)

    def execute(self):
        """Carries out this task. Should call the functions of the Action
        object to carry out the actions required to complete this task."""
        pass


class MoveToPoint(Task):
    """Movement Task. Rotates our Robot to face the point (x,y) and travels
    to it within some threshold.
    """

    #: Distance threshold to the point before considering ourselves done
    DISPLACEMENT_TOLERANCE = 0

    #: Specified x-position to travel to
    x = 0

    #: Specified y-position to travel to
    y = 0

    #: The current angle to move.
    angle = 0

    #: Tracks the last speeds we sent to the Robot to check for sending more values
    motionHandle = None

    def __init__(self, world, robot, role, x, y, tolerance=30):
        Task.__init__(self, world, robot, role)
        self.x = x
        self.y = y
        self.DISPLACEMENT_TOLERANCE = tolerance
	self.displacement = 0

    def move(self):
        """Updates this Task's motionHandle to move toward a new point."""
        if self.check_displacement():
            self.robot.stop()
            return True
        else:
            if self.motionHandle is None:
		self.angle = self.angle_to_point(self.x, self.y)
		self.angle = int(self.angle / (math.pi / 4)) * math.pi / 4  # round to nearest 45 degrees
		self.motionHandle = self.robot.move(self.angle, scale=self.get_movement_speed())
            else:
                if self.motionHandle.completed or self.motionHandle.finished:
			if self.displacement < self.get_displacement():
			    self.motionHandle = None
			else:
			    self.displacement = self.get_displacement()
            	return False

    def check_displacement(self):
        """Check if the robot is within DISPLACEMENT_TOLERANCE units of self.x, self.y"""
        return self.get_displacement() <= self.DISPLACEMENT_TOLERANCE

    def get_displacement(self):
	print self.displacement_to_point(self.x, self.y)
        return self.displacement_to_point(self.x, self.y)

    def get_movement_speed(self):
        return min(self.BASE_MOTOR_SPEED, max(self.get_displacement(), 35))

    def execute(self):
        if self.complete:
            return

        if self.move():
            self.complete = True


class TurnToPoint(Task):

    ROTATION_TOLERANCE = 0.2

    x = 0
    y = 0

    motionHandle = None

    def __init__(self, world, robot, role, x, y, tolerance=0.1):
        super(TurnToPoint, self).__init__(world, robot, role)
        self.x = x
        self.y = y
        self.ROTATION_TOLERANCE = tolerance

    def turn(self):
        """Updates this Task's motionHandle to move toward a new point."""
	if self.check_rotation() and self.motionHandle is not None and self.motionHandle.completed:
            return True
        else:
            if self.motionHandle is None:
                rotation = self.get_rotation()
                self.motionHandle = self.robot.turnBy(rotation)
            else:
                if (self.motionHandle.completed and not self.check_rotation()) or self.motionHandle.finished:
                    self.motionHandle = None
            return False

    def update(self, x, y):
        self.x = x
        self.y = y
        self.complete = False
        
    def check_rotation(self):
        """Check if the robot is closer than ROTATION_TOLERANCE radians to self.x, self.y"""
        return abs(self.get_rotation()) <= self.ROTATION_TOLERANCE

    def get_rotation(self):
        return self.angle_to_point(self.x, self.y)

    def execute(self):
        if self.complete:
            self.robot.stop()
            return
        
        if self.turn():
            self.complete = True

class TurnToObject(TurnToPoint):

    pitch_object = None
    
    def __init__(self, world, robot, role, pitch_object):
        self.pitch_object = pitch_object
        super(TurnToObject,self).__init__(world, robot, role, self.pitch_object.x, self.pitch_object.y)

    def update(self):
        self.x = self.pitch_object.x
        self.y = self.pitch_object.y
        self.complete = False

    def turn(self):
        if (self.pitch_object.x != self.x or self.pitch_object.y != self.y):
            self.update()

        return TurnToPoint.turn(self)

class MirrorObject(MoveToPoint, TurnToPoint):

    def __init__(self, world, robot, role, pitch_object):
	Task.__init__(self, world, robot, role)
        TurnToPoint.__init__(self, world, robot, role, pitch_object.x, self.robot_info.y, pitch_object)
        MoveToPoint.__init__(self, world, robot, role, self.robot_info.x, pitch_object.y)
	self.pitch_object = pitch_object

    def update(self):
        # Update turn to object and moving point
        self.y = self.pitch_object.y

    def execute(self):
        self.update()

        if (not TurnToPoint.turn(self)):
            TurnToPoint.execute(self)
        else:
            self.y = self.pitch_object.y
            MoveToPoint.execute(self)

class KickToPoint(TurnToPoint):
    '''Generic kick to point task. Rotate to face the point (x,y) and kick.'''

    #: Assigned True when the Task has kicked the ball
    kicked = False

    #: The action handle from kicking
    catch_handle = None
    kick_handle = None

    def __init__(self,world,robot,role,x,y):
        TurnToPoint.__init__(self, world, robot, role, x, y);

    def execute(self):

        if self.kicked:
            return

        # Turn if we still have to
        if TurnToPoint.turn(self):
            TurnToPoint.execute(self)
        else:

            # Open catcher
            if self.catch_handle is None:
                catch_handle = self.robot.open_catcher()

            # Kick
            elif catch_handle.finished and kick_handle is None:
                kick_handle = self.robot.kick()

            # Wait for kick to complete
            elif kick_handle.finished:
                kicked = True

class Shoot(KickToPoint):
    '''Simple Shoot task. Rotates the robot to face the goal and then kicks.'''
    
    def __init__(self,world,robot,role):
        x = world.their_goal.x
        y = world.their_goal.y
        KickToPoint.__init__(world,robot,role,x,y)
        
class AcquireBall(MoveToPoint, TurnToPoint):

    def __init__(self, world, robot, role, displacement_tolerance=30, rotation_tolerance=0.2):
        Task.__init__(self, world, robot, role)
        self.x = self.world.ball.x
        self.y = self.world.ball.y
        self.DISPLACEMENT_TOLERANCE = displacement_tolerance
        self.ROTATION_TOLERANCE = rotation_tolerance

    def update(self):
        self.x = self.world.ball.x
        self.y = self.world.ball.y

    turned = False
    moved = False
        
    def execute(self):
        if self.complete:
            robot.stop()
            return

        if self.turn():
            self.robot.open_catcher()
            if self.move():
                self.robot.catch()
                self.complete = True
