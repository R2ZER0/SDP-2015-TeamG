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
    DISPLACEMENT_TOLERANCE = 15

    #: Specified x-position to travel to
    x = 0

    #: Specified y-position to travel to
    y = 0

    #: The current angle to move.
    angle = 0

    #: Tracks the last speeds we sent to the Robot to check for sending more values
    motionHandle = None

    def __init__(self, world, robot, role, x, y, tolerance=15):
        super(MoveToPoint, self).__init__(world, robot, role)
        self.x = x
        self.y = y
        self.DISPLACEMENT_TOLERANCE = tolerance

    def move(self):
        """Updates this Task's motionHandle to move toward a new point."""
        if self.check_displacement():
            return True
        else:
            if self.motionHandle is None:
                self.angle = self.angle_to_point(self.x, self.y)
                self.angle = int(self.angle / (math.pi / 4)) * math.pi / 4  # round to nearest 45 degrees
                self.motionHandle = self.robot.move(self.angle, scale=self.get_movement_speed())
            else:
                if self.motionHandle.complete or self.motionHandle.finished:
                    self.motionHandle = None
            return False

    def check_displacement(self):
        """Check if the robot is within DISPLACEMENT_TOLERANCE units of self.x, self.y"""
        return self.get_displacement() <= self.DISPLACEMENT_TOLERANCE

    def get_displacement(self):
        return self.displacement_to_point(self.x, self.y)

    def get_movement_speed(self):
        return min(self.BASE_MOTOR_SPEED, max(self.get_displacement(), 40))

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

    def __init__(self, world, robot, role, x, y, tolerance=0.2):
        super(TurnToPoint, self).__init__(world, robot, role)
        self.x = x
        self.y = y
        self.ROTATION_TOLERANCE = tolerance

    def turn(self):
        """Updates this Task's motionHandle to move toward a new point."""
        if self.check_rotation():
            return True
        else:
            if self.motionHandle is None:
                rotation = self.get_rotation()
                self.motionHandle = self.robot.turnBy(rotation, scale=self.get_rotation_speed())
            else:
                if self.motionHandle.complete or self.motionHandle.finished:
                    self.motionHandle = None
            return False

    def check_rotation(self):
        """Check if the robot is closer than ROTATION_TOLERANCE radians to self.x, self.y"""
        return self.get_rotation() <= self.ROTATION_TOLERANCE

    def get_rotation(self):
        return self.angle_to_point(self.x, self.y)

    def get_rotation_speed(self):
        return min(self.BASE_MOTOR_SPEED, max(self.get_rotation() / math.pi * 100, 40))

    def execute(self):
        if self.complete:
            return

        if self.turn():
            self.complete = True


class AcquireBall(MoveToPoint, TurnToPoint):

    def __init__(self, world, robot, role, x, y, displacement_tolerance=30, rotation_tolerance=0.2):
        super(AcquireBall, self).__init__(world, robot, role)
        self.x = x
        self.y = y
        self.DISPLACEMENT_TOLERANCE = displacement_tolerance
        self.ROTATION_TOLERANCE = rotation_tolerance

    def execute(self):
        if self.complete:
            return

        if self.turn():
            if self.move():
                self.robot.catch()
                self.complete = True
