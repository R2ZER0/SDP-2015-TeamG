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

class PrintTask(Task):

	def __init__(self, world, robot, role, msg):
		super(PrintTask, self).__init__(world, robot, role)
		self.message = msg

	def __repr__(self):
		return self.message

	def execute(self):
		print self

class AcquireBall(Task):
	'''Acquire Ball task. Rotate to face the ball, move to an appropriate
	distance from it and then attempt to catch it. Uses two other tasks:
	TurnToPoint and MoveToPoint.'''
	
	def __init__(self,world,robot,role):
		super(AcquireBall,self).__init__(world,robot,role)

		self.catch_handle = None
		self.catch_handle2 = None

		self.turn_task = None
		self.move_task = None

		#: Becomes True once this Task has caught the ball
		self.complete = False


	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, if our catcher is closed, open it up.
		* Check if we're close enough to catch; if not, move forward.
		* Otherwise, attempt to catch the ball.
		'''

		if self.complete:
			return

		if self.turn_task is None:
			self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.world.ball.x, self.world.ball.y)
		elif not self.turn_task.complete:
			self.turn_task.execute()
			return
		elif self.catch_handle is None:
			self.catch_handle = self.robot.open_catcher()
		elif not self.catch_handle.completed:
			return
		elif self.move_task is None:
			self.move_task = MoveToPoint(self.world, self.robot, self.role, self.world.ball.x, self.world.ball.y, 20)
		elif not self.move_task.complete:
			self.move_task.execute()
			return
		elif self.catch_handle2 is None:
			self.catch_handle2 = self.robot.catch()
		elif not self.catch_handle2.completed:
			return
		else:
			self.complete = True

class MoveToPoint(Task):
	'''Movement Task. Rotates our Robot to face the point (x,y) and travels
	to it within some threshold.
	'''

	def __init__(self,world,robot,role,x,y, dist=35):
		super(MoveToPoint,self).__init__(world,robot,role)
		#: Specified x-position to travel to
		self.x = x
		#: Specified y-position to travel to
		self.y = y
		#: Distance threshold to the point before considering ourselves done
		self.DISP_TOLERANCE = dist
		#: Assigned True once we reach the point within the threshold
		self.complete = False

		self.move_handle = None
	
	def check(self):
		return self.robot_info.get_displacement_to_point(self.x,self.y) < self.DISP_TOLERANCE
	
	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''
		if self.complete:
			return

		if self.move_handle is None:
			angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.move_handle = self.robot.move(angle, 60)

		elif self.check():
			self.robot.stop()
			self.complete = True
	
class TurnToPoint(Task):
	'''Rotation Task; allows turning of the Robot to face a given point to
	within some threshold.
	'''

	def __init__(self,world,robot,role,x,y):
		super(TurnToPoint,self).__init__(world,robot,role)
		#: Our target x-position
		self.x = x
		#: Our target y-position
		self.y = y
		#: Gets assigned True once the Robot has rotated to the angle within accepted range.
		self.complete = False
		#: Margin of error allowed in this angle, radians.
		self.ANGLE_THRESHOLD = 0.1

		self.turn_handle = None

	def check(self):
		return abs(self.robot_info.get_rotation_to_point(self.x,self.y)) < self.ANGLE_THRESHOLD

	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''
		if self.complete:
			return

		if self.turn_handle is None:
			angle_diff = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.turn_handle = self.robot.turnBy(angle_diff)

		elif self.check():
			self.robot.stop()
			self.complete = True

class Shoot(Task):
	'''Simple Shoot task. Rotates the robot to face the goal and then kicks.'''
	
	def __init__(self,world,robot,role):
		super(Shoot,self).__init__(world,robot,role)
		self.x = world.their_goal.x
		self.y = world.their_goal.y
		self.DISP_TOLERANCE = 40
		self.complete = False

		self.turn_task = None
		self.catch_handle = None
		self.kick_handle = None
		self.last_time = 0

	def check(self):
		return self.robot_info.get_displacement_to_point(self.world.ball.x,self.world.ball.y) > self.DISP_TOLERANCE
	
	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''
		if self.complete:
			return

		if self.turn_task is None:
			self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.x, self.y)
		elif not self.turn_task.complete:
			self.turn_task.execute()
			return
		elif self.last_time == 0:
			self.last_time = time.time()
		elif not (time.time() - self.last_time) > 1:
			return
		elif self.catch_handle is None:
			self.catch_handle = self.robot.open_catcher()
			return
		elif not self.catch_handle.completed:
			return
		elif self.kick_handle is None:
			self.kick_handle = self.robot.kick()
			return
		elif not self.kick_handle.completed:
			return
		else:
			self.complete = True
