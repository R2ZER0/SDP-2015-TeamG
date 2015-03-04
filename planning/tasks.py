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
	THRESHOLD = 15
	#: Becomes True once this Task has caught the ball
	complete = False

	move_task = None
	turn_task = None
	c = None
	def __init__(self,world,robot,role):
		super(AcquireBall,self).__init__(world,robot,role)

	def get_move_task(self):
		'''Provides a MoveTask intialised with the position in front of the desired point.
		'''
		tx = self.world.ball.x - self.robot_info.x
		ty = self.world.ball.y - self.robot_info.y

		if self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y) < 110:
			l = math.hypot(tx, ty) / 30
			print 'Move Task 2'
		else:
			l = math.hypot(tx, ty) / 80
			print 'Move Task 1'

		tx = self.world.ball.x - tx / l
		ty = self.world.ball.y - ty / l
		self.move_task = MoveToPoint(self.world, self.robot, self.role, tx, ty)
		if self.robot_info.get_displacement_to_point(tx,ty) < 110:
			self.move_task.last_speed = 50
		else:
			self.move_task.last_speed = 70

	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, if our catcher is closed, open it up.
		* Check if we're close enough to catch; if not, move forward.
		* Otherwise, attempt to catch the ball.
		'''

		if self.complete:
			print "DONE"
			return
			
		
		
		if self.robot_info.can_catch_ball(self.world.ball):
			if self.turn_task != None and self.turn_task.complete:
				if self.move_task != None:
					self.move_task = None
					self.robot.stop()
				if self.robot_info.catcher == 'closed':
					print "OPEN"
					self.robot.open_catcher()
					self.robot.open_catcher()
					self.robot_info.catcher = 'open'
				else:
					if self.c == None or (self.c.finished and not self.c.completed):
						self.c = self.robot.catch()
						print "CAUGHT"
						return
					elif self.c.completed:
						self.complete = True
						return
			elif self.turn_task != None:
				self.turn_task.execute()

		if self.turn_task == None:	
			self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.world.ball.x, self.world.ball.y)
		if self.turn_task.complete and (self.move_task == None or (self.move_task.complete and self.robot_info.get_displacement_to_point(self.world.ball.x ,self.world.ball.y) < 110)):
			if not self.turn_task.check():
				self.turn_task.update_position(self.world.ball.x, self.world.ball.y)
				return
						
			self.get_move_task()

		if hypot(self.turn_task.x - self.world.ball.x, self.turn_task.y - self.world.ball.y) > self.THRESHOLD:
			print "UPDATE"
			if self.turn_task != None:
				self.turn_task.update_position(self.world.ball.x, self.world.ball.y)
			if self.move_task != None:
				self.get_move_task()
			if self.robot_info.catcher == 'closed':
				print "OPEN"
				self.robot.open_catcher()
				self.robot_info.catcher = 'open'
			return

		if self.turn_task.complete and self.move_task.complete:
			if self.robot_info.can_catch_ball(self.world.ball):
				if self.c == None or (self.c.finished and not self.c.completed):
					self.c = self.robot.catch()
					print "CAUGHT"
					return
				elif self.c.completed:
					self.complete = True
					return
			else:
				self.turn_task.update_position(self.world.ball.x, self.world.ball.y)
		
		if self.turn_task.complete:
			#Open the catcher.
			if self.robot_info.catcher == 'closed':
				print "OPEN"
				self.robot.open_catcher()
				self.robot_info.catcher = 'open'
			self.move_task.execute()
			print "MOVE - ",
		else:
			print "TURN - ",
			self.turn_task.execute()
			return



class MoveToPoint(Task):
	'''Movement Task. Rotates our Robot to face the point (x,y) and travels
	to it within some threshold.
	'''

	#: Distance threshold to the point before considering ourselves done
	DISP_TOLERANCE = 20
	TRAJECTORY_CONTROL = 0.1
	#: Specified x-position to travel to
	x = 0
	
	#: Specified y-position to travel to
	y = 0

	#: Tracks the last speeds we sent to the Robot to check for sending more values
	last_speed = 0
	base_speed = 70
	angle = 0
	motionHandle = None
	#: Assigned True once we reach the point within the threshold
	complete = False

	def __init__(self,world,robot,role,x,y, dist = 20):
		super(MoveToPoint,self).__init__(world,robot,role)
		self.x = x
		self.y = y
		self.last_speed = self.base_speed
		self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
		self.angle = int(self.angle / (math.pi/4))*(math.pi/4)
		self.motionHandle = None
		self.DISP_TOLERANCE = dist
	def update_position(self, x, y):

		if x == self.x and y == self.y:
			return

		self.x = x
		self.y = y
		self.last_speed = self.base_speed
		self.motionHandle = None

		self.complete = False
	
	def check(self):
		return self.robot_info.get_displacement_to_point(self.x,self.y) < self.DISP_TOLERANCE
	
	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''
		if self.complete:
			return
		print self.angle
		if self.motionHandle == None:
			self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.angle = int(self.angle / (math.pi/4))*math.pi/4
			self.motionHandle = self.robot.move(self.angle, scale = self.last_speed)
			self.last_speed = max(self.last_speed - 10, 40)
		else:
			if self.motionHandle.completed:
				if not self.check():
					if abs(-self.robot_info.get_rotation_to_point(self.x,self.y) - self.angle) < self.thresh():
						print abs(-self.robot_info.get_rotation_to_point(self.x,self.y) - self.angle), self.thresh()
					else:
						self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
						self.angle = int(self.angle / (math.pi/4))*math.pi/4
						self.motionHandle = self.robot.move(self.angle, scale = self.last_speed)
						self.last_speed = max(self.last_speed - 10, 40)
						print abs(-self.robot_info.get_rotation_to_point(self.x,self.y) - self.angle), self.thresh()
						print "REDO1"
				else:
					self.motionHandle = self.robot.stop()
					self.last_speed = self.base_speed
					self.complete = True
					print "STOPPED"
			elif self.motionHandle.finished:
				if self.check():
					self.complete = True
					self.robot.stop()
					print "STOPPED"
				else:
					self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
					self.angle = int(self.angle / (math.pi/4))*math.pi/4
					self.motionHandle = self.robot.move(self.angle, scale = self.last_speed)
					print "REDO2"
			else: print "MOVE ALT"

	def thresh(self):
		if self.robot_info.get_displacement_to_point(self.x,self.y) > 100:
			return 0.2
		else:
			return 0.2 + 20 / self.robot_info.get_displacement_to_point(self.x,self.y)

class MirrorObject(Task):
	'''Mirrors the y-movement of a robot or the ball.
	'''

	#: Distance threshold to the point before considering ourselves done
	DISP_TOLERANCE = 30
	TRAJECTORY_CONTROL = 0.2

	#: obj
	obj = None

	last_speed = 0
	#: Assigned True once we reach the point within the threshold
	complete = False

	turnHandle = None

	def __init__(self,world,robot,role,obj):
		super(MirrorObject,self).__init__(world,robot,role)
		self.obj = obj
		self.angle = self.robot_info.get_rotation_to_point(self.robot_info.x, obj.y)
		self.motionHandle = None

	def update_position(self, obj):
		if obj == self.obj:
			return

		self.obj = obj
		self.complete = False
	
	def check(self):
		return self.robot_info.get_displacement_to_point(self.robot_info.x, self.obj.y) < self.DISP_TOLERANCE
	
	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''

		if self.turnHandle == None:
			angle = -self.robot_info.get_rotation_to_point(self.obj.x,self.robot_info.y)
			self.turnHandle = self.robot.turnBy(angle)

		if self.turnHandle.completed:
			if self.motionHandle == None:
				self.angle = -self.robot_info.get_rotation_to_point(self.robot_info.x, self.obj.y)
				self.motionHandle = self.robot.move(self.angle, scale = self.calc_speed())
				self.last_speed = self.calc_speed()
			else:
				if self.motionHandle.completed or self.motionHandle.finished:
					if not self.check():
						if abs(-self.robot_info.get_rotation_to_point(self.robot_info.x, self.obj.y) - self.angle) < self.TRAJECTORY_CONTROL and 						   abs(self.calc_speed() - self.last_speed) < 10:
							print "WAIT"
						else:
							self.angle = -self.robot_info.get_rotation_to_point(self.robot_info.x, self.obj.y)
							self.motionHandle = self.robot.move(self.angle, scale = self.calc_speed())
							self.last_speed = self.calc_speed()
					else:
						self.robot.stop()

	def calc_speed(self):
		speed = math.sin(self.obj.angle) * self.obj.velocity
		return min(100, max(50, speed))
		
class TurnToPoint(Task):
	'''Rotation Task; allows turning of the Robot to face a given point to
	within some threshold.
	'''

	#: Margin of error allowed in this angle, radians.
	ANGLE_THRESHOLD = 0.1
	
	#: Our target x-position
	x = 0
	
	#: Our target y-position
	y = 0

	#: Tracks the last speeds we sent to the Robot to check for sending more values
	last_speed = 0
	motionHandle = None
	#: Gets assigned True once the Robot has rotated to the angle within accepted range.
	complete = False

	base_speed = 70

	def __init__(self,world,robot,role,x,y):
		super(TurnToPoint,self).__init__(world,robot,role)
		self.x = x
		self.y = y
		self.last_speed = self.base_speed

	def update_position(self, x, y):

		if x == self.x and y == self.y:
			return

		self.x = x
		self.y = y
		self.last_speed = self.base_speed
		self.motionHandle = None

		self.complete = False
	def check(self):
		return abs(self.robot_info.get_rotation_to_point(self.x,self.y)) < self.ANGLE_THRESHOLD
	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, we're done.
		'''
		if self.complete:
			return

		if self.motionHandle == None:
			angle_to_turn = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.motionHandle = self.robot.turnBy(angle_to_turn, scale = self.last_speed)
			self.last_speed = max(self.last_speed - 10, 40)
			print angle_to_turn
		else:
			if self.motionHandle.completed:
				if self.check():
					self.complete = True
				else:
					angle_to_turn = -self.robot_info.get_rotation_to_point(self.x,self.y)
					self.motionHandle = self.robot.turnBy(angle_to_turn, scale = self.last_speed)
					self.last_speed = max(self.last_speed - 10, 40)
					print angle_to_turn
			elif self.motionHandle.finished:
				if abs(self.robot_info.angular_velocity) < 0.1 and self.check():
					self.complete = True
				else:
					angle_to_turn = -self.robot_info.get_rotation_to_point(self.x,self.y)
					self.motionHandle = self.robot.turnBy(angle_to_turn, scale = self.last_speed)
					self.last_speed = max(self.last_speed - 10, 40)
					print angle_to_turn
			else:
				pass

class KickToPoint(Task):
	'''Generic kick to point task. Rotate to face the point (x,y) and kick.'''

	#: Allowed margin of error between the desired angle and ours.
	ANGLE_THRESHOLD = 0.1
	
	#: The target x-point to kick to.
	x = 0
	
	#: The target y-point to kick to.
	y = 0

	#: Assigned True when the Task has kicked the ball
	complete = False

	#: This increases through various values as we perform checks to ensure \
	#: we're on target to kick accurately.
	check_state = 0

	turn_task = None
	kickHandle = None
	
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

		if self.complete:
			return

		if self.turn_task is None:
			self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.x, self.y)

		if not self.turn_task.complete:
			self.turn_task.execute()
			return
		
		if self.kickHandle == None:
			self.kickHandle = self.robot.kick()

		if self.kickHandle != None and not self.kickHandle.completed:
			pass
		else:
			self.complete = True

class Shoot(KickToPoint):
	'''Simple Shoot task. Rotates the robot to face the goal and then kicks.'''
	
	def __init__(self,world,robot,role):
		x = world.their_goal.x
		y = world.their_goal.y
		super(Shoot,self).__init__(world,robot,role,x,y)

