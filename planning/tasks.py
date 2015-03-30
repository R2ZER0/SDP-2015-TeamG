from utilities import *
from action import Action
import time

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
		if self.complete:
			return

		if self.turnHandle == None:
			angle_to_turn = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.turnHandle = self.robot.turnBy(angle_to_turn)
		else:
			if self.turnHandle.completed:
				if self.check():
					self.complete = True
				else:
					self.turnHandle = None
			elif self.turnHandle.finished:
				if abs(self.robot_info.angular_velocity) < 0.1 and self.check():
					self.complete = True
				else:
					self.turnHandle = None
			else:
				pass

class MoveToPoint(Task):
	'''Movement Task. Travels to point(x,y) within some threshold.
	'''
	def __init__(self, world, robot, role, x, y, speed = 100, tolerance = 20):
		super(MoveToPoint,self).__init__(world,robot,role)
		self.x = x
		self.y = y
		self.speed = speed
		self.motionHandle = None
		self.DISP_TOLERANCE = tolerance
		self.TRAJECTORY_CONTROL = 0.1

	
	def check(self):
		return self.robot_info.get_displacement_to_point(self.x,self.y) < self.DISP_TOLERANCE

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
		if self.complete:
			return
		if self.motionHandle == None:
			self.angle = -self.robot_info.get_rotation_to_point(self.x,self.y)
			self.motionHandle = self.robot.move(self.angle, scale = self.speed)
		else:
			if self.motionHandle.completed:
				if not self.check():
					if abs(-self.robot_info.get_rotation_to_point(self.x,self.y) - self.angle) < self.angle_tolerance():
						pass
					else:
						motionHandle = None
				else:
					self.motionHandle = self.robot.stop()
					self.complete = True
			elif self.motionHandle.finished:
				if self.check():
					self.complete = True
					self.robot.stop()
				else:
					self.motionHandle = None


class AcquireBall(Task):
	'''Acquire Ball task. Rotate to face the ball, move to an appropriate
	distance from it and then attempt to catch it.'''

	def __init__(self,world,robot,role):
		super(AcquireBall,self).__init__(world,robot,role)
		self.move_task = None
		self.turn_task = None
		self.catch = None

	def updateTurn(self):
		if self.turn_task == None:
			return
		elif hypot(self.turn_task.x - self.world.ball.x, self.turn_task.y - self.world.ball.y) > 10:
			self.turn_task = None

	def updateMove(self):
		if self.move_task == None:
			return
		elif self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y) < 70 and self.move_task.speed == 100:
			self.move_task = None

	def execute(self):
		'''Executes another round of this Task. Performs as follows:

		* Check if our angle is on target; if not, adjust by rotating.
		* Otherwise, if our catcher is closed, open it up.
		* Check if we're close enough to catch; if not, move forward.
		* Otherwise, attempt to catch the ball.
		'''

		if self.complete:
			return

		if self.robot_info.can_catch_ball(self.world.ball):
			self.robot.stop()
			if self.robot_info.catcher == 'closed':
				self.robot.open_catcher()
				self.robot_info.catcher = 'open'
				
			else:
				if self.catch == None or (self.catch.finished and not self.catch.completed):
					self.catch = self.robot.catch()
				elif self.catch.completed:
					self.complete = True
			return
		else:
			self.updateTurn()

			if self.turn_task == None:
				self.turn_task = TurnToPoint(self.world, self.robot, self.role, self.world.ball.x, self.world.ball.y)
				return
			elif not self.turn_task.complete:
				self.turn_task.execute()
				return

			self.updateMove()

			if self.move_task == None:

				if self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y) > 70:
					target_x = self.world.ball.x
					target_y = self.world.ball.y
					speed = 100
				else:
					scale = 20 / self.robot_info.get_displacement_to_point(self.world.ball.x, self.world.ball.y)
					target_x = self.world.ball.x - (self.world.ball.x - self.robot_info.x)*scale
					target_y = self.world.ball.y - (self.world.ball.y - self.robot_info.y)*scale
					speed = 60
				self.move_task = MoveToPoint(self.world, self.robot, self.role, target_x, target_y, speed = speed)

			elif not self.move_task.complete:
				self.move_task.execute()




class MirrorObject(Task):
	'''Mirrors the y-movement of a robot or the ball.
	'''

	def __init__(self,world,robot,role,obj):
		super(MirrorObject,self).__init__(world,robot,role)
		self.DISP_TOLERANCE = 30
		self.TRAJECTORY_CONTROL = 0.2
		self.obj = obj
		self.angle = self.robot_info.get_rotation_to_point(self.robot_info.x, obj.y)
		self.motionHandle = None
		last_speed = 0
		self.turnHandle = None
	
	def check(self):
		return self.robot_info.get_displacement_to_point(self.robot_info.x, self.obj.y) < self.DISP_TOLERANCE
	
	def execute(self):
		'''Executes another round of this Task.
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
						if abs(-self.robot_info.get_rotation_to_point(self.robot_info.x, self.obj.y) - self.angle) > self.TRAJECTORY_CONTROL:
							self.turnHandle = None
						if abs(self.calc_speed() - self.last_speed) > 10:
							self.motionHandle = None
					else:
						self.robot.stop()

	def calc_speed(self):
		speed = math.sin(self.obj.angle) * self.obj.velocity
		return min(100, max(50, speed))


class KickToPoint(Task):
	'''Generic kick to point task. Rotate to face the point (x,y) and kick.'''

	def __init__(self,world,robot,role,x,y):
		super(KickToPoint,self).__init__(world,robot,role)
		self.x = x
		self.y = y
		self.turn_task = None
		self.kickHandle = None

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





