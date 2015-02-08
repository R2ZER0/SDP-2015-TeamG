'''
Simulator is a simple simulation of the Pitch, Ball, and Robots. It allows control of
one robot and simulates movement and collisions between the objects of the pitch.

This file contains three classes, all of which are required for a Simulation. 

* Simulator
	provides the core logic of the simulation 
* SimulatedAction 
	provides the responses to Action hooks and serves as in place replacement of Action class.
* SimulatedCamera
	provides the image frames of the simulation that feed directly into vision systme.
'''

from planning.models import Vector, World
import cv2
import numpy as np
import sys
import random
import math

import pymunk
from pymunk.constraint import PivotJoint, GearJoint
from pymunk import Vec2d

class SimulatedCamera:
	'''
	Simulates the camera above the Pitch. Gets fed robot positions and ball positions and draws
	a frame that is retrievable via get_frame.

	Colours are simulated in very simple manners such that thresholding is easy for the vision.
	'''
	PITCH_HEIGHT, PITCH_WIDTH = 640, 480

	# Our reference frame, used as the drawing buffer
	frame = None

	def __init__(self):
		'''
		Initialises basic object positions and sets up our frame for grabbing.
		'''
		# Initialise our blank frame
		self.frame = np.zeros((self.PITCH_WIDTH, self.PITCH_HEIGHT, 3), np.uint8)

	def get_frame(self):
		'''
		Similar to Camera, returns a frame containing the image of the Pitch in it's current state.
		'''
		return self.frame

	def fix_radial_distortion(self, frame):
		pass

	def get_adjusted_center(self, frame):
		return (320, 240)

	def draw(self, world):
		'''
		Top-level draw call. Draws the scene to the frame. Gets passed a World model
		and retrieves robot / ball positions from this.
		'''
		self._draw_pitch()

		for robot in [world.our_defender, world.our_attacker]:
			self._draw_robot(robot, 'yellow')

		for robot in [world.their_attacker, world.their_defender]:
			self._draw_robot(robot, 'blue')

		self._draw_ball(world.ball)

	def _draw_pitch(self):
		'''
		Draws the pitch background, zones, and borders to the image.
		'''
		cv2.fillConvexPoly(self.frame, np.array([[0,0], [640,0], [640,480], [0,480]]), (50,50,50))
		# Draw white border
		cv2.fillConvexPoly(self.frame, np.array([[0,100], [50,0], [590,0], [640, 100], [640,380], [590, 480], [50,480], [0,380]]), (230,224,226))

		# Draw inner green patch
		cv2.fillConvexPoly(self.frame, np.array([[10,100], [60,10], [570,10], [630, 100], [630,380], [570, 470], [60,470], [10,380]]), (74,93,71))

		# Draw stripes
		cv2.fillConvexPoly(self.frame, np.array([[140,0], [180,0], [180,480], [140,480]]), (230,224,226))
		cv2.fillConvexPoly(self.frame, np.array([[300,0], [340,0], [340,480], [300,480]]), (230,224,226))
		cv2.fillConvexPoly(self.frame, np.array([[460,0], [500,0], [500,480], [460,480]]), (230,224,226))

	def _draw_robot(self, robot, color):
		'''
		Draws this robot to the frame.
		'''

		# Rotate our points 
		points = [[robot.x-robot.length/2, robot.y-robot.width/2], # Upper left
				  [robot.x+robot.length/2, robot.y-robot.width/2], # Upper right
				  [robot.x+robot.length/2, robot.y+robot.width/2], # Lower right
				  [robot.x-robot.length/2, robot.y+robot.width/2]] # Lower left

		points = map(lambda x: self._trans_point(x, (robot.x,robot.y), robot.angle), points)
		cv2.fillConvexPoly(self.frame, np.array(points), (0, 255, 0))

		# draw dot
		dot_point = self._trans_point((robot.x+10, robot.y), (robot.x,robot.y), robot.angle)
		cv2.circle(self.frame, (dot_point[0], dot_point[1]), 4, (0,0,0), -1)

		# draw i
		i_points = [[robot.x, robot.y-5], [robot.x-15, robot.y-5],
					[robot.x-15, robot.y+5], [robot.x, robot.y+5]]

		i_points = map(lambda x: self._trans_point(x, (robot.x,robot.y), robot.angle), i_points)
		i_color = (0, 255, 255) if color == 'yellow' else (255, 0, 0) 
		cv2.fillConvexPoly(self.frame, np.array(i_points), i_color)

	def _trans_point(self, point, center, angle):
		'''
		Takes the given point and applies the given angle rotation, centred on
		the center.
		'''
		# convert robot's angle to correct domain
		angle = (-angle + math.pi/2)

		sx = center[0] - point[0]
		sy = center[1] - point[1]

		hyp = math.sqrt((sx**2) + (sy**2))

		src_angle = math.atan2(sy, sx)
		dst_angle = src_angle + angle

		dx = hyp*math.sin(dst_angle)
		dy = hyp*math.cos(dst_angle)

		return [int(center[0] + dx), int(center[1] + dy)]

	def _draw_ball(self, ball):
		''' 
		Draws the ball to the frame
		'''
		cv2.circle(self.frame, (int(ball.x), int(ball.y)), ball.width, (0,0,255), -1)

class SimulatedAction:
	'''
	SimulatedAction provides in-place replacement for the Action class. This replaces calls to communications by calls
	to the Simulator instance updating Robot's actions.
	'''

	# A reference to our simulator instance for passing messages
	simulator = None

	# Motor angles, starting from the front left, going anticlockwise
	# MOTOR_ANGLES = [ math.pi/3.0, math.pi, math.pi*5.0/3.0 ]
	MOTOR_ANGLES = [ math.pi*5.0/6.0, math.pi*3.0/2.0, math.pi/6.0 ]
	MOTORS = [ (math.cos(angle), math.sin(angle)) for angle in MOTOR_ANGLES ]

	def __init__(self, simulator):
		assert simulator is not None

		self.simulator = simulator

	def move(self, angle, scale):
		"""Move the robot in the angle (radians) from the front, with speed -1 to +1"""

		motor_speeds = [ scale * SimulatedAction._calc_motor_speed(motor, angle) for motor in self.MOTORS ]
		#motor_speeds = SimulatedAction._normalise_speeds(motor_speeds)
		#motor_speeds = map(lambda x: x*scale, motor_speeds)
		#motor_speeds = map(SimulatedAction._percentage_speed, motor_speeds)	
		self._send_run(motor_speeds)
  
	def turn(self, speed):
		speed = speed / 5
		"""Turn the robot in the given direction, clockwise/anticlockwise"""      
		speed = int(speed)
		self._send_run([speed, speed, speed])
  
	def stop(self):
		self.simulator.our_attacker_body.angular_velocity = 0
		self.simulator.our_attacker_body.velocity = (0,0)
  
	# Kicking
	def kick(self, scale=100):
		world = self.simulator.world
		robot = world.our_attacker

		if self.simulator.ball_posssessed:
			self.simulator.ball_posssessed = False

			dx = scale * math.cos(robot.angle)
			dy = scale * math.sin(robot.angle)

			self.simulator.ball_body.apply_impulse((dx, dy), (0,0))
	
	def catch(self, scale=100):
		world = self.simulator.world
		robot = world.our_attacker

		if robot.can_catch_ball(world.ball):
			self.simulator.ball_posssessed = True
			catcher_center = (robot.x + (30 * math.cos(robot.angle)), robot.y + (30 * math.sin(robot.angle)))
			world.ball.vector = Vector(catcher_center[0], catcher_center[1], 0, 0)
			self.simulator.ball_body.position = (catcher_center[0], catcher_center[1])

	def open_catcher(self, scale=100):
		self.kick(100)

	# Private

	# Utility
	@staticmethod
	def _normalise_speeds(speeds):
		maxspeed = max(speeds)
		return map( lambda x: (x/maxspeed)*x, speeds )        
	
	@staticmethod
	def _percentage_speed(speed):
		# We need the speed to be an integer between 100 and -100
		speed = int(round(100.0 * speed))
		speed = max(-100, min(speed, 100))
		return speed
	
	@staticmethod
	def _calc_motor_speed(motor, angle):
		return (math.cos(angle) * motor[0] - math.sin(angle) * motor[1])

	def _send_run(self, speeds):
		self.stop()

		robot = self.simulator.our_attacker_body

		# calculate current angle of wheels around robot body
		wheel_angles = map( lambda x: x - math.pi/2 + robot.angle, self.MOTOR_ANGLES )

		# Calculate where the centre of our wheels are and their facing angle
		wheel_pos = map( lambda x: (x+math.pi/2, (15)*math.cos(x), (15)*math.sin(x)), wheel_angles )

		# Given wheel angle, calculate vector for the force
		for num, motor in enumerate(wheel_pos):
			robot.apply_impulse((speeds[num]*math.cos(motor[0]), speeds[num]*math.sin(motor[0])),
								(motor[1], motor[2]))

class Simulator:
	'''
	Our Simulator contains the logic for collisions, movement, and updating our internal
	simulated world state. This provides the core logic of the simulation.
	'''

	# Our internal World model. Stores positions of robots and ball and allows simple referencing
	world = None

	# Pymunk Physics Objects
	space = None

	# Bodies for each robot
	our_attacker_body = None
	our_defender_body = None
	their_attacker_body = None
	their_defender_body = None

	# Body for the ball
	ball_body = None

	ball_posssessed = False

	def __init__(self):

		# Initialise the world
		self.world = World(our_side='left', pitch_num=1)

		# Assign initial positions
		self.world.our_defender.vector = Vector(80, 240, 0, 0)
		self.world.our_attacker.vector = Vector(400, 240, 0, 0)
		self.world.their_defender.vector = Vector(560, 240, 0, 0)
		self.world.their_attacker.vector = Vector(240, 240, 0, 0)
		self.world.ball.vector = Vector(400, 120, 0, 0)

		self._construct_world()

	def move_ball(self, angle):

		dx = 25 * math.cos(angle)
		dy = 25 * math.sin(angle)
		
		self.ball_body.apply_impulse((dx,dy),(0,0))

	def _construct_world(self):
		'''
		Constructs the physical simulation of our pitch.
		'''
		self.space = pymunk.Space()
		self.space.gravity = Vec2d(0,0)
		self.space.damping = 0.95

		# Construct the pitch boundaries
		static_body = pymunk.Body()
		static_lines = [pymunk.Segment(static_body, Vec2d(0,100), Vec2d(50,0), 10),
						pymunk.Segment(static_body, Vec2d(50,0), Vec2d(590,0), 10),
						pymunk.Segment(static_body, Vec2d(590,0), Vec2d(640,100), 10),
						pymunk.Segment(static_body, Vec2d(640,100), Vec2d(640,380), 10),
						pymunk.Segment(static_body, Vec2d(640,380), Vec2d(590,480), 10),
						pymunk.Segment(static_body, Vec2d(590,480), Vec2d(50,480), 10),
						pymunk.Segment(static_body, Vec2d(50,480), Vec2d(0,380), 10),
						pymunk.Segment(static_body, Vec2d(0,380), Vec2d(0,100), 10)]

		# Increase friction of each border
		for line in static_lines:
			line.friction = 0.4

		self.space.add(static_lines)

		robots = [self.world.our_attacker, self.world.our_defender, self.world.their_attacker, self.world.their_defender]

		self.our_attacker_body = self._construct_robot_body(self.world.our_attacker)
		self.our_defender_body = self._construct_robot_body(self.world.our_defender)
		self.their_attacker_body = self._construct_robot_body(self.world.their_attacker)
		self.their_defender_body  = self._construct_robot_body(self.world.their_defender)
		self.ball_body = self._construct_ball(self.world.ball)

	def _construct_robot_body(self, robot):
		'''
		Constructs the robot's body at the appropriate position and returns the body and shape.
		'''
		points = [[-robot.length/2, -robot.width/2], # Upper left
				  [robot.length/2, -robot.width/2], # Upper right
				  [robot.length/2, +robot.width/2], # Lower right
				  [-robot.length/2, +robot.width/2]] # Lower left

		mass = 10.0
		moment = pymunk.moment_for_poly(mass, points, (0,0))
		body = pymunk.Body(mass, moment)
		body.position = Vec2d(robot.x, robot.y)
		shape = pymunk.Poly(body, points, (0,0))

		shape.elasticity = 0
		shape.friction = 0.8

		self.space.add(body,shape)

		return body

	def _construct_ball(self, ball):
		mass = 2.0
		moment = pymunk.moment_for_circle(mass, 0, 10, (0,0))
		body = pymunk.Body(mass, moment)
		body.position = Vec2d(ball.x, ball.y)
		shape = pymunk.Circle(body, 10, (0,0))
		shape.friction = 0.1

		self.space.add(body,shape)

		return body

	def _bodies(self):
		return [self.our_attacker_body, self.our_defender_body, self.their_defender_body, self.their_attacker_body]

	def update(self, dt=1):
		'''
		Steps the simulatoin and updates the world state to that of the simulation, returns the world object.
		'''
		self.space.step(dt)

		# If we have the ball, continue moving it each update
		if self.ball_posssessed:
			robot = self.world.our_attacker
			catcher_center = (robot.x + (30 * math.cos(robot.angle)), robot.y + (30 * math.sin(robot.angle)))
			self.world.ball.vector = Vector(catcher_center[0], catcher_center[1], 0, 0)
			self.ball_body.position = (catcher_center[0], catcher_center[1])

		# Manual damping on robot's angular velocities
		for body in self._bodies():
			if body.angular_velocity > 0.1:
				body.angular_velocity -= 0.005
			elif body.angular_velocity < -0.1:
				body.angular_velocity += 0.005
			else:
				body.angular_velocity = 0

		# Retrieve positions of all bodies in the space
		our_attacker = Vector(self.our_attacker_body.position.x, self.our_attacker_body.position.y, self._fix_angle(self.our_attacker_body.angle), 0)
		our_defender = Vector(self.our_defender_body.position.x, self.our_defender_body.position.y, self._fix_angle(self.our_defender_body.angle), 0)
		their_defender = Vector(self.their_defender_body.position.x, self.their_defender_body.position.y, self._fix_angle(self.their_defender_body.angle), 0)
		their_attacker = Vector(self.their_attacker_body.position.x, self.their_attacker_body.position.y, self._fix_angle(self.their_attacker_body.angle), 0)
		ball = Vector(self.ball_body.position.x, self.ball_body.position.y, 0, 0)

		self.world.update_positions({'our_attacker': our_attacker, 'our_defender': our_defender,
									 'their_attacker': their_attacker, 'their_defender': their_defender,
									 'ball': ball})

		return self.world

	def _fix_angle(self, angle):
		return angle % (2*math.pi)

if __name__ == '__main__':
	sim = Simulator()
	act = SimulatedAction(sim)
	cam = SimulatedCamera()

	cv2.namedWindow('Simulator', cv2.CV_WINDOW_AUTOSIZE)

	c = True
	while (c != 27 and c != 1048603):  # the ESC key
		
		# Retrieve updated world
		world = sim.update(2)

		# Draw frame
		cam.draw(world)

		# Retrieve frame
		frame = cam.get_frame()

		cv2.imshow('Simulator', frame)

		c = cv2.waitKey(2)
		if c == -1:
			continue
		
		print 'Pressed %d' % (c)
		
		if (c == 2490368 or c == 1113938):
			act.move(0, 5)
		elif (c == 2621440 or c == 1113940):
			act.move(math.pi, 5)
		elif (c == 1048689):
			act.move(math.pi*3/4, 5)
		elif (c == 1048695):
			act.move(math.pi*5/4, 5)
		elif (c == 2424832 or c == 1113937):
			act.turn(-5)
		elif (c == 2555904 or c == 1113939):
			act.turn(5)
		elif c == 32:
			act.kick()