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

from Polygon.cPolygon import Polygon

import pymunk
from pymunk.constraint import PivotJoint, GearJoint
from pymunk import Vec2d

from entities import SimulatedRobot, SimulatedBall, SimulatedPitch
from action import *
import numpy as np
import cv2

import math

class SimulatedAction:
	'''SimulatedAction provides in-place replacement for the Action class. This replaces calls to communications by calls
	to a SimulatedRobot updating the motors.
	'''

	def __init__(self, robot):
		'''Initialises this Action instance. Uses the passed robot as the object to pass messages
		to. 

		:param robot: A SimulatedRobot, retrievable from :func:`Simulator.our_attacker` and similar functions.
		'''
		assert robot is not None
		self.robot = robot

		# Command handles
		self.move_handle = MovementActionHandle(100, 'S', 0, 0)
		self.kick_handle = KickerActionHandle(100, 'I', 0)
		self.catch_handle = CatcherActionHandle(100, 'I', 0)

	def _cmd_movement(self, cmd, angle, scale):
		self.move_handle._onNextCommand()
		self.move_handle = MovementActionHandle(self.move_handle.idx+1, cmd, angle, scale)
		return self.move_handle

	def exit(self):
		pass

	def last_command(self):
		current_handle = self.robot.move_handle

		if current_handle is None:
			return [0,0,0]
		elif current_handle.cmd == 'M':
			dx = math.cos(self.robot.move_handle.dir)*self.robot.move_handle.spd
			dy = math.sin(self.robot.move_handle.dir)*self.robot.move_handle.spd
			return [dx,dy,0]
		elif current_handle.cmd == 'T':
			return [0,0,self.robot.move_handle.spd]
		else:
			return [0,0,0]

	def move(self, angle, scale=64):
		'''Move the robot in the angle (radians) from the front, with speed -1 to +1'''
		self.move_handle = self._cmd_movement('M', angle, scale)
		self.robot.move(self.move_handle)

		return self.move_handle
  
	def turnBy(self, angle, scale=64):
		'''Moves the robot in the given direction.

		:param speed: A value to turn, with positive being clockwise, in the range [-100,100]
		'''
		# Make new angle target
		target = mkangle(self.robot.body.angle+angle)
		self.move_handle = self._cmd_movement('T', target, scale)
		self.robot.turn(self.move_handle)

		return self.move_handle
  
	def stop(self):
		'''Sends 0 to all Robot's motors to stop the movement.
		'''
		self.move_handle = self._cmd_movement('S', 0, 0)
		self.robot.stop(self.move_handle)

		return self.move_handle
  
	# Kicking
	def kick(self, scale=100):
		'''Attempts a kick. Makes no check as to kicker/catcher being open/closed.

		.. note::

		   Scale currently not used.

		:param scale: Defaults to 100, the kicking power.
		'''
		self.kick_handle._onNextCommand()
		self.kick_handle = KickerActionHandle(self.kick_handle.idx+1, 'K', scale)
		self.robot.kick(self.kick_handle)
		return self.kick_handle

	def _cmd_catcher(self, cmd, scale):
		self.catch_handle._onNextCommand()
		self.catch_handle = CatcherActionHandle(self.catch_handle.idx+1, cmd, scale)
		return self.catch_handle

	def catch(self, scale=100):
		'''Attempts to catch using the catcher. Makes no check as to kicker/catcher being open/closed.

		.. note::

		   Scale currently not used.

		:param scale: Defaults to 100, the catching power.
		'''
		catch_handle = self._cmd_catcher('C', scale)
		self.robot.close_catcher(catch_handle)
		return catch_handle

	def open_catcher(self, scale=100):
		'''Attempts to open the catcher.

		.. note::

		   Scale not currently used.

		:param scale: Defaults to 100, speed to open.
		'''
		catch_handle = self._cmd_catcher('R', scale)
		self.robot.open_catcher(catch_handle)
		return catch_handle

class SimulatedCamera():

	#: Holds the instance of our simulation
	simulator = None

	#: The frame we draw the simulation to, overwritten each draw rather than instantiating a new one
	frame = None

	def __init__(self, simulator):
		'''Given a simulator instance, provides convenient access to retrieving a current image of the simulation;
		mimicking a camera interface to the Pitch.

		:param simulator: A Simulator instance.
		'''
		self.simulator = simulator
		self.frame = np.zeros((self.simulator.width, self.simulator.length, 3), np.uint8)

	def get_frame(self):
		'''Similar to Camera, returns a frame containing the image of the Pitch in it's current state.
		'''
		self.simulator.draw(self.frame)

		return self.frame

	def fix_radial_distortion(self, frame):
		pass

	def get_adjusted_center(self, frame):
		return (self.simulator.width/2, self.simulator.length/2)

class Simulator():
	'''Runs the Simulation. Provides convenient interfaces to objects in the simulation and manages setup.
	'''

	#: Pymunk Variables

	#: The pymunk space being simulated
	space = None

	#: The four robots being simulated; SimulatedRobot objects
	robots = []

	#: The SimulatedBall object
	ball = None

	#: Represents the complete pitch in the world (zones, drawing, border)
	pitch = None

	#: General Game Rules
	
	#: Our side, 'left' or 'right', role, 'attacker' or 'defender', and colour 'yellow' or 'blue'
	our_side, our_role, our_color = None, None, None

	def __init__(self, side, role, color):
		'''Constructs a new simulation, with our controlled robot being based on the given side,
		role, and colour.

		:param side: Either left or right, based upon the side of the frame
		:param role: 'attacker' or 'defender'
		:param color: The colour for our robot's plates, 'yellow' or 'blue'
		'''
		assert side in ['left', 'right']
		assert role in ['attacker', 'defender']
		assert color in ['yellow', 'blue']

		self.our_side = side
		self.our_role = role
		self.our_color = color

		self.space = pymunk.Space()
		self.space.gravity = Vec2d(0,0)
		self.space.damping = 0.99

		self.pitch = SimulatedPitch(self.space)
		self.ball = SimulatedBall(self.space, (self.pitch.PITCH_LENGTH*3/4,self.pitch.PITCH_WIDTH/2))

		their_color = 'yellow' if color == 'blue' else 'blue'
		our_color = color

		# Generate our robots using a list, iterating over all possible zones
		for num in range(0,4):

			# Construct a polygon of our zone using the vertices in SimulatedPitch
			zone_poly = Polygon(self.pitch.ZONES[num])

			# Change which side to compare for colour assignment based on zone (even compares to left, odd to right)
			if num % 2 == 0:
				side_compare = 'left'
			else:
				side_compare = 'right'

			# Construct our robot with appropriate colour and position, and add to our robot list
			robot = SimulatedRobot(self.space, zone_poly.center(), our_color if side == side_compare else their_color)
			self.robots.append(robot)

	@property
	def control_robot(self):
		''':returns Returns the robot that we're currently assigned to control, based on initialised role.
		'''
		return self.our_attacker if self.our_role == 'attacker' else self.our_defender

	@property
	def our_attacker(self):
		''':returns: Returns the attacker belonging to our side, as a SimulatedRobot instance
		'''
		return self.robots[2] if self.our_side == 'left' else self.robots[1]

	@property
	def their_attacker(self):
		''':returns: Returns the opposing attacker, as a SimulatedRobot instance
		'''
		return self.robots[1] if self.our_side == 'left' else self.robots[2]

	@property
	def our_defender(self):
		''':returns: Returns the defender belonging to our side, as a SimulatedRobot instance
		'''
		return self.robots[0] if self.our_side == 'left' else self.robots[3]

	@property
	def their_defender(self):
		''':returns: Returns the opposing defender, as a SimulatedRobot instance
		'''
		return self.robots[3] if self.our_side == 'left' else self.robots[0]

	@property
	def width(self):
		'''Retrieves the width of this simulation, which is the Pitch's width. (Narrower dimension).
		'''
		return self.pitch.PITCH_WIDTH

	@property
	def length(self):
		'''Retrieves the length of this simulation, which is the Pitch's length. (Longer dimension).
		'''
		return self.pitch.PITCH_LENGTH

	def draw(self, frame):
		'''Draws the simulation to the given frame.

		:param frame: An OpenCV compatible, numpy image array.
		'''
		# Draw pitch first, then ball, then robots (occludes the ball if underneath robot)
		self.pitch.draw(frame)
		self.ball.draw(frame)

		for robot in self.robots:
			robot.draw(frame)

	@staticmethod
	def get_croppings():
		'''Convenience function; retrieves cropping coordinates for the current simulation. This returns a list of
		polygons in a dictionary, including:

		  * Outline: Corresponding to the INNER_POLY
		  * Zone_[0-3]: Corresponding to the zones from left to right
		  * Zone_0_Goal: Corresponds to y-positions between indents
		  * Zone_3_Goal: Corresponds to y-positions between indents
		'''
		pitch = SimulatedPitch(None)
		croppings = {}
		croppings['outline'] = pitch.INNER_POLY

		for num, zone in enumerate(['Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']):
			croppings[zone] = pitch.ZONES[num]

		croppings['Zone_0_Goal'] = sorted(croppings['Zone_0'], key=lambda x:x[0])[:2]
		croppings['Zone_3_Goal'] = sorted(croppings['Zone_3'], key=lambda x:x[0], reverse=True)[:2]

		return croppings

	def update(self, dt):
		'''Updates the simulation, stepping it forward the given number of seconds.

		:param dt: Time step for simulator, in seconds.
		'''
		# Call update on all our pitch objects first
		for robot in self.robots:
			robot.update(dt)

		self.ball.update(dt)

		# Call update on the simulation
		self.space.step(dt)

if __name__ == '__main__':
	sim = Simulator('left', 'attacker', 'yellow')
	cam = SimulatedCamera(sim)
	robot = SimulatedAction(sim.control_robot)

	while True:
		sim.update(1)
		cv2.imshow('World', cam.get_frame())
		
		c = cv2.waitKey(1) & 0xFF

		if c == ord('t'):
			sim.ball.body.velocity = Vec2d(0,-0.8)
		elif c == ord('g'):
			sim.ball.body.velocity = Vec2d(0,0.8)
		elif c == ord('f'):
			sim.ball.body.velocity = Vec2d(-0.8,0)
		elif c == ord('h'):
			sim.ball.body.velocity = Vec2d(0.8,0)
		elif c == ord('y'):
			robot.open_catcher()
		elif c == ord('u'):
			robot.catch()
		elif c == ord('i'):
			robot.kick()
		elif c == ord('o'):
			robot.turn(100)
		elif c == ord('q'):
			robot.turn(50)
		elif c == ord('w'):
			robot.turn(-50)
		elif c == ord('e'):
			robot.turn(-100)
		elif c == ord('p'):
			robot.stop()
		elif c == ord('z'):
			robot.move(100,0)
		elif c == ord('x'):
			robot.move(100,math.pi)
		elif c == 27:
			break
