
import pymunk
from pymunk.constraint import PivotJoint, GearJoint
from pymunk import Vec2d

import numpy as np
import cv2

import math


class SimulatedPitch:
	'''Simulates the Pitch, representing the slanted corners and goals in realistic
	dimensions. Provides means to query the pitch size and draws the Pitch.
	'''

	# Default Physical Parameters for the Pitch

	#: Total scale parameter; all dimensions are scaled by this before usage, control size of pitch using this
	PITCH_SCALE = 0.2

	#: The width (narrower) and length (longer) dimensions
	PITCH_WIDTH, PITCH_LENGTH = 1140, 2118

	#: The full frame of the pitch
	FRAME_POLY = [(0,0),(PITCH_LENGTH,0),(PITCH_LENGTH,PITCH_WIDTH),(0,PITCH_WIDTH)]

	#: The internal area of the pitch, including white border
	INNER_BORDER = [(0,271),(126,0),(1992,0),(PITCH_LENGTH,271),(PITCH_LENGTH,869),(1992,PITCH_WIDTH),(126,PITCH_WIDTH),(0,869)]

	#: The internal area of the pitch, excluding white border
	INNER_POLY = [(26,345),(164,50),(1956,50),(2092,345),(2092,795),(1956,1090),(164,1090),(26,795)]

	#: Our 'stripe' polys, representing white separations
	STRIPES = [[(379,0),(575,0),(575,PITCH_WIDTH),(379,PITCH_WIDTH)], \
			   [(970,0),(1166,0),(1166,PITCH_WIDTH),(970,PITCH_WIDTH)], \
			   [(1559,0),(1755,0),(1755,PITCH_WIDTH),(1559,PITCH_WIDTH)]]

	#: The zone polygons for each zone, moving left to right
	ZONES = [[(0,271),(126,0),(477,0),(477,PITCH_WIDTH),(126,PITCH_WIDTH),(0,869)], \
			 [(477,0),(1068,0),(1068,PITCH_WIDTH),(477,PITCH_WIDTH)], \
			 [(1068,0),(1657,0),(1657,PITCH_WIDTH),(1068,0)], \
			 [(1657,0),(1992,0),(PITCH_LENGTH,271),(PITCH_LENGTH,869),(1992,PITCH_WIDTH),(1657,PITCH_WIDTH)]]

	#: Draw colours for the Pitch
	COLORS = {'frame': (50,50,50), 'white': (230,224,2226), 'green': (74,93,71)}

	# Pymunk Variables

	#: The pymunk space to place the Pitch in
	space = None

	def __init__(self, space):
		'''Initialises the Pitch, placing static lines for the Pitch walls.

		:param space: The space to add our walls to
		'''
		self.space = space
		self._scale_sizes()

		#: Prevent constructing pitch if we've initialised Pitch without Physics space; used for
		#: constructing Pitch and getting croppings
		if space is not None:
			self._construct_pitch()

	def draw(self, frame):
		'''Given a frame, draws the Pitch to it.

		:param frame: A cv-compatible numpy image array.
		'''
		cv2.fillConvexPoly(frame, np.array(self.FRAME_POLY), self.COLORS['frame'])
		cv2.fillConvexPoly(frame, np.array(self.INNER_BORDER), self.COLORS['white'])
		cv2.fillConvexPoly(frame, np.array(self.INNER_POLY), self.COLORS['green'])

		for stripe in self.STRIPES:
			cv2.fillConvexPoly(frame, np.array(stripe), self.COLORS['white'])

	def _scale_sizes(self):
		'''Scales the dimensions of this Pitch to suit the provided scale value.
		'''
		scale_func = lambda (x,y): (int(x*self.PITCH_SCALE), int(y*self.PITCH_SCALE))

		self.PITCH_WIDTH = self.PITCH_SCALE * self.PITCH_WIDTH
		self.PITCH_LENGTH = self.PITCH_SCALE * self.PITCH_LENGTH

		self.FRAME_POLY = map(scale_func, self.FRAME_POLY)
		self.INNER_BORDER = map(scale_func, self.INNER_BORDER)
		self.INNER_POLY = map(scale_func, self.INNER_POLY)
		self.ZONES = map(lambda list: map(scale_func, list), self.ZONES)
		self.STRIPES = map(lambda list: map(scale_func, list), self.STRIPES)

	def _construct_pitch(self):
		'''Constructs our pitch; placing static walls around the exterior.
		'''
		static_body = pymunk.Body()

		static_lines = []
		for num, point in enumerate(self.INNER_BORDER):

			# Retrieve next point in the poly, looping back to beginning when at the end
			next_point = self.INNER_BORDER[(num+1) % len(self.INNER_BORDER)]

			segment = pymunk.Segment(static_body, Vec2d(point[0],point[1]), Vec2d(next_point[0], next_point[1]), 1)
			segment.friction = 0.5
			segment.elasticity = 1

			static_lines.append(segment)

		self.space.add(static_lines)

class SimulatedBall:
	'''Represents our Ball model. Very simple simulation class.
	'''

	# Default Physical Parameters for the Ball

	#: Mass of the ball; grams.
	BALL_MASS = 0.05

	#: Radius of the ball; cm
	BALL_RADIUS = 5

	# Pymunk Variables

	#: Group this Ball belongs to
	BALL_GROUP = 5
	BALL_COLLIDE = 25

	BALL_MOVE_SPEED = 0.02

	#: The body representing this Ball
	body = None

	#: The pymunk space holding this ball
	space = None

	def __init__(self, space, center, scale=1):
		'''Alike SimulatedRobot, accepts a physics space and position and
		constructs our ball at this position. Scale is an optional parameter 
		to tune the sizing of the objects.

		:param space: The physics space to add the ball to
		:param center: The central position of the ball
		:param scale: The scaling parameter.
		'''
		self.space = space
		self._construct_ball(center, scale)

	def stop(self):
		'''Resets all forces on the ball and sets it's velocity to zero.
		'''
		self.body.reset_forces()
		self.body.angular_velocity = 0
		self.body.velocity = Vec2d(0,0)

	def update(self, dt):
		'''This function is executed every time the world is updated, before stepping the world by
		the given dt value.

		:param dt: The time value, in seconds, to step the world.
		'''
		pass

	def draw(self, frame):
		'''Receiving an image frame, this function draws the ball it.

		:param frame: The image frame to draw to. A cv-compatible numpy array.
		'''
		for shape in self.body.shapes:
			cv2.circle(frame, (int(self.body.position.x), int(self.body.position.y)), int(shape.radius), (0,0,255), -1)

	def _construct_ball(self, center, scale):
		'''Constructs a new simulated ball at the given position.

		:param center: The central position of the ball
		:param scale: The scaling parameter
		'''
		moment = pymunk.moment_for_circle(self.BALL_MASS, 0, self.BALL_RADIUS, (0,0))
		
		self.body = pymunk.Body(self.BALL_MASS, moment)
		self.body.position = Vec2d(center[0], center[1])
		
		self.space.add(self.body)

		shape = pymunk.Circle(self.body, self.BALL_RADIUS, (0,0))
		shape.group = self.BALL_GROUP
		shape.layers = SimulatedRobot.ROBOT_LAYER | SimulatedRobot.CATCHER_LAYER | SimulatedRobot.KICKER_LAYER
		shape.collision_type = self.BALL_COLLIDE
		shape.elasticity = 1

		self.space.add(shape)

class SimulatedRobot:
	'''Represents our Robot model, and it's simulation specific features in pymunk.
	'''

	# Drawing parameters
	COLORS = {'plate': (0,255,0), 'dot': (0,0,0), 'yellow': (0, 255, 255), 'blue': (255, 0, 0), 'legs': (60,60,60)}

	# Default Physical Parameters for the Robot

	#: Plate dimensions
	PLATE_WIDTH, PLATE_LENGTH = 25, 30

	#: Plate (dot) Dimensions
	DOT_RADIUS, DOT_OFFSET = 4, 8

	#: Plate (i) Dimensions
	I_WIDTH, I_LENGTH, I_OFFSET = 10, 6, 5
	
	#: Mass of the robot, catcher, and kicker; grams.
	ROBOT_MASS, CATCHER_MASS, KICKER_MASS = 50, 10, 10

	#: Tracks the length of the catcher and kicker, in cm.
	CATCHER_LENGTH, KICKER_LENGTH = 15, 10
	
	#: Maximum forces
	CATCHER_MAX_FORCE, KICKER_MAX_FORCE = 1, 5

	#: Central solid circle radius, mimicking internals
	CENTRAL_RADIUS = 10

	#: Individual wheel 'leg' length, cm.
	LEG_LENGTH, LEG_WIDTH = 25, 15

	#: Angles of each leg, adding a new angle will result in another leg, radians.
	LEG_ANGLES = [math.pi*1/4, math.pi*3/4, math.pi*5/4, math.pi*7/4]

	#: Template vertices for a leg, at 0 radians.
	LEG_VERTICES = [(-LEG_WIDTH/2, 0), (LEG_WIDTH/2, 0), (LEG_WIDTH/2, LEG_LENGTH), (-LEG_WIDTH/2, LEG_LENGTH)]

	# Pymunk Variables

	#: Catcher speed, defined in terms of cm/second travelled.
	KICKER_SPEED, CATCHER_SPEED = 1, 3

	#: Multiplier for additional kicker power, used in :func:`SimulatedRobot.ball_kicked`
	KICKER_MULTIPLIER = 0.15

	#: The base power for the motors; is multiplied by seconds and speed to receive final vlaue.
	MOVE_POWER, TURN_POWER = 0.04, 0.0004

	#: Ensure all shapes for the Robot are kept in the same group to prevent collisions
	ROBOT_GROUP, ROBOT_LAYER = 10, 1

	#: Ensure kicker/catcher don't collide with Robot; not in same layer
	KICKER_GROUP, KICKER_LAYER, CATCHER_GROUP, CATCHER_LAYER = 3, 2, 4, 2

	#: Collision types for each of our shapes
	KICKER_COLLIDE, CATCHER_COLLIDE, ROBOT_COLLIDE = 10, 15, 20

	#: Keep track of the Physics space for reference later
	space = None

	#: The pymunk body representing our robot
	body = None

	#: The pymunk body representing our catcher and kicker
	catcher, kicker = None, None

	#: The key, controllable, joints (distance) constraining our kicker/catcher to the Robot
	kicker_joint, catcher_joint = None, None

	#: Stores the impulse points to apply motor speeds to as 4-tuples (x,y,fx,fy)
	impulse_points = []

	def __init__(self, space, center, color='yellow', scale=1):
		'''Constructs a new SimulatedRobot, centered on the given position. Allows a scale
		value to modify the resultant size of the Robot.

		:param space: The pymunk physics Space to add our creation to
		:param center: The center position of the Robot in the world.
		:param color: The colour of our robot. Only matters for the 'i' on the plate
		:param scale: Multiplier for all dimensions, < 1 results in smaller robot, > 1 larger.
		'''
		self.space = space

		#: Our robot's colour, used as index into COLORS 
		self.our_color = color

		self._construct_robot(center, scale)
	
		#: Catcher and Kicker states are one of: open, closed, opening, closing
		self.catcher_state, self.kicker_state = 'closed', 'closed'
		
		#: Stores movement, and rotation action handles
		self.move_handle = None
		self.kick_handle = None
		self.catch_handle = None

		# Add collision handler between our Robot and the ball
		space.add_collision_handler(SimulatedRobot.KICKER_COLLIDE, SimulatedBall.BALL_COLLIDE, separate=self._ball_kicked)

	def _ball_kicked(self, space, arbiter):
		'''Handles a collision between the ball and the kicker, after they've separated. Used to artificially increase the
		impulse applied to the ball during kicking.
		'''

		# Ball will always be the second shape, as defined in collision handler order
		ball = arbiter.shapes[1].body

		# Retrieve velocity of ball
		velocity = ball.velocity
		velocity = Vec2d(velocity[0] * self.KICKER_MULTIPLIER, velocity[1] * self.KICKER_MULTIPLIER)

		# Reapply velocity
		ball.apply_impulse(velocity, (0,0))

	def draw(self, frame):
		'''Given a frame, draws this robot at it's position.

		:param frame: An opencv compatible numpy image array.
		'''

		# Draw Robot Shapes
		for shape in self.body.shapes:
			if isinstance(shape, pymunk.Poly):
				vertices = shape.get_vertices()
				vertices = np.array(list(map(lambda x: (int(x[0]), int(x[1])), vertices)))
				cv2.fillConvexPoly(frame, vertices, self.COLORS['legs'])
			elif isinstance(shape, pymunk.Circle):
				cv2.circle(frame, (int(self.body.position[0] + shape.offset[0]), \
								   int(self.body.position[1] + shape.offset[1])), \
								   int(shape.radius), self.COLORS['legs'], -1)
			else:
				raise TypeError('Error: Shape in Robot body shapes not expected!')

		# Draw Catcher
		for shape in self.catcher.shapes:
			vertices = shape.get_vertices()
			vertices = np.array(list(map(lambda x: (int(x[0]), int(x[1])), vertices)))
			cv2.fillConvexPoly(frame, vertices, (0,0,255) if self.catcher_state == 'closed' else (255,0,0))	

		# Draw Kicker
		for shape in self.kicker.shapes:
			vertices = shape.get_vertices()
			vertices = np.array(list(map(lambda x: (int(x[0]), int(x[1])), vertices)))
			cv2.fillConvexPoly(frame, vertices, (0,255,255) if self.kicker_state == 'closed' else (255,255,0))

		# Draw Impulse Points
		for point in self._get_impulse_points():
			cv2.circle(frame, (int(point[0]+self.body.position.x), int(point[1]+self.body.position.y)), 2, (0,125,0), -1)

		# Draw plate
		cv2.fillConvexPoly(frame, np.array(self._get_plate_poly()), self.COLORS['plate'])
		cv2.fillConvexPoly(frame, np.array(self._get_i_poly()), self.COLORS[self.our_color])
		cv2.circle(frame, self._get_dot_center(), self.DOT_RADIUS, self.COLORS['dot'], -1)

	def update(self, dt):
		'''This function is executed every time the world is updated, before stepping the world by
		the given dt value.

		:param dt: The time value, in seconds, to step the world.
		'''
		self._update_catcher(dt)
		self._update_kicker(dt)
		self._update_movement(dt)

	def _update_movement(self, dt):
		'''Updates the Robot's movement based upon the supplied motor speeds. Simplifies the holonomics down to
		far simpler cases.
		'''

		if self.move_handle is None:
			self.body.angular_velocity = 0
			self.body.velocity = Vec2d(0,0)
			return

		self.move_handle._onRunning()

		#: We have a turning command
		if self.move_handle.cmd == 'S':
			#: Set our move handle as completed if it's a stop
			self.move_handle._onComplete()

			self.body.angular_velocity = 0
			self.body.velocity = Vec2d(0,0)
		
		elif self.move_handle.cmd == 'T':

			target = self.move_handle.dir
			our = self.body.angle

			acw_dist = (our-target+math.pi*2) if (our-target) < 0 else our-target
			cw_dist = (target-our + math.pi*2) if (target-our) < 0 else target-our

			dist = min(acw_dist, cw_dist)
			multiplier = 1 if acw_dist > cw_dist else -1

			if dist < 0.03:
				self.body.angular_velocity = 0.0
				self.move_handle._onComplete()
				self.move_handle = None
			elif dist < 0.5:
				self.body.angular_velocity = multiplier*90*self.TURN_POWER
			elif dist < 1.5:
				self.body.angular_velocity = multiplier*95*self.TURN_POWER
			else:
				self.body.angular_velocity = multiplier*100*self.TURN_POWER

		elif self.move_handle.cmd == 'M':
			#: Set our move handle as completed if it's just a move
			self.move_handle._onComplete()

			dir = self.move_handle.dir + self.body.angle
			speed = self.move_handle.spd*self.MOVE_POWER

			self.body.angular_velocity = 0
			self.body.velocity = Vec2d(speed*math.cos(dir), speed*math.sin(dir))

	def _update_catcher(self, dt):
		'''Updates the catcher, moving it further closed / open based upon our state and the given
		time value that is about to pass.

		:param dt: The change in time given
		'''
		if self.catcher_state == 'closed' or self.catcher_state == 'open': 
			return

		self.catch_handle._onRunning()

		# Positive will move the catcher away from the robot
		change = self.CATCHER_SPEED if self.catcher_state == 'opening' else -self.CATCHER_SPEED

		# Adjust joint positioning
		self.catcher_joint.distance = max(2, min(self.CATCHER_LENGTH, self.catcher_joint.distance + change))

		# Check for termination states
		if self.catcher_joint.distance == 2 and self.catcher_state == 'closing':
			self.catcher_state = 'closed'
			self._change_catcher_group(self.CATCHER_GROUP)
			self.catch_handle._onComplete()
		elif self.catcher_joint.distance == self.CATCHER_LENGTH and self.catcher_state == 'opening':
			self.catcher_state = 'open' 
			self._change_catcher_group(SimulatedBall.BALL_GROUP)
			self.catch_handle._onComplete()

	def _update_kicker(self, dt):
		'''Updates the kicker, moving it further open / closed based upon our state and the given time
		value that is about to pass.

		:param dt: The change in time given
		'''
		if self.kicker_state == 'closed':
			return

		self.kick_handle._onRunning()

		# Positive will move the kicker away from the robot
		change = self.KICKER_SPEED if self.kicker_state == 'opening' else -self.KICKER_SPEED

		# Adjust joint positioning
		self.kicker_joint.distance = max(0, min(self.KICKER_LENGTH, self.kicker_joint.distance + change))

		# Change kicker status based on proximity to center of robot
		if self.kicker_joint.distance >= 10:
			self._change_kicker_group(SimulatedBall.BALL_GROUP)
		else:
			self._change_kicker_group(self.KICKER_GROUP)

		# Check for termination states
		if self.kicker_joint.distance == 0 and self.kicker_state == 'closing':
			self.kicker_state = 'closed'
			self.kick_handle._onComplete()
		elif self.kicker_joint.distance == self.KICKER_LENGTH and self.kicker_state == 'opening':
			self.kicker_state = 'closing'

	def kick(self, handle):
		'''Attempts to kick, performs no check to determine if this should occur or not.
		'''
		if self.kick_handle != None:
			self.kick_handle._onNextCommand()

		self.kicker_state = 'opening'
		self.kick_handle = handle

	def open_catcher(self, handle):
		'''Attempts to open the catcher. Replaces current catcher handle with this one.
		'''
		if self.catch_handle != None:
			self.catch_handle._onNextCommand()

		self.catcher_state = 'opening'
		self.catch_handle = handle

	def close_catcher(self, handle):
		'''Attempts to close the catcher, performs no check to determine if this should occur or not.
		'''
		if self.catch_handle != None:
			self.catch_handle._onNextCommand()

		self.catcher_state = 'closing'
		self.catch_handle = handle

	def turn(self, handle):
		'''Simulates a rotation request to the Robot.

		:param speed: The power of this turn, in the range [-100,100]
		'''
		if self.move_handle != None:
			self.move_handle._onNextCommand()

		self.move_handle = handle

	def move(self, handle):
		'''Simulates a movement request to the Robot to travel in the given angle with the supplied speed.

		:param speed: The speed of this movement.
		:param angle: The direction of travel, relevant to the robot's x-axis, in radians.
		'''

		#: Ensure we set the previous movement command as stopped
		if self.move_handle != None:
			self.move_handle._onNextCommand()

		#: Assign our new movement handle
		self.move_handle = handle

	def run_motors(self, speeds):
		'''Sends a sequence of run speeds to the motors of this Robot.

		.. note::

		   These speeds are remembered, so will continue to run until told to stop with :func:`SimulatedRobot.stop`

		:param speeds: A sequence of speeds to assign to the motors, should be equal to the length of LEG_ANGLES.
		'''
		raise NotImplemented('Error: run_motors hasn''t been implemented!')		

	def stop(self, handle):
		'''Sends the Stop request to the robot, indicating movement should cease.
		'''
		if self.move_handle != None:
			self.move_handle._onNextCommand()

		self.move_handle = handle

	def _to_world(self, point):
		'''Converts the given point to world coordinates by translating the center of the robot.
		'''
		return (point[0] + self.body.position.x, point[1] + self.body.position.y)

	def _get_plate_poly(self):
		'''Retrieves a polygon representing the plate on this robot using the class constants.
		'''
		points = [(-self.PLATE_WIDTH/2, -self.PLATE_LENGTH/2),(-self.PLATE_WIDTH/2,self.PLATE_LENGTH/2), \
				  (self.PLATE_WIDTH/2, self.PLATE_LENGTH/2),(self.PLATE_WIDTH/2, -self.PLATE_LENGTH/2)]
		points = map(lambda point: self._rotate_point(point, -self.body.angle), points)
		points = map(lambda point: self._to_world(point), points)
		points = map(lambda (x,y): (int(x), int(y)), points)

		return points

	def _get_i_poly(self):
		'''Retrieves a polygon representing the 'i' symbol on this robot using the class constants.
		'''
		points = [(-self.I_LENGTH/2, -self.I_WIDTH/2), (-self.I_LENGTH/2, self.I_WIDTH/2), \
				  (self.I_LENGTH/2, self.I_WIDTH/2), (self.I_LENGTH/2, -self.I_WIDTH/2)]

		points = map(lambda (x,y): self._rotate_point((x, y+self.I_OFFSET), -self.body.angle), points)
		points = map(lambda point: self._to_world(point), points)
		points = map(lambda (x,y): (int(x), int(y)), points)

		return points

	def _get_dot_center(self):
		'''Retrieves the central position of the dot on our plate, in world position.
		'''
		point = self._to_world(self._rotate_point((0,-self.DOT_OFFSET), -self.body.angle))

		return (int(point[0]), int(point[1]))

	def _get_impulse_points(self):
		'''Calculates and returns the (x,y) positions of our impulse points based on the Robot's
		current angle.

		:returns: A list containing (x,y,fx,fy) values, with x,y position of impulse and fx,fy trig coefficients.
		'''
		# Calculate where our legs are angled now, and where the endpoints are
		angles = map(lambda angle: -self.body.angle + angle, self.LEG_ANGLES)
		points = map(lambda angle: (angle, self._rotate_point((0,self.LEG_LENGTH), angle)), angles)

		# Calculate and return the forces for each of these points
		impulses = map(lambda (theta,(x,y)): (x,y,math.cos(theta+math.pi/2),math.sin(theta+math.pi/2)), points)

		return impulses

	def _construct_robot(self, center, scale):
		'''High-level entrypoint for constructing the physical robot in the world.

		:param center: The center position of the Robot in the world.
		:param scale: Multiplier for the dimensions.
		'''
		vertices = map(lambda (x,y): (x*scale, y*scale), SimulatedRobot.LEG_VERTICES)

		# Calculate moment for full robot
		moment = pymunk.moment_for_poly(SimulatedRobot.ROBOT_MASS, vertices)
		moment = moment * len(self.LEG_ANGLES)

		self.body = pymunk.Body(SimulatedRobot.ROBOT_MASS, moment)
		self.body.position = Vec2d(center[0], center[1])
		self.body.angle = 0

		self.space.add(self.body)

		# Construct our leg shapes for the Robot
		leg_polys = map(lambda angle: self._get_leg_polygon(vertices, angle), self.LEG_ANGLES)

		for leg in leg_polys:
			shape = pymunk.Poly(self.body, leg, (0,0))
			shape.group = SimulatedRobot.ROBOT_GROUP
			shape.layers = 1
			shape.collision_type = SimulatedRobot.ROBOT_COLLIDE
			self.space.add(shape)

		# Define solid central circle to prevent ball entering interior
		shape = pymunk.Circle(self.body, self.CENTRAL_RADIUS)
		shape.group = SimulatedRobot.ROBOT_GROUP
		self.space.add(shape)

		# Flatten our list of leg vertices into single list
		all_points = [item for points in leg_polys for item in points]

		# Sort by x-value to retrieve two most extreme x-positions
		points = sorted(all_points, key=lambda x: x[0], reverse=True)[:2]
		points = sorted(points, key=lambda x: x[1])

		# Finally, build our catcher then kicker
		self._construct_catcher(points[0], points[1])
		self._construct_kicker()

	def _construct_kicker(self):
		'''Constructs a kicker body/shape and joints.

		.. note::

		   The Kicker is, by default, the same width as the leg width.
		'''
		moment = pymunk.moment_for_poly(SimulatedRobot.KICKER_MASS, [(0, self.LEG_WIDTH/2), (0, -self.LEG_WIDTH/2)])

		self.kicker = pymunk.Body(SimulatedRobot.KICKER_MASS, moment)
		self.kicker.position = Vec2d(self.body.position.x+5, self.body.position.y)

		kicker_shape = pymunk.Poly(self.kicker, [(0, self.LEG_WIDTH/2), (0, -self.LEG_WIDTH/2)])
		kicker_shape.group = self.KICKER_GROUP
		kicker_shape.layers = self.KICKER_LAYER
		kicker_shape.collision_type = self.KICKER_COLLIDE

		self.space.add(self.kicker, kicker_shape)

		# Constrain kicker to same angle as our robot
		kick_gear = pymunk.constraint.GearJoint(self.kicker, self.body, 0, 1)

		# Constrain kicker to x-axis movement
		kick_joint = pymunk.constraint.GrooveJoint(self.body, self.kicker, (5,0), (5+SimulatedRobot.KICKER_LENGTH,0), (0,0))

		self.kicker_joint = pymunk.constraint.PinJoint(self.body, self.kicker, (5,0), (0,0))
		self.kicker_joint.max_force = SimulatedRobot.KICKER_MAX_FORCE
		self.kicker_joint.distance = 2

		self.space.add(kick_gear, kick_joint, self.kicker_joint)

	def _construct_catcher(self, upper_point, lower_point):
		'''Constructs the catcher body/shape and joints and adds to the
		physics space.

		:param upper_point: The rightmost point on the upper leg
		:param lower_point: The rightmost point on the lower leg
		'''
		moment = pymunk.moment_for_poly(SimulatedRobot.CATCHER_MASS, [upper_point, lower_point])
		
		self.catcher = pymunk.Body(SimulatedRobot.CATCHER_MASS, moment)
		self.catcher.position = Vec2d(self.body.position.x + upper_point[0], self.body.position.y)

		# Move points to be relative to catcher's origin
		r_upper = (0, upper_point[1])
		r_lower = (0, lower_point[1])

		catcher_shape = pymunk.Poly(self.catcher, [r_lower, r_upper])
		catcher_shape.group = self.CATCHER_GROUP
		catcher_shape.layer = self.CATCHER_LAYER
		catcher_shape.collision_type = self.CATCHER_COLLIDE

		self.space.add(self.catcher, catcher_shape)

		# Constrain catcher to the same angle as our robot
		catch_gear = pymunk.constraint.GearJoint(self.catcher, self.body, 0, 1)

		# Constrain catcher to x-axis movement
		catch_joint = pymunk.constraint.GrooveJoint(self.body, self.catcher, (upper_point[0],0), \
			(upper_point[0]+SimulatedRobot.CATCHER_LENGTH,0), (r_upper[0], 0))

		self.catcher_joint = pymunk.constraint.PinJoint(self.body, self.catcher, (upper_point[0],0), (0,0))
		self.catcher_joint.max_force = SimulatedRobot.CATCHER_MAX_FORCE
		self.catcher_joint.distance = 2
		
		self.space.add(catch_gear, catch_joint, self.catcher_joint)

	def _change_catcher_group(self, group):
		'''Alters the group the shapes of the catcher belong to. Used to adjust the
		Catcher to be clipping or no clipping based on it's state.

		:param group: The new group number to alter to.
		'''
		for shape in self.catcher.shapes:
			shape.group = group

	def _change_kicker_group(self, group):
		'''Alters the group the shapes of the kicker belong to. Used to adjust the
		Kicker to be clipping or no clipping based on it's state.

		:param group: The new group number to alter to.
		'''
		for shape in self.kicker.shapes:
			shape.group = group

	@staticmethod
	def _get_leg_polygon(leg_points, angle):
		'''Retrieves a new leg polygon rotated to the given angle.
		'''
		return map(lambda point: SimulatedRobot._rotate_point(point, angle), leg_points)

	@staticmethod
	def _rotate_point(point, angle):
		'''Rotates the given point by the given angle, relative to (0,0)
		'''
		sx = point[0]
		sy = point[1]

		hyp = math.sqrt((sx**2) + (sy**2))

		src_angle = math.atan2(sy, sx)
		dst_angle = src_angle + angle

		dx = hyp*math.sin(dst_angle)
		dy = hyp*math.cos(dst_angle)

		return (dx, dy)
