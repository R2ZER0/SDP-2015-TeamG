import pdb
from action import Action
from planning.tasks import AcquireBall
from vision.vision import Vision, Camera, GUI
from planning.planner import Planner
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
import vision.tools as tools
from cv2 import waitKey
import math
import cv2
import serial
import warnings
import time
from planning.models import World
import Prediction as pre
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera
import action
import sys

warnings.filterwarnings("ignore", category=DeprecationWarning)


class Controller:
	"""
	Primary source of robot control. Ties vision and planning together.
	"""

	def __init__(self, pitch, color, our_side, our_role, video_port=0, comm_port='/dev/ttyUSB0', comms=1, sim=True):
		"""
		Entry point for the SDP system.

		Params:
			[int] video_port                port number for the camera
			[string] comm_port              port number for the arduino
			[int] pitch                     0 - main pitch, 1 - secondary pitch
			[string] our_side               the side we're on - 'left' or 'right'
			*[int] port                     The camera port to take the feed from
			*[Robot_Controller] attacker    Robot controller object - Attacker Robot has a RED
											power wire
			*[Robot_Controller] defender    Robot controller object - Defender Robot has a YELLOW
											power wire
		"""
		assert pitch in [0, 1, 2]
		assert color in ['yellow', 'blue']
		assert our_side in ['left', 'right']
		assert our_role in ['attacker', 'defender']

		self.pitch = pitch

		#pdb.set_trace()

		# Set up camera for frames
		if sim:
			self.sim = sim
			self.simulator = Simulator(our_role)
			self.robot = SimulatedAction(self.simulator)
			self.camera = SimulatedCamera()
		else:
			self.sim = None
			self.camera = Camera(port=video_port, pitch=self.pitch)

			if comms:
				self.comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
			else:
				self.comm = None

			self.robot = Action(self.comm)
		
		frame = self.camera.get_frame()
		center_point = self.camera.get_adjusted_center(frame)

		# Set up vision
		self.calibration = tools.get_colors(pitch)
		self.vision = Vision(
			pitch=pitch, color=color, our_side=our_side,
			frame_shape=frame.shape, frame_center=center_point,
			calibration=self.calibration)

		# Set up postprocessing for vision
		self.postprocessing = Postprocessing()

		# Set up ball prediction
		#self.predictor = pre.PhysicsBallPredictor()

		# Set up world
		self.world = World(our_side, pitch)

		# Set up main planner
		self.planner = Planner(world=self.world, robot=self.robot, role=our_role)

		# Set up GUI
		self.GUI = GUI(calibration=self.calibration, pitch=self.pitch)

		self.color = color
		self.side = our_side
		self.role = our_role

		self.preprocessing = Preprocessing()
		
		self.comms = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
		#comm = None
		self.real_robot = action.Action(self.comm)

		self.reqAngle = 3.14
		self.angleReached = False
		self.initialised = False
		self.startAngle = 0
		self.finalAngle = 0
		self.done = False
		self.check = True
		self.timestarted = False
		self.timer = 0

	def wow(self):
		"""
		Ready your sword, here be dragons.
		"""
		counter = 1L
		timer = time.clock()
		#try:
		c = True

		task = AcquireBall(self.world,self.robot,'attacker')

		while c != 27:  # the ESC key

			if self.sim:
				world = self.simulator.update(0.25)
				self.camera.draw(world)

			frame = self.camera.get_frame()
			pre_options = self.preprocessing.options
			# Apply preprocessing methods toggled in the UI
			preprocessed = self.preprocessing.run(frame, pre_options)
			frame = preprocessed['frame']
			if 'background_sub' in preprocessed:
				cv2.imshow('bg sub', preprocessed['background_sub'])
			# Find object positions
			# model_positions have their y coordinate inverted

			model_positions, regular_positions = self.vision.locate(frame)
			model_positions = self.postprocessing.analyze(model_positions)

			# Update world state
			self.world.update_positions(model_positions)

			# Predict ball and update position
			#certain, point = predictor.predict(self.world)
			#if not certain:
			#	ball = self.world.ball
			#	self.world.ball.vector = Vector(point[0], point[1], ball.angle, ball.velocity)

			# TEST TASKS
			self.planner.plan()

			# Information about the grabbers from the world
			grabbers = {
				'our_defender': self.world.our_defender.catcher_area,
				'our_attacker': self.world.our_attacker.catcher_area
			}

			# Information about states
			attackerState = (self.planner._current_state, self.planner._current_state)
			defenderState = (self.planner._current_state, self.planner._current_state)
			
			#########
			# Tests #
			#########

			if not self.done:
				if not self.initialised:
					if self.role == "att":
						self.robot = self.world.our_attacker
					else:
						self.robot = self.world.our_defender

					self.startAngle = self.robot.angle
					self.finalAngle = self.startAngle + self.reqAngle
					self.finalAngle = self.finalAngle % 2*math.pi

				if abs(self.robot.angle - self.finalAngle) < math.pi/16:
					self.real_robot.stop()
					self.done = True
				else:
					self.real_robot.turn(-100)
			else:
				self.real_robot.stop()
				if not self.timestarted:
					self.timestarted = True
					self.timer = 20
				if self.timestarted:
					self.timer -= 1

				if not self.check and self.timer <= 0:
					self.stop_angle = self.robot.angle
					self.check = True
					self.error = abs(self.finalAngle-self.stop_angle)
					print "Angle Error: " + str(self.error) + " radians"
					sys.exit()

			#####
			
			attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
			defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

			# Use 'y', 'b', 'r' to change color.
			c = waitKey(2) & 0xFF

			if c == ord('g'):
				self.simulator.move_ball(math.pi/2)
			elif c == ord('t'):
				self.simulator.move_ball(-math.pi/2)
			elif c == ord('f'):
				self.simulator.move_ball(math.pi)
			elif c == ord('h'):
				self.simulator.move_ball(0)

			actions = []
			fps = float(counter) / (time.clock() - timer)
			# Draw vision content and actions
			
			self.GUI.draw(
				frame, model_positions, actions, regular_positions, fps, attackerState,
				defenderState, attacker_actions, defender_actions, grabbers,
				our_color=self.color, our_side=self.side, key=c, preprocess=pre_options)
			counter += 1

		#except:
		#	if self.robot is not None:
		#		self.robot.stop()

		#finally:
			# Write the new calibrations to a file.
			#tools.save_colors(self.pitch, self.calibration)
			#if self.robot is not None:
			#	self.robot.stop()
		tools.save_colors(self.pitch, self.calibration)


if __name__ == '__main__':
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument("pitch", help="[0] Main pitch, [1] Secondary pitch")
	parser.add_argument("side", help="The side of our defender ['left', 'right'] allowed.")
	parser.add_argument("role", help="The role of our robot ['att', 'def'] allowed.")
	parser.add_argument("color", help="The color of our team ['yellow', 'blue'] allowed.")
	parser.add_argument(
		"-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")
	parser.add_argument(
		"-s", "--sim", help="Enables simulation.", action="store_true")

	args = parser.parse_args()
	if args.nocomms:
		c = Controller(
			pitch=int(args.pitch), color=args.color, our_side=args.side, our_role=args.role, comms=0, sim=args.sim).wow()
	else:
		c = Controller(
			pitch=int(args.pitch), color=args.color, our_side=args.side, our_role=args.role, sim=args.sim).wow()
