from action import Action
from planning.tasks import AcquireBall, MoveToPoint
from planning.planner import Planner
from planning.models import World
from vision.vision import Vision, Camera, GUI
import vision.tools as tools
from preprocessing.preprocessing import Preprocessing
from postprocessing.postprocessing import Postprocessing
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera
from cv2 import waitKey
import cv2
import math
import serial
import warnings
import time
from planning.models import World
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera

warnings.filterwarnings("ignore", category=DeprecationWarning)

class Controller:
	'''Main controller for the Robot. Pulls together all modules and executes our Vision, 
	Processing and Planning to move the Robot.
	''' 

	def __init__(self, pitch=0, color='yellow', our_side='left', our_role='attacker', video_port=0, comms=True, sim=False):
		'''Initialises the main controller. Constructs all necessary elements; doesn't start until
		run is called.

		:param pitch: Pitch number to play on. [0: Main pitch, 1: Secondary pitch]
		:param color: Which color plate to run as; only impact is aesthetic.
		:param our_side: Designates our side, left represents left of the camera frame, core \
					distinguishing feature for our / their functions.
		:param our_role: Designates the role we're playing as, one of ['attacker', 'defender'].
		:param video_port: Which serial port to use for the camera, an integer, usually 0 for /dev/video0.
		:param comms: True indicates we should attempt communication with the Arduino, False ignores communications.
		:param sim: True indicates we should run the simulated pitch rather than the real pitch.
		'''
		# Pitch is 2 by default for Simulator
		if not sim:
			assert pitch in [0, 1]
		else:
			pitch = 2

		assert color in ['yellow', 'blue']
		assert our_side in ['left', 'right']
		assert our_role in ['attacker', 'defender']

		self.pitch = pitch

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
		self.calibration = tools.get_colors(self.pitch)
		self.vision = Vision(
			pitch=pitch, color=color, our_side=our_side,
			frame_shape=frame.shape, frame_center=center_point,
			calibration=self.calibration)

		# Set up postprocessing for vision
		self.postprocessing = Postprocessing()

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

	def run(self):
		'''Main loop of the Controller. Executes a continuous loop doing the following:
		
		* Retrieve Camera frame, performing barrel distortion fix.
		* Perform preprocessing fixes to the frame
		* Pass frame to Vision module to locate positions of objects of interest
		* Perform postprocessing analysis on the positions
		* Update our World state with new positions
		* Contact our Planner to find out next tasks
		* Redraw the GUI with updated information.
		'''
		counter = 1L

		#: Timer is used for FPS counting
		timer = time.clock()

		#: Tracker is used for a Planning timer, running Planner only in
		#: certain time intervals
		tracker = time.clock()

		c = True

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

			#: Run planner only every 5ms.
			if (time.clock() - tracker) > 0.05: 
				self.planner.plan()
				tracker = time.clock()

			# Information about the grabbers from the world
			grabbers = {
				'our_defender': self.world.our_defender.catcher_area,
				'our_attacker': self.world.our_attacker.catcher_area
			}

			# Information about states
			attackerState = (self.planner._current_state, self.planner._current_state)
			defenderState = (self.planner._current_state, self.planner._current_state)

			attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
			defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

			# Use 'y', 'b', 'r' to change color.
			c = waitKey(2) & 0xFF

			actions = []
			fps = float(counter) / (time.clock() - timer)
			# Draw vision content and actions
			
			self.GUI.draw(
				frame, model_positions, actions, regular_positions, fps, attackerState,
				defenderState, attacker_actions, defender_actions, grabbers,
				our_color=self.color, our_side=self.side, key=c, preprocess=pre_options)
			counter += 1

		if self.robot is not None:
			self.robot.stop()

		tools.save_colors(self.pitch, self.calibration)

# MAIN
if __name__ == '__main__':
	import argparse
	parser = argparse.ArgumentParser()

	parser.add_argument('-p', '--pitch', type=int, default=0, help="[0] Main pitch (Default), [1] Secondary pitch")
	parser.add_argument('-s', '--side', default='left', help="Our team's side ['left', 'right'] allowed. [Default: left]")
	parser.add_argument(
		'-r', '--role', default='attacker', help="Our controlled robot's role ['attacker', 'defender'] allowed. [Default: attacker]")
	parser.add_argument(
		'-c', '--color', default='yellow', help="The color of our team ['yellow', 'blue'] allowed. [Default: yellow]")
	parser.add_argument(
		"-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")
	parser.add_argument("--sim", help="Enables simulation.", action="store_true")

	args = parser.parse_args()

	if args.nocomms:
		c = Controller(
			pitch=args.pitch, color=args.color, our_side=args.side, our_role=args.role, comms=False, sim=args.sim).run()
	else:
		c = Controller(
			pitch=args.pitch, color=args.color, our_side=args.side, our_role=args.role, sim=args.sim).run()
