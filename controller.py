from cv2 import waitKey
import time

import cv2
import serial
import warnings

from action import Action
from planning.planner import Planner
from planning.tasks import AcquireBall
from vision.vision import Vision, Camera, GUI
import vision.tools as tools
from prediction.Prediction import KalmanBallPredictor, KalmanRobotPredictor
from preprocessing.preprocessing import Preprocessing
from postprocessing.postprocessing import Postprocessing
from planning.models import World


warnings.filterwarnings("ignore", category=DeprecationWarning)


class MyError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Controller:
	'''Main controller for the Robot. Pulls together all modules and executes our Vision, 
	Processing and Planning to move the Robot.
	''' 

	def __init__(self, passing, pitch=0, color='yellow', our_side='left', our_role='attacker', video_port=0, comms=True):
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
		assert pitch in [0, 1]
		assert color in ['yellow', 'blue']
		assert our_side in ['left', 'right']
		assert our_role in ['attacker', 'defender']

		self.pitch = pitch

		# Set up camera for frames
		self.camera = Camera(port=video_port, pitch=self.pitch)

		if comms:
			self.comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
			self.comm.flushInput()
		else:
			self.comm = None

		self.robot = Action(self.comm)
		time.sleep(7)
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
		self.planner = Planner(world=self.world, robot=self.robot, role=our_role, passing=passing)

		# Set up GUI
		self.GUI = GUI(calibration=self.calibration, pitch=self.pitch)
		
		# Set up predictors
		self.ball_predictor = None
		self.robot_predictor = None

		# Set up our cache of commands for the predictors
		self.command_cache = [[0,0]]*8
		self.command = [0,0]

		self.color = color
		self.side = our_side
		self.role = our_role
		self.our_robot = self.world.our_attacker if self.role == 'attacker' else self.world.our_defender
		
		self.task = None

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
		start = time.clock()
		#: Tracker is used for a Planning timer, running Planner only in
		#: certain time intervals
		tracker = time.clock()
		try:
			c = True
			mh = None
			while c != 27:  # the ESC key

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
				if self.task is None:
					self.task = MoveToPoint(self.world, self.robot, self.role, self.our_robot.x, self.world.pitch.height/2)
				if self.ball_predictor is None:
					self.ball_predictor = KalmanBallPredictor(self.world.ball.vector, friction=0)

				if self.robot_predictor is None:
					self.robot_predictor = KalmanRobotPredictor(self.our_robot.vector, friction=-10, acceleration=5)

				#: Run planner only every 5ms.
				if (time.clock() - tracker) > 0.05 and time.clock() - start > 1: 
					
					self.command = self.command_cache.pop(0)
					self.planner.plan()
					if self.robot.move_handle.cmd == 'M':
						dx = math.cos(self.our_robot.vector.angle + self.robot.move_handle.dir - math.pi / 2)*self.robot.move_handle.spd
						dy = math.sin(self.our_robot.vector.angle + self.robot.move_handle.dir - math.pi / 2)*self.robot.move_handle.spd
						last_com = [dx,dy]
					else:
						last_com = [0,0]
					self.command_cache.append(last_com)
					tracker = time.clock()

				ball_doubtful, self.world.ball.vector = self.ball_predictor.predict(self.world, time = 8)
				if regular_positions['ball']:
					regular_positions['ball']['x'] =  self.world.ball.vector.x
					regular_positions['ball']['y'] = self.world._pitch.height -  self.world.ball.vector.y
				self.world.our_attacker.vector = self.robot_predictor.predict(self.command, self.world, time = 8)

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
		except MyError as e:
			print e.value()
		finally:
			if self.robot is not None:
				self.robot.stop()
				self.robot.exit()

			tools.save_colors(self.pitch, self.calibration)

# MAIN
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument('-p', '--pitch', type=int, default=0, help="[0] Main pitch (Default), [1] Secondary pitch")
    parser.add_argument('-s', '--side', default='left',
                        help="Our team's side ['left', 'right'] allowed. [Default: left]")
    parser.add_argument(
        '-r', '--role', default='attacker',
        help="Our controlled robot's role ['attacker', 'defender'] allowed. [Default: attacker]")
    parser.add_argument(
        '-c', '--color', default='yellow', help="The color of our team ['yellow', 'blue'] allowed. [Default: yellow]")
    parser.add_argument(
        "-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")

    parser.add_argument("--passing", dest='passing', help="Signals us as the passing robot.", action="store_true")
    parser.add_argument("--receiving", dest='passing', help="Signals us as the receigin robot.", action="store_false")

    args = parser.parse_args()

    if args.nocomms:
        Controller(args.passing, pitch=args.pitch, color=args.color, our_side=args.side, our_role=args.role, comms=False).run()
    else:
        Controller(args.passing, pitch=args.pitch, color=args.color, our_side=args.side, our_role=args.role).run()
