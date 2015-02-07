import pdb
from action import Action
from planning.tasks import AcquireBall
from vision.vision import Vision, Camera, GUI
from planning.planner import Planner
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
import vision.tools as tools
from cv2 import waitKey
import cv2
import serial
import warnings
import time
from planning.models import World
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera

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
		assert our_role in ['att', 'def']

		self.pitch = pitch

		pdb.set_trace()

		# Set up camera for frames
		if sim:
			self.sim = sim
			self.simulator = Simulator()
			self.robot = SimulatedAction(self.simulator)
			self.camera = SimulatedCamera()
		else:
			self.sim = None
			self.camera = Camera(port=video_port, pitch=self.pitch)

			self.comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
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

		# Set up world
		self.world = World(our_side, pitch)

		# Set up main planner
		#self.planner = Planner(our_side=our_side, pitch_num=self.pitch, world=self.world)

		# Set up GUI
		self.GUI = GUI(calibration=self.calibration, pitch=self.pitch)

		self.color = color
		self.side = our_side
		self.role = our_role

		self.preprocessing = Preprocessing()

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
				world = self.simulator.update(0.5)
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

			# TEST TASKS
			task.execute()

			# Information about the grabbers from the world
			grabbers = {
				'our_defender': self.world.our_defender.catcher_area,
				'our_attacker': self.world.our_attacker.catcher_area
			}

			# Information about states
			attackerState = ('QWER', 'ASDF')
			defenderState = ('QWER', 'ASDF')

			attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
			defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

			# Use 'y', 'b', 'r' to change color.
			c = waitKey(2) & 0xFF

			if c == 32:
				self.simulator.move_ball()

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

# MAIN
if __name__ == '__main__':
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument("pitch", help="[0] Main pitch, [1] Secondary pitch")
	parser.add_argument("side", help="The side of our defender ['left', 'right'] allowed.")
	parser.add_argument("role", help="The role of our robot ['att', 'def'] allowed.")
	parser.add_argument("color", help="The color of our team ['yellow', 'blue'] allowed.")
	parser.add_argument(
		"-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")

	args = parser.parse_args()
	if args.nocomms:
		c = Controller(
			pitch=int(args.pitch), color=args.color, our_side=args.side, our_role=args.role, comms=0, sim=True).wow()
	else:
		c = Controller(
			pitch=int(args.pitch), color=args.color, our_side=args.side, our_role=args.role, sim=True).wow()
