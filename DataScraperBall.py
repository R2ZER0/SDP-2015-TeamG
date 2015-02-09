from vision.vision import Vision, Camera, GUI
import vision.tools as tools
from cv2 import waitKey
import cv2
import serial
import warnings
import time
from postprocessing.postprocessing import Postprocessing
from planning.models import World
import Prediction as pre
import math
warnings.filterwarnings("ignore", category=DeprecationWarning)

class DataScraper:
	"""
	Primary source of robot control. Ties vision and planning together.
	"""

	def __init__(self, pitch, color, our_side, video_port=0, comm_port='/dev/ttyUSB0', comms=1):
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
		assert pitch in [0, 1]
		assert color in ['yellow', 'blue']
		assert our_side in ['left', 'right']

		self.pitch = pitch
		
		# Set up camera for frames
		self.camera = Camera(port=video_port, pitch=self.pitch)
		frame = self.camera.get_frame()
		center_point = self.camera.get_adjusted_center(frame)

		# Set up vision
		self.calibration = tools.get_colors(pitch)
		self.vision = Vision(
			pitch=pitch, color=color, our_side=our_side,
			frame_shape=frame.shape, frame_center=center_point,
			calibration=self.calibration)
		self.postprocessing = Postprocessing()
		self.world = World(our_side, pitch)

		self.color = color
		self.side = our_side

	def run(self):
		f = open('data.txt', 'w')
		cv2.namedWindow("Test")
		cache = [] # oldest is first, newest is last
		for i in range(9):
			frame = self.camera.get_frame()
			# Find object positions
			# model_positions have their y coordinate inverted
			model_positions, regular_positions = self.vision.locate(frame)
			model_positions = self.postprocessing.analyze(model_positions)

			# Find appropriate action
			self.world.update_positions(model_positions)
			newFeat = [self.world._ball.x, self.world._ball.y , math.cos(self.world._ball.angle) * self.world._ball.velocity, math.sin(self.world._ball.angle) * self.world._ball.velocity]
			cache.append(newFeat)

		counter = 1L
		timer = time.clock()
		c = True
		grab = False
		while c != 27:
			frame = self.camera.get_frame()
			# Find object positions
			# model_positions have their y coordinate inverted
			model_positions, regular_positions = self.vision.locate(frame)
			model_positions = self.postprocessing.analyze(model_positions)
			self.world.update_positions(model_positions)
			newFeat = [self.world._ball.x, self.world._ball.y , math.cos(self.world._ball.angle) * self.world._ball.velocity, math.sin(self.world._ball.angle) * self.world._ball.velocity]
			if grab:			
				i = buildfeature(cache)
				for n in i:
					f.write(str(n) + " ")
				f.write("-> ")
				f.write(str(newFeat[0] - cache[7][0]) + " " + str(newFeat[1] - cache[7][1]))
				f.write("\n")
			old = cache.pop(0)
			cache.append(newFeat)
			print newFeat
			counter += 1
			c = waitKey(2) & 0xFF
			if c == 32:
				grab = not grab
def buildfeature(cache):
	i = []
	for f in range(6):
		i.append(cache[f][0] - cache[7][0])
		i.append(cache[f][1] - cache[7][1])
		i.extend(cache[f][2:])
	print i
	return i

if __name__ == '__main__':
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument("pitch", help="[0] Main pitch, [1] Secondary pitch")
	parser.add_argument("side", help="The side of our defender ['left', 'right'] allowed.")
	parser.add_argument("color", help="The color of our team - ['yellow', 'blue'] allowed.")
	parser.add_argument(
		"-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")

	args = parser.parse_args()
	if args.nocomms:
		c = DataScraper(
			pitch=int(args.pitch), color=args.color, our_side=args.side, comms=0).run()
	else:
		c = DataScraper(
			pitch=int(args.pitch), color=args.color, our_side=args.side).run()
