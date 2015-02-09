import numpy as np
import Polygon.Utils
import math
class NN:
	def __init__(self, inpu, hidden, out):
		# number of input, hidden, and output nodes
		self.input = inpu + 1
		self.hid = hidden
		self.out = out

		# current activation levels of different neurons        
		self.ain = [1.0]*self.input
		self.ahid = [1.0]*self.hid
		self.aout = [1.0]*self.out
		
		# create weights
		self.LayerIn = np.random.rand(self.input, self.hid)
		self.LayerOut = np.random.rand(self.hid, self.out)
		
		self.momentIn = [[0.0 for x in range(self.hid)] for y in range(self.input)]
		self.momentOut = [[0.0 for x in range(self.out)] for y in range(self.hid)]

	def update(self, inputs):
		if len(inputs) == self.input - 1:
			# input activations
			for i in range(self.input-1):
				self.ain[i] = inputs[i]

			# hidden activations
			for j in range(self.hid):
				sum = 0.0
				for i in range(self.input):
					sum = sum + self.ain[i] * self.LayerIn[i][j]
				self.ahid[j] = math.tanh(sum)

			# output activations
			for k in range(self.out):
				sum = 0.0
				for j in range(self.hid):
					sum = sum + self.ahid[j] * self.LayerOut[j][k]
				self.aout[k] = sum

		return self.aout


	def backPropagate(self, targets, learnRate, moment):
		if len(targets) != self.out:
			raise ValueError('wrong number of target values' + str(len(targets)) + " " + str(self.out))

		# calculate error terms for output
		errorOut = [0.0] * self.out
		for k in range(self.out):
			error = targets[k] - self.aout[k]
			errorOut[k] = error

		# calculate error terms for hidden
		errorHid = [0.0] * self.hid
		for j in range(self.hid):
			error = 0.0
			for k in range(self.out):
				error = error + errorOut[k]*self.LayerOut[j][k]
			errorHid[j] = (1.0 - (self.ahid[j])*(self.ahid[j])) * error
		# update output weights
		for j in range(self.hid):
			for k in range(self.out):
				change = errorOut[k]*self.ahid[j]
				self.LayerOut[j][k] = self.LayerOut[j][k] + learnRate * change + moment * self.momentOut[j][k]
				self.momentOut[j][k] = change

		# update input weights
		for i in range(self.input):
			for j in range(self.hid):
				change = errorHid[j]*self.ain[i]
				self.LayerIn[i][j] = self.LayerIn[i][j] + learnRate * change + moment * self.momentIn[i][j]
				self.momentIn[i][j] = change
		# calculate error
		error = 0.0
		for k in range(len(targets)):
			error = error + 0.5*(targets[k]-self.aout[k])**2
		return error


	def test(self, patterns):
		for p in patterns:
			print(p[0], '->', self.update(p[0]))

	def weights(self):
		print('Input weights:')
		for i in range(self.input):
			print(self.LayerIn[i])
		print()
		print('Output weights:')
		for j in range(self.hid):
			print(self.LayerOut[j])

	def train(self, patterns, iterations=100, N=0.01, M = 0.01):
		# N: learning rate
		pastErr = 0
		for i in range(iterations):
			error = 0.0
			for p in patterns:
				inputs = p[0]
				targets = p[1]
				self.update(inputs)
				error = error + self.backPropagate(targets, N, M)
			if i % 10 == 0:
				print(error)
			if pastErr - error < 0.0001:
				#print(error)
				#break
				pass
			pastErr = error
		

class NeuralBallPredictor:
	def __init__(self,patterns):
		self.nn = NN(24, 10, 2)
		self.nn.train(patterns)
		self.nn.weights()
		self.cur = [0 for x in range(4)]           # feature vector of form [x,y, dx, dy, dist, cos(angle), sin(angle)]
		self.hist = [[0 for x in range(4)] for x in range(6)] # newest one at list end
	
	def predict(self):
		input = []
		for vec in self.hist:
			input.append(self.cur[0] - vec[0])  # Difference in x coord.
			input.append(self.cur[1] - vec[1])  # Difference in y coord.
			input.append(vec[2])                # dx
			input.append(vec[3])                # dy
		return self.nn.update(input)
	
	def update(self, vec):
		if len(vec) == 4:
			self.hist.pop(0)
			self.hist.append(self.cur)
			self.cur = vec
		return self.predict()

class PhysicsBallPredictor:
	CATCH_DIST = 20
	THRESHOLD = 50
	def init(self, friction, tiltx, tilty):
		self.f = friction
	    	self.tx = tiltx
		self.ty = tilty
	
	def predict(world, time = 4):
		x = world._Ball.x
		y = world._Ball.y
		doubtFul = False
		angle = world._Ball.angle
		v = world._Ball.v
		zone = None
		for z in self.world._pitch._zones:
			if z.isInside(self.world._ball.x, self.world._ball.y):
				zone = z	
		for i in range(time):
			xnew = x + v * math.cos(angle)
			ynew = y + v * math.sin(angle)
			zoneNew = None
			for z in self.world._pitch._zones:
				if z.isInside(xnew, ynew):
					zoneNew = z	
			if zoneNew == None:
				xnew, ynew, angle = collison(self, x, y, angle, v, zone, world)
			elif zoneNew != z and v <= THRESHOLD:
				xnew, ynew, angle = collison(self, x, y, angle, v, zone, world)
			x = xnew
			y = ynew
			for robot in world._robots:
				if math.hypot(robot.x - xnew, robot.y - ynew) < CATCH_DIST:
					doubtFul = True
			v = v - friction
		return doubtFul, x, y

	def collision(self, x, y, angle, v, zone, world):
		# calculates what happens if Ball "collides" with zone boundary
		xnew = x 
		ynew = y
		anglenew = a
		# find the line on which the collision happens
		for p in Polygon.Utils.pointList(zone):
			a = math.atan((p[1] - y)/(p[0] - x)) if (p[0] - x) != 0 else 0
			if angle > pointAngle and a > angle or angle < pointAngle and a < angle:
				v1x = p1[0] - x
				v1y = p1[1] - y
				v2x = p[0] - p1[0]
				v2y = p[1] - p1[1]
				alpha = math.acos((v1x*v2x + v1y*v2y)/(math.hypot(v1x, v1y)*math.hypot(v2x, v2y)))
				beta = math.pi - angle + a - alpha
				dist = math.hypot(v1x, v1y) * math.sin(alpha) / math.sin(beta)
				anglenew = if  math.pi > alpha + 2 * beta > 0 else alpha + 2 * beta - math.pi
				xnew += dist * math.cos(angle) + (v-dist) * math.cos(anglenew)
				ynew += dist * math.sin(angle) + (v-dist) * math.sin(anglenew)
				break
			else:
				p1 = p
				pointAngle = a
		return xnew, ynew, anglenew

class RobotPredictor:
	def __init__(self,patterns):
		self.nn = NN(48, 15, 4)
		self.nn.train(patterns)
		self.cur = [0 for x in range(8)]           # feature vector of form [x,y, dirx, diry, dx, dy, rot, distFront]
		self.hist = [[0 for x in range(8)] for x in range(6)] # newest one at list end
	
	def predict(self):
		input = []
		for vec in self.hist:
			input.append(self.cur[0] - vec[0])  # Difference in x coord.
			input.append(self.cur[1] - vec[1])  # Difference in y coord.
			input.append(vec[2])                # dir x
			input.append(vec[3])                # dir y
			input.append(vec[4])                # dx
			input.append(vec[5])                # dy
			input.append(vec[6])                # rot
			input.append(vec[7])                # dist
		return self.nn.update(input)
	
	def update(self, vec):
		self.hist.pop(0)
		self.hist.append(self.cur)
		self.cur = vec
		return self.predict()
			
			
def distRobot(x, y, angle, poly, world):
	p1 = Polygon.Utils.pointList(poly)[0]	
	pointAngle = math.atan((p1[1] - y)/(p1[0] - x)) if (p1[0] - x) != 0 else 0
	for p in Polygon.Utils.pointList(poly):
		a = math.atan((p[1] - y)/(p[0] - x)) if (p[0] - x) != 0 else 0
		if angle > pointAngle and a > angle or angle < pointAngle and a < angle:
			v1x = p1[0] - x
			v1y = p1[1] - y
			v2x = p[0] - p1[0]
			v2y = p[1] - p1[1]
			alpha = math.acos((v1x*v2x + v1y*v2y)/(math.hypot(v1x, v1y)*math.hypot(v2x, v2y)))
			beta = math.pi - angle + a - alpha
			return math.hypot(v1x, v1y) * math.sin(alpha) / math.sin(beta)
		else:
			p1 = p
			pointAngle = a
	return 0

