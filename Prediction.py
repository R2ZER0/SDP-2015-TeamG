import numpy as np
import Polygon.Utils
import math
class NN:
	def __init__(self, input, hidden, out):
		# number of input, hidden, and output nodes
		self.input = input + 1
		self.hid = hidden
		self.out = out

		# current activation levels of different neurons        
		self.ain = [1.0]*self.input
		self.ahid = [1.0]*self.hid
		self.aout = [1.0]*self.out
		
		# create weights
		self.LayerIn = np.random.rand(self.input, self.hid)
		self.LayerOut = np.random.rand(self.hid, self.out)

		# last change in weights for momentum   
		self.momentumIn = makeMatrix(self.input, self.nh)
		self.momentumOut = makeMatrix(self.nh, self.no)

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
				self.ahid[j] = sigmoid(sum)

			# output activations
			for k in range(self.out):
				sum = 0.0
				for j in range(self.hid):
					sum = sum + self.ahid[j] * self.LayerOut[j][k]
				self.aout[k] = sigmoid(sum)

		return self.aout


	def backPropagate(self, targets, learnRate, momentum):
		if len(targets) != self.out:
			raise ValueError('wrong number of target values')

		# calculate error terms for output
		errorOut = [0.0] * self.out
		for k in range(self.out):
			error = targets[k] - self.aout[k]
			errorOut[k] = dsigmoid(self.aout[k]) * error

		# calculate error terms for hidden
		errorHid = [0.0] * self.nh
		for j in range(self.hid):
			error = 0.0
			for k in range(self.out):
				error = error + errorOut[k]*self.LayerOut[j][k]
			errorHid[j] = dsigmoid(self.ahid[j]) * error

		# update output weights
		for j in range(self.hid):
			for k in range(self.out):
				change = errorOut[k]*self.ahid[j]
				self.LayerOut[j][k] = self.LayerOut[j][k] + learnRate*change + momentum*self.momentOut[j][k]
				self.momentOut[j][k] = change

		# update input weights
		for i in range(self.input):
			for j in range(self.hid):
				change = errorhid[j]*self.ain[i]
				self.LayerIn[i][j] = self.LayerIn[i][j] + learnRate*change + momentum*self.momentIn[i][j]
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
		for i in range(self.ni):
			print(self.wi[i])
		print()
		print('Output weights:')
		for j in range(self.nh):
			print(self.wo[j])

	def train(self, patterns, iterations=100000, N=0.5, M=0.1):
		# N: learning rate
		# M: momentum factor
		for i in range(iterations):
			error = 0.0
			for p in patterns:
				inputs = p[0]
				targets = p[1]
				self.update(inputs)
				error = error + self.backPropagate(targets, N, M)
			if i % 100 == 0:
				print('error %-.5f' % error)

class Predictor:
	def __init__(self,patterns):
		self.nn = NN(60,20,4)
		self.nn.train(patterns)
		self.cur = [0 for x in range(10)]           # feature vector of form [x,y, dirx, diry, dx, dy, rot, distRight, distFront, distLeft ]
		self.hist = [[0 for x in range(10)] for x in range(6)] # newest one at list end
	
	def predict(self):
		input = []
		for vec in hist:
			input.append(self.cur[0] - vec[0])  # Difference in x coord.
			input.append(self.cur[1] - vec[1])  # Difference in y coord.
			input.append(self.cur[2] - vec[2])  # Difference in cos(angle)
			input.append(self.cur[3] - vec[3])  # Difference in math.sin(angle)
			input.append(vec[4])                # dx of command
			input.append(vec[5])                # dy of command
			input.append(vec[6])                # rotation of commands
			input.append(vec[7])                # distance at -45 to movement dir
			input.append(vec[8])                # distance in movement dir
			input.append(vec[9])                # distance at 45  to movement dir
		return self.nn.update(input)
	
	def update(self, vec):
		self.hist.pop(0)
		self.hist.append(self.cur)
		self.cur = vec
		return self.nn.predict()
		
def preprocess(self, world, com, bot):
	vec = []
	vec.append(world._robots[bot][1])
	vec.append(world._robots[bot][2])
	vec.append(Math.cos(world._robots[bot][3]))
	vec.append(Math.math.sin(world._robots[bot][3]))
	vec.extend(com)
	vec.extend(dist(world._robots[bot][1],world._robots[bot][2], ANGLE, world.pitch.zones[bot]))
	
# TO BE CAPTURED BY WORLD STATE --> DIRECTION OF ROBOT VS. DIRECTION OF MOVEMENT --> ADJUSTED BY ACTION CLASS
#                               --> ROTATIONAL SPEED --> ADJUSTED BY ACTION CLASS
def dist(x, y, angle, poly):
	p1 = Polygon.Utils.pointList(poly)[0]	
	pointAngle = math.atan((p1[1] - y)/(p1[0] - x))
	fst = snd = third = False
	seg = [0,0,0]
	for p in Polygon.Utils.pointList(poly):
		a = math.atan((p[1] - y)/(p[0] - x))
		if not fst and (angle - 45) > pointAngle and a > (angle - 45) or (angle - 45) < pointAngle and a < (angle - 45):
			alpha = math.acos(((p1[0] - x) * (p[0] - p1[0]) + (p1[1] - y) * (p[1] - p1[1]))/(hypot((p1[0] - x), (p1[1] - y))*hypot((p1[0] - p[0]), (p1[1] - p[1]))))
			beta = 180 - angle - 45 + a - alpha
			seg[0] = hypot((p1[0] - x),(p1[1] - y)) * math.sin(alpha) / math.sin(beta)
			fst = True
		if not snd and angle > pointAngle and a > angle or angle < pointAngle and a < angle:
			alpha = math.acos(((p1[0] - x) * (p[0] - p1[0]) + (p1[1] - y) * (p[1] - p1[1]))/(hypot((p1[0] - x), (p1[1] - y))*hypot((p1[0] - p[0]), (p1[1] - p[1]))))
			beta = 180 - angle + a - alpha
			seg[1] = hypot((p1[0] - x),(p1[1] - y)) * math.sin(alpha) / math.sin(beta)
			snd = True
		if not third and angle + 45 > pointAngle and a > angle + 45 or angle + 45 < pointAngle and a < angle + 45:
			alpha = math.acos(((p1[0] - x) * (p[0] - p1[0]) + (p1[1] - y) * (p[1] - p1[1]))/(hypot((p1[0] - x), (p1[1] - y))*hypot((p1[0] - p[0]), (p1[1] - p[1]))))
			beta = 180 - angle + 45 + a - alpha
			seg[2] = hypot((p1[0] - x),(p1[1] - y)) * math.sin(alpha) / math.sin(beta)
			third = True
		if fst and snd and third:
			break
		else:
			p1 = p
			pointAngle = a
	return seg
