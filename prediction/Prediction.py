import numpy as np
import Polygon.Utils
import math
from planning.models import Vector

class KalmanFilter:
	def __init__(self, init, stateTransition, control, measurement, uncertainty, noise):
		self.A = stateTransition
		self.B = control
		self.C = measurement
		self.Ex = uncertainty
		self.Ez = noise
		self.x = init
		self.var = np.zeros(stateTransition.shape)
	def step(self, meas, contr):
		# Prediction Step
		self.x = self.pStep(self.x, contr)
		# Correction Step
		self.var = np.dot(np.dot(self.A, self.var), self.A.transpose()) + self.Ex
		S = np.dot(np.dot(self.C, self.var), self.C.transpose()) + self.Ez
		K = np.dot(np.dot(self.var, self.C.transpose()), S.getI())
		y = np.subtract(meas, np.dot(self.C, self.x))
		self.x = np.add(self.x, np.dot(K, y))
		self.var = np.dot((np.identity(self.C.shape[1]) - np.dot(K, self.C)), self.var)
		return self.x
	def pStep(self, x, c):
		return np.add(np.dot(self.A, x) , np.dot(self.B, c))
		

class KalmanBallPredictor:

	def __init__(self, init, friction = 1):
		t = 1
		A = np.matrix([[1, 0, t, 0],
			       [0, 1, 0, t],
			       [0, 0, 1, 0],
			       [0, 0, 0, 1]])
		B = np.matrix([[0],
			       [0],
			       [0],
			       [0]])
		C = np.matrix([[1, 0, 0, 0],
			       [0, 1, 0, 0],
			       [0, 0, 1, 0],
			       [0, 0, 0, 1]])
		Ex = np.matrix([[0, 0, 0, 0],
			        [0, 0, 0, 0],
			        [0, 0, 0.1, 0],
			        [0, 0, 0, 0.1]])
		Ez = np.matrix([[3, 0, 0, 0],
			        [0, 3, 0, 0],
			        [0, 0, 3, 0],
			        [0, 0, 0, 3]])

		vector = np.array([[init.x], 
	 			  [init.y], 
				  [math.cos(init.angle)*init.velocity], 
				  [math.sin(init.angle)*init.velocity]])

		self.filter = KalmanFilter(vector, A,B,C,Ex,Ez)
		self.control = friction
		self.THRESHOLD = 25
		self.CATCH_DIST = 20

	def predict(self, world, time = 8):
		vector = np.array([[world.ball.x], 
			 	   [world.ball.y], 
				   [math.cos(world.ball.angle)*world.ball.velocity], 
				   [math.sin(world.ball.angle)*world.ball.velocity]])
		#for x in vector:
		#	print x,
		#print	
		doubtFul = False
		zone = None
		predX = self.filter.step(vector, self.control)	
		for z in world._pitch._zones:
			if z.isInside(predX[0], predX[1]):
				zone = z
		if zone == None:
			return True, world.ball.vector
		for i in range(time - 1):
			new = self.filter.pStep(predX, self.control)
			zoneNew = None
			for z in world._pitch._zones:
				if z.isInside(new[0], new[1]):
					zoneNew = z
			if zoneNew == None:
				new = self.collision(predX, zone, world)
			elif zoneNew != zone and math.hypot(new[2], new[3]) <= self.THRESHOLD:
				new = self.collision(predX, zone, world)
			for robot in world._robots:
				if math.hypot(robot.x - new[0], robot.y - new[1]) < self.CATCH_DIST:
					doubtFul = True
			predX = new 

		old_vector = world.ball.vector
		
		#old_vector.x = predX[0]
		#old_vector.y = predX[1]
		#for x in predX:
		#	print x,
		#print
		#print
		return doubtFul, old_vector

	def collision(self, vector, zone, world):
		x = vector.item((0, 0))
		y = vector.item((1, 0))
		angle = math.atan2(vector[3], vector[2])
		v = math.hypot(vector[3], vector[2])
		# calculates what happens if Ball "collides" with zone boundary
		xnew = x 
		ynew = y
		anglenew = angle
		# find the line on which the collision happens
		p1 = Polygon.Utils.pointList(zone)[0]	
		pointAngle = math.atan((p1[1] - y)/(p1[0] - x)) if (p1[0] - x) != 0 else 0
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
				anglenew = alpha + 2 * beta if (math.pi > alpha + 2 * beta > 0) else alpha + 2 * beta - math.pi
				xnew += dist * math.cos(angle) + (v-dist) * math.cos(anglenew)
				ynew += dist * math.sin(angle) + (v-dist) * math.sin(anglenew)
				break
			else:
				p1 = p
				pointAngle = a
		return np.matrix([[xnew], [ynew], [math.cos(anglenew)*v], [math.sin(anglenew)*v]])

class KalmanRobotPredictor:
	def __init__(self,init, friction = -10, acceleration = 25):
		t = 1
		A = np.matrix([[1, 0, 0, t, 0, 0],
			       [0, 1, 0, 0, t, 0],
			       [0, 0, 1, 0, 0, 1],
			       [0, 0, 0, 1, 0, 0],
			       [0, 0, 0, 0, 1, 0],
			       [0, 0, 0, 0, 0, 1]])
		B = np.zeros((6,3))
		B = np.matrix([[0, 0, 0],
			       [0, 0, 0],
			       [0, 0, 0],
			       [t, 0, 0],
			       [0, t, 0],
			       [0, 0, t]])
		C = np.matrix([[1, 0, 0, 0, 0, 0],
			       [0, 1, 0, 0, 0, 0],
			       [0, 0, 1, 0, 0, 0],
			       [0, 0, 0, 1, 0, 0],
			       [0, 0, 0, 0, 1, 0],
			       [0, 0, 0, 0, 0, 1]])
		Ex = np.matrix([[0, 0, 0, 0, 0, 0],
			       [0, 0, 0, 0, 0, 0],
			       [0, 0, 0.0001, 0, 0, 0],
			       [0, 0, 0, 0.1, 0, 0],
			       [0, 0, 0, 0, 0.1, 0],
			       [0, 0, 0, 0, 0, 0.0001]])
		Ez = np.matrix([[10, 0, 0, 0, 0, 0],
			       [0, 10, 0, 0, 0, 0],
			       [0, 0, 0.0005, 0, 0, 0],
			       [0, 0, 0, 25, 0, 0],
			       [0, 0, 0, 0, 25, 0],
			       [0, 0, 0, 0, 0, 0.1]])

		vector = np.array([[init.x],
						  [init.y],
						  [init.angle],
						  [math.cos(init.angle)*init.velocity],
						  [math.sin(init.angle)*init.velocity],
						  [0]])

		self.filter = KalmanFilter(vector, A,B,C,Ex,Ez)
		self.friction = friction
		self.a = acceleration

	def predict(self, control, world, time = 8):
		vector = np.array([[world.our_attacker.x],
						  [world.our_attacker.y],
						  [world.our_attacker.angle],
						  [math.cos(world.our_attacker.angle)*world.our_attacker.velocity],
						  [math.sin(world.our_attacker.angle)*world.our_attacker.velocity],
						  [0]])
		control = np.array(control).transpose()
		zone = None
		predX = self.filter.step(vector, self.convert(vector, control))
		for z in world._pitch._zones:
			if z.isInside(predX[0], predX[1]):
				zone = z
		if zone == None:
			return world.our_attacker.vector

		for i in range(time - 1):
			new = self.filter.pStep(predX, self.convert(predX, control))
			zoneNew = None
			for z in world._pitch._zones:
				if z.isInside(new[0], new[1]):
					zoneNew = z
			if zoneNew == z:
				old_vector = world.our_attacker.vector

				#old_vector.x = predX.item((0, 0))
				#old_vector.y = predX.item((1, 0))
				#old_vector.angle = predX.item((2, 0)) % (2*math.pi)

				return old_vector
			predX = new
			predX[2,0] = predX.item((2, 0)) % 2*math.pi
			if predX.item((3, 0)) < 0:
				predX[3,0] = 0
			if predX.item((4, 0)) < 0:
				predX[4,0] = 0
			if predX.item((5, 0)) < 0:
				predX[5,0] = 0

		old_vector = world.our_attacker.vector

		#old_vector.x = predX.item((0, 0))
		#old_vector.y = predX.item((1, 0))
		#old_vector.angle = predX.item((2, 0))

		return old_vector

	def convert(self, state, control):
		ax = 0
		ay = 0
		ar = 0
		if state.item((3, 0)) != control[0]:
			if control[0] != 0:
				ax = (control[0] - state.item((3, 0))) / self.a
			else:
				if abs(self.friction) < state.item((3, 0)):
					ax = self.friction
				else:
					ax = int(-state.item((3, 0)))
		if state.item((4, 0)) != control[1]:
			if control[0] != 0:
				ay = (control[1] - state.item((4, 0))) / self.a
			else:
				if abs(self.friction) < state.item((4, 0)):
					ay = self.friction
				else:
					ay = int(-state.item((3, 0)))
		if state.item((5, 0)) != control[2]:
			if control[2] != 0:
				ar = control[2] - state.item((5, 0)) / self.a
			else:
				if abs(self.friction) < state.item((5, 0)):
					ar = self.friction / 10
				else:
					ar = int(-state.item((5, 0)))
		return np.matrix([[ax],
				  [ay],
				  [ar]])
