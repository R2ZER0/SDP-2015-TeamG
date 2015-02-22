
import math
import motor_calibration

class Action():
	'''Provides an interface allowing the sending high-level movement commands
	to the Robot.
	'''

	#: Setting True enables verbose output of commands
	Debug = False
	
	#: The angles of our wheels, relative to the x-axis. Begins \
	#: from front-left and moves anticlockwise.
	MOTOR_ANGLES = [ math.pi*5.0/3.0, math.pi/2.0, math.pi*4/3.0 ]

	#: Trigonometric constants for each motor, used for holonomic movement.
	MOTORS = list(map(lambda x: (math.cos(x), math.sin(x)), MOTOR_ANGLES))
	
	def __init__(self, comm):
		'''Initialises an Action instance with the given communication object.

		:param comm: A pySerial serial object for communication
		'''
		self.comm = comm
	
	def move(self, angle, scale):
		"""Moves the robot in the given angle with given scale as power value.

		.. warning::

		  Movement currently not functioning for angles other than 0 and pi.

		:param angle: The direction of movement, in radians, relative to x-axis \
				of robot
		:param scale: Power-scale for this movement, range: -1 to 1
		"""

		motor_speeds = [ Action._calc_motor_speed(motor, angle) for motor in self.MOTORS ]
		motor_speeds = Action._normalise_speeds(motor_speeds)
		motor_speeds = map(lambda x: int(x*scale_max), motor_speeds)
		self._send_run(motor_speeds)
  
	def turn(self, speed):
		"""Turn the robot with the given power.

		:param speed: The speed of the turn, range is [-100,100]. Positive values \
					result in clockwise rotation, negative in anticlockwise.
		"""
		self._send_run([int(speed), int(speed), int(speed)])
  
	def stop(self):
		"""Stops the robot's movement by setting all motors to zero.

		.. note::

		   This currently does not stop the Kicker/Catcher mechanisms.
		"""
		motor_speeds = [ int(0), int(0), int(0) ]
		self._send_run(motor_speeds)
  
	def kick(self, scale=100):
		'''Sends the kick command to the robot.

		:param scale: Power scale for this kick. Range [0,100].
		'''
		# Note, scale must be in list form
		self._send_command("KICK",[scale])
	
	def catch(self, scale=100):
		"""Sends the catch command to the robot.

		:param scale: Power scale for this catch. Range [0,100].
		"""
		self._send_command("CATCH",[scale])

	def open_catcher(self, scale=100):
		"""Sends the command to open the catcher to the robot.

		:param scale: Power scale for the catch. Range [0,100].
		"""
		self._send_command("RELEASE", [scale])

	# Utility
	def ping(self):
		"""Sends the PING command to the Robot. Robot should return "PONG" via
		the serial connection if correctly functioning.
		"""
		self._send_command("PING", [])
	
	# Private Methods #
	
	# Commands
	def _send_run(self, speeds):
		"""Convenience function for sending the given motor speeds to the Robot.

		:param speeds: A list of motor speeds for each motor. Range [-100,100].
		"""
		self._send_command("RUN", speeds)

	# Utility
	@staticmethod
	def _normalise_speeds(speeds):
		'''Normalises a list of motor speeds to be ratio of speed to 
		maximum speed in the list.

		:param speeds: A list of motor speeds
		:returns: A new list of motor speeds, with speeds relative to maximum.
		'''
		maxspeed = max([abs(s) for s in speeds])
		return map( lambda x: (x/maxspeed), speeds )        
	
	@staticmethod
	def _percentage_speed(speed):
		"""Constrains speed to be within the range [-100,100].

		:param speed: A speed in the range [-1,1]
		"""
		speed = int(round(100.0 * speed))
		speed = max(-100, min(speed, 100))
		return speed
	
	@staticmethod
	def _calc_motor_speed(angle):
		'''Hardcoded motor speed calculations for wheels in current placement of robot.
		Calculates wheel powers for each wheel to move in the given direction.

		:param angle: The direction of travel.
		:returns: A list of motor speeds, in the range [-1,1]
		'''
		m = [0,0,0]
		m[0] = math.cos(angle) - math.sin(angle)/(2+math.sqrt(3))
		m[1] = 2*math.sin(angle)/(2+math.sqrt(3))
		m[2] = -math.cos(angle) - math.sin(angle)/(2+math.sqrt(3))
		return m

	@staticmethod
	def _get_command_string(command, *args):
		"""Constructs a suitable syntax command string for sending the given
		command to the Robot with supplied arguments.

		:param command: A command string
		:param args: None, single argument, multiple argument, or a list.
		"""
		commstr = str(command)
		
		# Append each argument with a space
		for arg in args:
			commstr = commstr + " " + str(arg)
			
		# Terminate with correct newline syntax
		return commstr + "\r\n"            
	
	def _send_command(self, command, *args):
		"""Sends the given command to the robot, with arguments.

		:param command: A command string
		:param args: None, single argument, multiple argument, or a list.
		"""
		commstr = Action._get_command_string(command, args)

		if self.Debug:
			print "Sending command: " + commstr
		
		if self.comm is not None:
			self.comm.write(commstr)
			self.comm.flush()
	
	
	
