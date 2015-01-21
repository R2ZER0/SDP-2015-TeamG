import Tkinter as tk
import time
import serial
class Controller:
	def __init__(self, comm_port='/dev/ttyACM0', comms=1):

		self.arduino = Arduino(comm_port, 115200, 1, comms)
		self.robot = Robot_Controller()

		# Minimalist GUI
		self.root = tk.Tk()
		self.root.geometry('300x200')
		self.text = tk.Text(self.root, background='black', foreground='white', font=('Comic Sans MS', 12))
		self.text.pack()
		self.root.bind('<KeyPress>', self.onKeyPress)
		self.root.after(500, self.run)
		self.root.mainloop()


	def run(self):
		"""
		Ready your sword, here be dragons.
		"""
		counter = 1L
		timer = time.clock()
		
		self.robot.execute(self.arduino)
		self.text.insert('end', 'Incoming %s\n' % (self.arduino.read(), ))
		self.root.after(500, self.run)
		

	def onKeyPress(self,event):
		self.text.insert('end', 'You pressed %s\n' % (event.char, ))
		self.robot.add(event.char)

class Robot_Controller(object):

	def __init__(self):
		self.actions = []

	def add(self, action):
		self.actions.append(action)

	def execute(self, comm):
		"""
		Execute robot action.
		"""
		for act in self.actions:
			comm.write(act)
			self.actions.remove(act)
	
	def shutdown(self, comm):
		# TO DO
			pass


class Arduino:

	def __init__(self, port, rate, timeOut, comms):
		self.serial = None
		self.comms = comms
		self.port = port
		self.rate = rate
		self.timeout = timeOut
		self.setComms(comms)

	def setComms(self, comms):
		if comms > 0:
			self.comms = 1
			if self.serial is None:
				try:
					self.serial = serial.Serial(self.port, self.rate, timeout=self.timeout)
				except:
					print "No Arduino detected!"
					print "Continuing without comms."
					self.comms = 0
					#raise
		else:
			#self.write('A_RUN_KICK\n')
			self.write('A_RUN_ENGINE %d %d\n' % (0, 0))
			#self.write('D_RUN_KICK\n')
			self.write('D_RUN_ENGINE %d %d\n' % (0, 0))
			self.comms = 0

	def write(self, string):
		if self.comms == 1:
			self.serial.write(string)


	def read(self):
		if self.comms == 1:
			return self.serial.read()

if __name__ == '__main__':
	c = Controller().run()
	