from action import Action
from serial import Serial
from math import pi
import sys

serial_port = "/dev/ttyACM0"
if len(sys.argv) > 1:
    serial_port = sys.argv[1]
    print "Using serial port " + serial_port
        

s = Serial(serial_port, 115200)
a = Action(s)

def quit():
      a.exit()
      exit()
