from action import Action
from serial import Serial
from math import pi

s = Serial('/dev/ttyACM0', 115200)
a = Action(s)

def quit():
  a.exit()
  exit()