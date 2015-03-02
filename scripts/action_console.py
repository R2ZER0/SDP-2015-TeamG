from action import Action
from serial import Serial

s = Serial('/dev/ttyACM3', 115200)
a = Action(s)

def quit():
  a.exit()
  exit()
