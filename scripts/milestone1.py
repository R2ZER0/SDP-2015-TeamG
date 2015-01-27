
import serial
from action import Action
import sys
import time
import argparse

# Serial object for communication
comm = None

# Action class instance
robot = None

def forward10():
  '''
  Moves the Robot forward 10cm. Sleep time custom tuned for distance.
  '''
  robot.move(0, 1)
  time.sleep(0.47)
  robot.stop()


def backward20():
  '''
  Moves the Robot backwards 20cm. Sleep time custom tuned for distance.
  '''
  robot.move(0, -1)
  time.sleep(0.71)
  robot.stop()


def forward50():
  '''
  Moves the Robot forward 50cm. Sleep time custom tuned for distance.
  '''
  robot.move(0, 1)
  time.sleep(1.57)
  robot.stop()

def kick():
  robot.kick(100)
  time.sleep(1)
  catch()

def kick_pass():
  robot.kick(50)
  time.sleep(1)
  catch()

def catch():
  robot.catch(100)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()

  # Add serial port as an optional argument
  parser.add_argument("--port", default='/dev/ttyACM0', help="Serial port for the communication. Default:/dev/ttyACM0")

  # Add two mutually-exclusive possible actions: kick and distance
  action_group = parser.add_mutually_exclusive_group(required=True)
  action_group.add_argument("-d", "--distance", type=int, help="Distance to move. Valid: 10, -20, 50")
  action_group.add_argument("-k", "--kick", action="store_true", help="Activate Kicker")
  action_group.add_argument("-c", "--catch", action="store_true", help="Activate Grabber")
  action_group.add_argument("-p", "--kick_pass", action="store_true", help="Pass to Attacker Zone")

  # Validate arguments
  args = parser.parse_args()

  # Establish communications
  print 'Using port %s' % (args.port)

  comm = serial.Serial(args.port, 115200)
  robot = Action(comm)

  comm.write('PING' + '\r\n')
  print comm.readline()

  # Kicking action
  if args.kick:
    kick()

  # Catching action
  elif args.catch:
    catch()

  elif args.kick_pass:
    kick_pass()

  # Distance kicking actions
  elif args.distance:

    # Ensure distances matches valid set of distances
    if args.distance == 10:
      forward10()
    elif args.distance == -20:
      backward20()
    elif args.distance == 50:
      forward50()
    else:
      print 'Error: Invalid distance specified to move %d' % (args.distance)
      sys.exit(1)

