#!/usr/bin/env python

import serial
from action import Action
import math

comm = serial.Serial("/dev/ttyACM6", 115200, timeout=1)
#comm = None
robot = Action(comm)

from flask import Flask
app = Flask(__name__)

@app.route("/stop")
def do_stop():
    robot.stop()
    return "ok"
    
@app.route("/forward")
def do_forward():
    robot.move(0, 1)
    return "ok"
    
@app.route("/backward")
def do_backward():
    robot.move(0, -1)
    return "ok"

@app.route("/left")
def do_left():
    robot.move(math.pi/2.0, 1)
    return "ok"

@app.route("/right")
def do_right():
    robot.move(math.pi*3.0/2.0, 1)
    return "ok"

@app.route("/turncw")
def do_turncw():
    robot.turn(Action.TURN_CLOCKWISE, 0.5)
    return "ok"

@app.route("/turnacw")
def do_turnacw():
    robot.turn(Action.TURN_ANTICLOCKWISE, 0.5)
    return "ok"
    
if __name__ == "__main__":
    app.debug = True
    app.run(port=5001)