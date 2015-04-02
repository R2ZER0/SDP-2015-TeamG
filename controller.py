from action import Action
from planning.tasks import AcquireBall, MoveToPoint, TurnToPoint
from planning.planner import Planner
from planning.models import World
from vision.vision import Vision, Camera, GUI
import vision.tools as tools
from prediction.Prediction import KalmanBallPredictor, KalmanRobotPredictor
from preprocessing.preprocessing import Preprocessing
from postprocessing.postprocessing import Postprocessing
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera
from cv2 import waitKey
import cv2
import math
import serial
import warnings
import time
from planning.models import World
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera

warnings.filterwarnings("ignore", category=DeprecationWarning)

class CustomError(Exception):
    '''Custom exception class. Used to catch, and print out the exception that
    occurred.'''

    def __init__(self, value):
        '''Initialises with a given error occurrence, this is used to format the
        string representation of this error.'''
        self.value = value

    def __str__(self):
        return repr(self.value)

class Controller:
    '''Primary controller for the robot. Executes the standard loop, using all modules.'''

    #: Beginning initialisation time, in seconds.
    INITIALISE_TIME = 3

    def __init__(self, pitch, color, our_side, attack_port, defend_port, plannerSpecFiles, video_port=0):
        '''Initialises the main controller. Constructs all necessary elements; doesn't start until
        run is called.

        :param pitch:       Pitch number to play on. [0: Main pitch, 1: Secondary pitch]
        :param color:       Which color our plate is; used for vision tracking. ['yellow', 'blue']
        :param our_side:    Designates our side, 'left' for  left side of captured frame, 'right' otherwise.
        :param attack_port: Serial port to communicate with attacker robot. None for no communications
        :param defend_port: Serial port to communicate with defender robot. None for no communications
        :param video_port:  Which serial port to use for the camera, an integer, usually 0 for /dev/video0.
        '''
        assert pitch in [0, 1]
        assert color in ['yellow', 'blue']
        assert our_side in ['left', 'right']

        # Initialise starting time; used for initial settling time
        start_time = time.clock()

        self.pitch = pitch

        # Set up camera for frames
        self.camera = Camera(port=video_port, pitch=self.pitch)
        
        # Group 10's Robot initialisation (Defender)
        self.arduino10 = Arduino(defend_port, 115200, 1, not defend_port is None)
        self.robot10 = Robot_Controller()

        # Group 9's Robot initialisation (Attacker)
        if not attack_port is None:
            comm = serial.Serial(attack_port, 115200, timeout=1)
            comm.flushInput()
        else:
            comm = None

        self.robot9 = Action(comm)

        # The input spec files
        self.plannerSpecFiles = plannerSpecFiles

        # Grab an initial frame to adjust for center position
        frame = self.camera.get_frame()
        center_point = self.camera.get_adjusted_center(frame)

        # Set up vision
        self.calibration = tools.get_colors(self.pitch)
        self.vision = Vision(
            pitch=pitch, color=color, our_side=our_side,
            frame_shape=frame.shape, frame_center=center_point,
            calibration=self.calibration)

        # Set up postprocessing for vision
        self.preprocessing = Preprocessing()
        self.postprocessing = Postprocessing()

        # Set up world
        self.world = World(our_side, pitch)

        # Set up main planner
        self.planner = Planner(self.world, self.robot9, 'attacker', self.plannerSpecFiles)

        # Set up GUI
        self.GUI = GUI(calibration=self.calibration, pitch=self.pitch)

        # Set up our cache of commands for the predictors
        self.command_cache = [[0,0,0]]*8
        self.command = [0,0,0]

        self.color = color
        self.side = our_side

        # start capturing frames to fill up an intial world state
        counter = 0
        while time.clock() < start_time + Controller.INITIALISE_TIME:

                frame = self.camera.get_frame()
                pre_options = self.preprocessing.options

                # model_positions have their y coordinate inverted
                model_positions, regular_positions = self.vision.locate(frame)
                model_positions = self.postprocessing.analyze(model_positions)

                # Update world state
                self.world.update_positions(model_positions)

                # Information about the grabbers from the world
                grabbers = {
                    'our_defender': self.world.our_defender.catcher_area,
                    'our_attacker': self.world.our_attacker.catcher_area
                }

                # Information about states
                attackerState = ("INITIALISING", "INITIALISING")
                defenderState = ("INITIALISING", "INITIALISING")

                attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
                defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

                # Use 'y', 'b', 'r' to change color.
                c = waitKey(2) & 0xFF

                actions = []
                fps = float(counter) / (time.clock() - start_time)
                # Draw vision content and actions
            
                self.GUI.draw(
                    frame, model_positions, actions, regular_positions, fps, attackerState,
                    defenderState, attacker_actions, defender_actions, grabbers,
                    our_color=self.color, our_side=self.side, key=c, preprocess=pre_options)
                counter += 1
        
        # Set up our cache of commands for the predictors
        self.command_cache = [[0,0,0]]*8
        self.command = [0,0,0]

        # Set up predictors
        #
        # TODO: Add Robot 10 prediction here
        self.ball_predictor = KalmanBallPredictor(self.world.ball.vector, friction=0)
        self.robot_predictor = KalmanRobotPredictor(self.world.our_attacker.vector, friction=-10, acceleration=25)

    def run(self):
        '''Main function, controls entire loop of the program.
        '''
        counter = 1L

        #: Timer is used for FPS counting
        timer = time.clock()

        #: Try used to catch any internal exceptions and still send shutdown to
        #: robots if one does occur.
        try:
            c = True
            while c != 27:  # the ESC key

                frame = self.camera.get_frame()
                pre_options = self.preprocessing.options

                # Apply preprocessing methods toggled in the UI
                preprocessed = self.preprocessing.run(frame, pre_options)
                frame = preprocessed['frame']

                # Find object positions
                # model_positions have their y coordinate inverted
                model_positions, regular_positions = self.vision.locate(frame)
                model_positions = self.postprocessing.analyze(model_positions)

                # Update world state
                self.world.update_positions(model_positions)

                # Run the planner
                #
                # TODO: Add Robot 10 prediction here
                self.command = self.command_cache.pop(0)
                self.planner.plan()
                self.command_cache.append(self.robot9.last_command())

                # Predict ball position and replace regular ball position with this
                ball_doubtful, self.world.ball.vector = self.ball_predictor.predict(self.world, time = 8)
                if regular_positions['ball']:
                    regular_positions['ball']['x'] = self.world.ball.vector.x
                    regular_positions['ball']['y'] = self.world._pitch.height -  self.world.ball.vector.y
                self.world.our_attacker.vector = self.robot_predictor.predict(self.command, self.world, time = 8)

                # Information about the grabbers from the world
                grabbers = {
                    'our_defender': self.world.our_defender.catcher_area,
                    'our_attacker': self.world.our_attacker.catcher_area
                }

                # Information about states
                attackerState = (self.planner.current_state, self.planner.current_state)
                defenderState = (self.planner.current_state, self.planner.current_state)

                attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
                defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

                # Use 'y', 'b', 'r' to change color.
                c = waitKey(2) & 0xFF

                # TODO: fix the GUI stuff, i.e. why does it need this actions?
                # also the grabbers above.
                # Maybe just give GUI the world?
                actions = []
                fps = float(counter) / (time.clock() - timer)
                # Draw vision content and actions
            
                self.GUI.draw(
                    frame, model_positions, actions, regular_positions, fps, attackerState,
                    defenderState, attacker_actions, defender_actions, grabbers,
                    our_color=self.color, our_side=self.side, key=c, preprocess=pre_options)
                counter += 1

        except CustomError as e:
            # Print out any exception that occurs
            print e.value()

        finally:
            # Shutdown Robot9, if present
            if self.robot9 is not None:
                self.robot9.stop()
                self.robot9.exit()
                
            # Shutdown Robot10, if present
            if self.robot10 is not None:
                self.robot10.shutdown(self.arduino10)

            # Save updated calibrations for this pitch
            tools.save_colors(self.pitch, self.calibration)


class Arduino:

    def __init__(self, port, rate, timeOut, comms):
        self.serial = None
        self.comms = comms
        self.port = port
        self.rate = rate
        self.timeout = timeOut
        self.last_command = ""
        self.setComms(comms)
        self.increment_command=0

    def setComms(self, comms):
        if comms:
            self.comms = True
            if self.serial is None:
                try:
                    self.serial = serial.Serial(self.port, self.rate, timeout=self.timeout)
                except:
                    print self.port
                    print self.rate
                    print self.timeout
                    print "No Arduino detected!"
                    print "Continuing without comms."
                    self.comms = False
        else:
            self.write('RUN_ENG %d %d\r' % (0, 0))
            self.comms = False

    def write(self, string):
        if self.comms:
            self.increment_command+=1;
            if self.last_command != string or self.increment_command > 5:
                self.increment_command=0;
                print string
                self.last_command = string
                self.serial.write(string)

class Robot_Controller(object):
    """
    Robot_Controller class for robot control.
    """

    def __init__(self):
        """
        Initialise variables
        """
        self.current_speed = 0
        self.last_command = ""

    def execute(self, comm, action):
        """
        Execute robot action.
        """

        # Do whatever actions are specified in the action dict
        # To kick without affecting wheels, don't send 'left_motor' or 'right_motor' at all
        if action is not None:
            if 'left_motor' in action or 'right_motor' in action:
                left_motor = 0
                right_motor = 0
                if 'left_motor' in action:
                    left_motor = int(action['left_motor'])
                if 'right_motor' in action:
                    right_motor = int(action['right_motor'])
                msg = 'RUN_ENG %d %d\r' % (max(min(left_motor, 99), -99), max(min(right_motor, 99), -99))
                comm.write(msg)

            if 'speed' in action:
                # LB: need to decide whether to use this or not
                speed = action['speed']

            if 'kicker' in action and action['kicker'] != 0:
                try:
                    comm.write('RUN_KICK %d\r' % (action['kicker']))
                    comm.write('RUN_KICK %d\r' % (action['kicker']))
                    # Let the kick finish before we tell it what else to do
                except StandardError:
                    pass
            elif 'catcher' in action and action['catcher'] != 0:
                try:
                    comm.write('RUN_CATCH\r')
                except StandardError:
                    pass
            elif 'drop' in action and action['drop'] != 0:
                try:
                    comm.write('DROP\r')
                except StandardError:
                    pass

    def shutdown(self, comm):
        comm.write('RUN_ENG %d %d\r' % (0, 0))
        time.sleep(1)

if __name__ == '__main__':
    '''Entrypoint for the controller. Accepts a few arguments to handle varying conditions and robots being present
    or not.'''
    import argparse

    parser = argparse.ArgumentParser()

    # General Pitch / Side Setup Conditions
    parser.add_argument('-p', '--pitch', type=int, default=0, help="[0] Main pitch (Default), [1] Secondary pitch")
    parser.add_argument('-s', '--side', default='left', help="Our team's side ['left', 'right'] allowed. [Default: left]")
    parser.add_argument(
        '-c', '--color', default='yellow', help="The color of our team ['yellow', 'blue'] allowed. [Default: yellow]")

    # Planner spec files
    parser.add_argument('plannerSpecFiles', nargs='*', type=str, help="The location of the planner finite state machine specification files")

    # Communication Conditions
    parser.add_argument(
        "-a", "--attacker", default=None, help="Serial port for Attacker communication [Omit for no communications]")
    parser.add_argument(
        "-d", "--defender", default=None, help="Serial port for Defender communication [Omit for no communications]")

    args = parser.parse_args()

    # Setup controller with appropriate parameters
    c = Controller(pitch=args.pitch, color=args.color, our_side=args.side, attack_port=args.attacker, args.plannerSpecFiles, defend_port=args.defender).run()