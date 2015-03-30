from action import Action
from planning.tasks import AcquireBall, MoveToPoint
from planning.planner import Planner
from planning.models import World
from vision.vision import Vision, Camera, GUI
import vision.tools as tools
from preprocessing.preprocessing import Preprocessing
from postprocessing.postprocessing import Postprocessing
from simulator.simulator import Simulator, SimulatedAction, SimulatedCamera
from simulator.entities import SimulatedRobot, SimulatedBall
from cv2 import waitKey
import cv2
import math
import serial
import warnings
import time
import numpy as np
from pymunk import Vec2d

warnings.filterwarnings("ignore", category=DeprecationWarning)

class Controller:
    '''Simulation Controller. Constructs a simulation and displays a minimal GUI showing the
    Vision system.
    '''

    #: Constant, the index into :mod:`vision.tools.PITCHES` for the Simulator
    SIM_PITCH = 2

    #: Our Simulator instance
    simulator = None

    #: Our camera, for drawing the current state
    camera = None

    def __init__(self, color='yellow', our_side='left', our_role='attacker'):
        '''Initialises the Simulation controller

        :param color: Which color plate to run as; only impact is aesthetic.
        :param our_side: Designates our side, left represents left of the camera frame, core \
                    distinguishing feature for our / their functions.
        :param our_role: Designates the role we're playing as, one of ['attacker', 'defender'].
        '''
        assert color in ['yellow', 'blue']
        assert our_side in ['left', 'right']
        assert our_role in ['attacker', 'defender']

        self.simulator = Simulator(our_side, our_role, color)
        self.camera = SimulatedCamera(self.simulator)
        self.robot = SimulatedAction(self.simulator.control_robot)

        frame = self.camera.get_frame()
        center_point = self.camera.get_adjusted_center(frame)

        # Set up our vision module
        self.calibration = self._get_sim_colors()
        self.vision = Vision(
            pitch=self.SIM_PITCH, color=color, our_side=our_side,
            frame_shape=frame.shape, frame_center=center_point,
            calibration=self.calibration)

        # Set up postprocessing for vision
        self.postprocessing = Postprocessing()

        # Set up world
        self.world = World(our_side, self.SIM_PITCH)

        # Set up main planner
        #self.planner = Planner(world=self.world, robot=self.robot, role=our_role)

        # Set up GUI
        self.GUI = GUI(calibration=self.calibration, pitch=self.SIM_PITCH)

        self.color = color
        self.side = our_side
        self.role = our_role

        self.preprocessing = Preprocessing()

    def run(self):
        '''Main loop of the Controller. Executes a continuous loop doing the following:
        
        * Retrieve Camera frame.
        * Perform preprocessing fixes to the frame
        * Pass frame to Vision module to locate positions of objects of interest
        * Perform postprocessing analysis on the positions
        * Update our World state with new positions
        * Contact our Planner to find out next tasks
        * Redraw the GUI with updated information.
        '''
        counter = 1L
        timer = time.clock()
        tracker = time.clock()

        c = True

        move_handle = None
        kick_handle = None
        catch_handle = None

        while c != 27:  # the ESC key

            self.simulator.update(1)

            frame = self.camera.get_frame()
            pre_options = self.preprocessing.options

            # Apply preprocessing methods toggled in the UI
            preprocessed = self.preprocessing.run(frame, pre_options)
            frame = preprocessed['frame']
            if 'background_sub' in preprocessed:
                cv2.imshow('bg sub', preprocessed['background_sub'])

            # Find object positions
            # model_positions have their y coordinate inverted

            model_positions, regular_positions = self.vision.locate(frame)
            model_positions = self.postprocessing.analyze(model_positions)

            # Update world state
            self.world.update_positions(model_positions)

            #self.planner.plan()

            # Information about the grabbers from the world
            grabbers = {
                'our_defender': self.world.our_defender.catcher_area,
                'our_attacker': self.world.our_attacker.catcher_area
            }

            # Information about states
            attackerState = ('','') #(self.planner._current_state, self.planner._current_state)
            defenderState = ('','') #(self.planner._current_state, self.planner._current_state)

            attacker_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}
            defender_actions = {'left_motor' : 0, 'right_motor' : 0, 'speed' : 0, 'kicker' : 0, 'catcher' : 0}

            # Use 'y', 'b', 'r' to change color.
            c = waitKey(2) & 0xFF

            # Handle the key press in the simualtor; return result indicates if
            # this key press has been used
            key_handled = self.simulator.key_pressed(c)

            actions = []
            fps = float(counter) / (time.clock() - timer)

            # Draw vision content and actions
            self.GUI.draw(
                frame, model_positions, actions, regular_positions, fps, attackerState,
                defenderState, attacker_actions, defender_actions, grabbers,
                our_color=self.color, our_side=self.side, key=(-1 if key_handled else c), preprocess=pre_options)
            counter += 1
        
        if self.robot is not None:
            self.robot.stop()

    def _get_sim_colors(self):
        '''Retrieves colour calibrations for the Simulator, based upon the colours assigned in :mod:`simulator.simulator`.
        '''
        colors = {}
        colors['blue'] = {'max': np.array([120,255,255]), 'min': np.array([120,255,255]), 'contrast': 0, 'blur': 0}
        colors['plate'] = {'max': np.array([60,255,255]), 'min': np.array([60,255,255]), 'contrast': 0, 'blur': 0}
        colors['yellow'] = {'max': np.array([30,255,255]), 'min': np.array([30,255,255]), 'contrast': 0, 'blur': 0}
        colors['dot'] = {'max': np.array([0,0,0]), 'min': np.array([0,0,0]), 'contrast': 0, 'blur': 0}
        colors['red'] = {'max': np.array([0,255,255]), 'min': np.array([0,255,255]), 'contrast': 0, 'blur': 0}

        return colors

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()

    parser.add_argument('-s', '--side', default='left', help="Our team's side ['left', 'right'] allowed. [Default: left]")
    parser.add_argument(
        '-r', '--role', default='attacker', help="Our controlled robot's role ['attacker', 'defender'] allowed. [Default: attacker]")
    parser.add_argument(
        '-c', '--color', default='yellow', help="The color of our team ['yellow', 'blue'] allowed. [Default: yellow]")
    
    args = parser.parse_args()

    Controller(color=args.color, our_side=args.side, our_role=args.role).run()
