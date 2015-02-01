from vision.vision import Vision, Camera, GUI
from planning.planner import Planner
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
import vision.tools as tools
from cv2 import waitKey
import cv2
import serial
import warnings
import time
from planning.models import World
from action import Action
from math import cos, sin, hypot, pi, atan2

warnings.filterwarnings("ignore", category=DeprecationWarning)


class Controller:
    """
    Primary source of robot control. Ties vision and planning together.
    """

    def __init__(self, pitch, color, our_side, video_port=0, comm_port='/dev/ttyACM0', comms=1, angle=0):
        """
        Entry point for the SDP system.

        Params:
            [int] video_port                port number for the camera
            [string] comm_port              port number for the arduino
            [int] pitch                     0 - main pitch, 1 - secondary pitch
            [string] our_side               the side we're on - 'left' or 'right'
            *[int] port                     The camera port to take the feed from
            *[Robot_Controller] attacker    Robot controller object - Attacker Robot has a RED
                                            power wire
            *[Robot_Controller] defender    Robot controller object - Defender Robot has a YELLOW
                                            power wire
        """
        assert pitch in [0, 1]
        assert color in ['yellow', 'blue']
        assert our_side in ['left', 'right']

        self.pitch = pitch

        # Set up the Arduino communications
        self.arduino = Arduino(comm_port, 115200, 1, comms)

        # Set up camera for frames
        self.camera = Camera(port=video_port, pitch=self.pitch)
        frame = self.camera.get_frame()
        center_point = self.camera.get_adjusted_center(frame)

        # Set up vision
        self.calibration = tools.get_colors(pitch)
        self.vision = Vision(
            pitch=pitch, color=color, our_side=our_side,
            frame_shape=frame.shape, frame_center=center_point,
            calibration=self.calibration)

        # Set up postprocessing for vision
        self.postprocessing = Postprocessing()

        # Set up world
        self.world = World(our_side, pitch)

        # Set up main planner
        self.planner = Planner(our_side=our_side, pitch_num=self.pitch, world=self.world)

        # Set up GUI
        self.GUI = GUI(calibration=self.calibration, arduino=self.arduino, pitch=self.pitch)

        self.color = color
        self.side = our_side

        self.preprocessing = Preprocessing()

        self.attacker = Attacker_Controller()
        self.defender = Defender_Controller()

        self.angle = angle

        self.comm = serial.Serial("/dev/ttyACM0", 115200)
        #comm = None
        self.robot = Action(self.comm)

        self.angleReached = False

    def wow(self):
        """
        Ready your sword, here be dragons.
        """
        counter = 1L
        timer = time.clock()
        try:
            c = True
            while c != 27:  # the ESC key

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

                # Find appropriate action
                #self.planner.update_world(model_positions)
                self.world.update_positions(model_positions)

                #########
                # Tests #
                #########
                print "Requested angle: " + self.angle
                print "Current angle: " + self.world.our_attacker.angle

                if abs(float(self.angle)-float(self.world.our_attacker.angle))<0.5:
                    self.angleReached = True

                if not self.angleReached:
                    self.robot._send_run([100,0,0])
                    print "Please turn"
                else:
                    self.robot.stop()
                #####


                # Use 'y', 'b', 'r' to change color.
                c = waitKey(2) & 0xFF
                actions = []
                fps = float(counter) / (time.clock() - timer)
                # Draw vision content and actions


                counter += 1

        except:
            if self.defender is not None:
                self.defender.shutdown(self.arduino)
            if self.attacker is not None:
                self.attacker.shutdown(self.arduino)
            raise

        finally:
            # Write the new calibrations to a file.
            tools.save_colors(self.pitch, self.calibration)
            if self.attacker is not None:
                self.attacker.shutdown(self.arduino)
            if self.defender is not None:
                self.defender.shutdown(self.arduino)


class Robot_Controller(object):
    """
    Robot_Controller superclass for robot control.
    """

    def __init__(self):
        """
        Connect to Brick and setup Motors/Sensors.
        """
        self.current_speed = 0

    def shutdown(self, comm):
        # TO DO
            pass


class Defender_Controller(Robot_Controller):
    """
    Defender implementation.
    """

    def __init__(self):
        """
        Do the same setup as the Robot class, as well as anything specific to the Defender.
        """
        super(Defender_Controller, self).__init__()

    def execute(self, comm, action):
        """
        Execute robot action.
        """

        if 'turn_90' in action:
            comm.write('D_RUN_ENGINE %d %d\n' % (0, 0))
            time.sleep(0.2)
            comm.write('D_RUN_SHOOT %d\n' % int(action['turn_90']))
            time.sleep(2.2)

        #print action
        left_motor = int(action['left_motor'])
        right_motor = int(action['right_motor'])
        speed = action['speed']

        comm.write('D_SET_ENGINE %d %d\n' % (speed, speed))
        comm.write('D_RUN_ENGINE %d %d\n' % (left_motor, right_motor))
        if action['kicker'] != 0:
            try:
                comm.write('D_RUN_KICK\n')
                time.sleep(0.5)
            except StandardError:
                pass
        elif action['catcher'] != 0:
            try:
                comm.write('D_RUN_CATCH\n')
            except StandardError:
                pass

    def shutdown(self, comm):
        comm.write('D_RUN_KICK\n')
        comm.write('D_RUN_ENGINE %d %d\n' % (0, 0))


class Attacker_Controller(Robot_Controller):
    """
    Attacker implementation.
    """

    def __init__(self):
        """
        Do the same setup as the Robot class, as well as anything specific to the Attacker.
        """
        super(Attacker_Controller, self).__init__()

    def execute(self, comm, action):
        """
        Execute robot action.
        """
        if 'turn_90' in action:
            comm.write('A_RUN_ENGINE %d %d\n' % (0, 0))
            time.sleep(0.2)
            comm.write('A_RUN_SHOOT %d\n' % int(action['turn_90']))
            # time.sleep(1.2)
        else:
            left_motor = int(action['left_motor'])
            right_motor = int(action['right_motor'])
            speed = int(action['speed'])
            comm.write('A_SET_ENGINE %d %d\n' % (speed, speed))
            comm.write('A_RUN_ENGINE %d %d\n' % (left_motor, right_motor))
            if action['kicker'] != 0:
                try:
                    comm.write('A_RUN_KICK\n')
                except StandardError:
                    pass
            elif action['catcher'] != 0:
                try:
                    comm.write('A_RUN_CATCH\n')
                except StandardError:
                    pass

    def shutdown(self, comm):
        comm.write('A_RUN_KICK\n')
        comm.write('A_RUN_ENGINE %d %d\n' % (0, 0))


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


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("pitch", help="[0] Main pitch, [1] Secondary pitch")
    parser.add_argument("side", help="The side of our defender ['left', 'right'] allowed.")
    parser.add_argument("color", help="The color of our team - ['yellow', 'blue'] allowed.")
    parser.add_argument(
        "-n", "--nocomms", help="Disables sending commands to the robot.", action="store_true")
    parser.add_argument("angle", help="Angle to turn to [0-2pi]")

    args = parser.parse_args()
    if args.nocomms:
        c = Controller(
            pitch=int(args.pitch), color=args.color, our_side=args.side, comms=0, angle=args.angle).wow()
    else:
        c = Controller(
            pitch=int(args.pitch), color=args.color, our_side=args.side, angle=args.angle).wow()
