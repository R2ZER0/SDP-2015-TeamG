from planning.models import Vector
from copy import deepcopy
from math import atan2, pi, hypot
import math

class Postprocessing(object):
    '''
    Postprocessing performs analysis of the PitchObjects after the vision module
    has performed it's locating. This generally calculates angle and velocity
    measurements.
    '''

    def __init__(self):
        '''
        Initialise the postprocessing at time 0 with default positions for all.
        '''
        self._vectors = {}
        self._vectors['ball'] = {'vec': Vector(0, 0, 0, 0), 'time': 0}
        self._vectors['our_attacker'] = {'vec': Vector(0, 0, 0, 0), 'time': 0}
        self._vectors['their_attacker'] = {'vec': Vector(0, 0, 0, 0), 'time': 0}
        self._vectors['our_defender'] = {'vec': Vector(0, 0, 0, 0), 'time': 0}
        self._vectors['their_defender'] = {'vec': Vector(0, 0, 0, 0), 'time': 0}
        self._time = 0

    def analyze(self, vector_dict):
        '''
        This method analyzes current positions and collects them into a dictionary
        before returning them.

        :param vector_dict: An old dictionary containing vectors for each PitchObject
        :returns: A new vector dictionary containing analysed data for this frame.
        '''

        # Increase our time for each analyze call
        self._time += 1

        new_vector_dict = {}
        for name, info in vector_dict.iteritems():
            if name == 'ball':
                new_vector_dict[name] = self.analyze_ball(info)
            else:
                new_vector_dict[name] = self.analyze_robot(name, info)
        return new_vector_dict

    def analyze_ball(self, info):
        '''
        Calculates the new angle and velocity of the ball using a simple time calculation and
        change in direction from the previous info dictionary.

        :param info: The previous postprocessing dictionary for the ball
        :returns: A new vector dictionary containing x,y,angle,velocity for the ball.
        '''
        if not(info['x'] is None) and not (info['y'] is None):
            
            # Calculate change in the balls movement
            delta_x = info['x'] - self._vectors['ball']['vec'].x
            delta_y = info['y'] - self._vectors['ball']['vec'].y
            
            # Use difference between current time and previous time as the duration of movement
            velocity = hypot(delta_y, delta_x)/(self._time - self._vectors['ball']['time'])
            if velocity == 0:
                angle = 0
            else:
                angle = atan2(delta_y, delta_x) % (2*pi)
            self._vectors['ball']['vec'] = Vector(info['x'], info['y'], angle, velocity)
            self._vectors['ball']['time'] = self._time
            return Vector(int(info['x']), int(info['y']), angle, velocity)
        else:
            return deepcopy(self._vectors['ball']['vec'])

    def analyze_robot(self, key, info):
        '''
        Calculates the angle and velocity of a robot given the new frame and the change in time.

        :param key: A robot key: ['our_attacker', 'their_attacker', 'our_defender', 'their_defender']
        :param info: The previous vector dictionary for this robot.
        '''
        if not(info['x'] is None) and not(info['y'] is None) and not(info['angle'] is None):

            # Reject angle if it has altered further than it should have
            old_angle = self._vectors[key]['vec'].angle
            robot_angle = info['angle']

            diff = min((2 * pi) - abs(old_angle - robot_angle), abs(old_angle - robot_angle))
            
            #: pi/2 is the maximum angle turn allowed in a single frame before we reject this as a vision
            #: error
            if self._vectors[key]['time'] > 10 and diff > pi/2:
                info['angle'] = old_angle
                robot_angle = old_angle
                print 'Rejected new angle'

            delta_x = info['x'] - self._vectors[key]['vec'].x
            delta_y = info['y'] - self._vectors[key]['vec'].y

            # Calculate the angle of the delta vector relative to (1, 0)
            delta_angle = atan2(delta_y, delta_x)

            # Offset the angle if negative, we only want positive values
            delta_angle = delta_angle % (2*math.pi)

            velocity = hypot(delta_y, delta_x)/(self._time - self._vectors[key]['time'])

            self._vectors[key]['vec'] = Vector(info['x'], info['y'], info['angle'], velocity)
            self._vectors[key]['time'] = self._time
            
            return Vector(info['x'], info['y'], info['angle'], velocity)
        else:
            return deepcopy(self._vectors[key]['vec'])
