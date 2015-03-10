import pdb
from Polygon.cPolygon import Polygon
from math import acos, cos, sin, hypot, pi, atan2, sqrt
from vision import tools

'''Models contains all data structures pertaining to the positioning and movement
of objects in the physical world.

    ROBOT_WIDTH     Narrower dimension of the robot
    ROBOT_LENGTH    Larger dimension of the robot
    ROBOT_HEIGHT    Vertical dimension of the robot

    BALL_WIDTH      Radius of the ball
    BALL_LENGTH     Radius of the ball
    BALL_HEIGHT     Radius of the ball

    GOAL_WIDTH      Distance between goal posts on the Pitch
    GOAL_LENGTH     Depth of the goal
    GOAL_HEIGHT     Vertical length of the goal
'''     

ROBOT_WIDTH = 30
ROBOT_LENGTH = 45
ROBOT_HEIGHT = 10

BALL_WIDTH = 5
BALL_LENGTH = 5
BALL_HEIGHT = 5

GOAL_WIDTH = 140
GOAL_LENGTH = 1
GOAL_HEIGHT = 10

class Coordinate(object):

    def __init__(self, x, y):
        '''
        :param x: An assignment for x, not None
        :param y: An assignment for y, not None
        :raises ValueError: If either x or y are None
        '''
        if x == None or y == None:
            raise ValueError('Can not initialize to attributes to None')
        else:
            self._x = x
            self._y = y

    @property
    def x(self):
        ''':returns: The x-value for this coordinate.
        '''
        return self._x

    @property
    def y(self):
        ''':returns: The y-value for this coordinate.
        '''
        return self._y

    @x.setter
    def x(self, new_x):
        '''
        :param new_x: New assignment value for x, not None
        :raises ValueError: If the value is None
        '''
        if new_x == None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._x = new_x

    @y.setter
    def y(self, new_y):
        '''
        :param new_y: New assignment value for y, not None
        :raises ValueError: If the value is None
        '''
        if new_y == None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._y = new_y

    def __repr__(self):
        '''Constructs a string representation of this coordinate.
        '''
        return 'x: %s, y: %s\n' % (self._x, self._y)


class Vector(Coordinate):

    def __init__(self, x, y, angle, velocity, angular_velocity=0):
        '''Assigns the given values and constructs a Vector.

        :param x: The x coordinate position, not None
        :param y: The y coordinate position, not None
        :param angle: The angle of this Vector, in radians. The angle must be \
                        not None and in the range [0,2pi]
        :param velocity: The force value for this vector.
        :raises ValueError: If the angle is outwith the range [0,2pi], or if \
                        velocity is negative.
        '''
        super(Vector, self).__init__(x, y)

        # Constrain angle and velocity to valid values
        if angle == None or velocity == None:
            raise ValueError('Cannot initialise angle or velocity attributes to None') 
        elif angle < 0 or angle >= (2*pi):
            raise ValueError('Vector angle initialised outwith range [0,2pi]')
        elif velocity < 0:
            raise ValueError('Vector velocity initialised outwith range: >= 0')
        else:
            self._angle = angle
            self._velocity = velocity
            self._angular_velocity = angular_velocity

    @property
    def angle(self):
        ''':returns: The angle of this vector'''
        return self._angle

    @property
    def velocity(self):
        ''':returns: The velocity of this vector'''
        return self._velocity

    @property
    def angular_velocity(self):
        ''':returns: The angular velocity of this vector'''
        return self._angular_velocity

    @angle.setter
    def angle(self, new_angle):
        ''':param new_angle: New assignment value for angle, not None and within the \
                        range [0,2pi]
        :returns ValueError: If the angle is outwith the range [0,2pi]
        '''
        if new_angle == None or new_angle < 0 or new_angle >= (2*pi):
            raise ValueError('Angle can not be None, also must be between 0 and 2pi')
        self._angle = new_angle

    @velocity.setter
    def velocity(self, new_velocity):
        ''':param new_velocity: A new value for velocity
        :returns ValueError: If the velocity is None or negative.
        '''
        if new_velocity == None or new_velocity < 0:
            raise ValueError('Velocity can not be None or negative')
        self._velocity = new_velocity

    @angular_velocity.setter
    def angular_velocity(self, ang_velocity):
        ''':param ang_velocity: A new value for the angular velocity
        '''
        self._angular_velocity = ang_velocity

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and (self.__dict__ == other.__dict__))

    def __repr__(self):
        return ('x: %s, y: %s, angle: %s, velocity: %s\n' %
                (self.x, self.y,
                 self._angle, self._velocity))


class PitchObject(object):
    '''
    A class that describes an abstract pitch object
    Width measures the front and back of an object
    Length measures along the sides of an object
    '''

    def __init__(self, x, y, angle, velocity, width, length, height, angle_offset=0):
        '''
        :param x: x-position
        :param y: y-position
        :param angle: current angle
        :param velocity: current velocity
        :param width: front and rear dimension, not None and > 0
        :param length: side dimensions, not None and > 0
        :param height: vertical dimension, not None and > 0
        :param angle_offset: convenient angle offset, used to adjust any specified \
                            angle by this amount. default: 0
        '''
        if width is None or length is None or height is None:
            raise ValueError('Object dimensions must be non-None')
        elif width <= 0 or length <= 0 or height <= 0:
            raise ValueError('Object dimensions must be positive')
        else:
            self._width = width
            self._length = length
            self._height = height
            self._angle_offset = angle_offset
            self._vector = Vector(x, y, angle, velocity)

    @property
    def width(self):
        ''':returns: The width of this object
        '''
        return self._width

    @property
    def length(self):
        ''':returns: The length of this object
        '''
        return self._length

    @property
    def height(self):
        ''':returns: The height (vertical) of this object
        '''
        return self._height

    @property
    def angle_offset(self):
        return self._angle_offset

    @property
    def angle(self):
        return self._vector.angle

    @property
    def velocity(self):
        return self._vector.velocity

    @property
    def angular_velocity(self):
        return self._vector.angular_velocity

    @property
    def x(self):
        return self._vector.x

    @property
    def y(self):
        return self._vector.y

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, new_vector):
        if new_vector == None or not isinstance(new_vector, Vector):
            raise ValueError('The new vector can not be None and must be an instance of a Vector')
        else:
            self._vector = Vector(new_vector.x, new_vector.y, new_vector.angle - self._angle_offset, new_vector.velocity, new_vector.angular_velocity)

    def get_generic_polygon(self, width, length):
        '''Retrieves a polygon describing the generic shape of this object and
        rotated to correspond to our angle.

        :param width: The robot width (front and rear dimensions)
        :param length: The robot length (side dimensions)
        '''
        front_left = (self.x + length/2, self.y + width/2)
        front_right = (self.x + length/2, self.y - width/2)
        back_left = (self.x - length/2, self.y + width/2)
        back_right = (self.x - length/2, self.y - width/2)
        poly = Polygon((front_left, front_right, back_left, back_right))
        poly.rotate(self.angle, self.x, self.y)
        return poly[0]

    def get_polygon(self):
        '''
        Returns 4 edges of a rectangle bounding the current object in the
        following order: front left, front right, bottom left and bottom right.
        '''
        return self.get_generic_polygon(self.width, self.length)

    def __repr__(self):
        return ('x: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self.x, self.y,
                 self.angle, self.velocity, (self.width, self.length, self.height)))


class Robot(PitchObject):

    def __init__(self, zone, x, y, angle, velocity, width=ROBOT_WIDTH, length=ROBOT_LENGTH, height=ROBOT_HEIGHT, angle_offset=0):
        '''Sets up a new Robot instance with the given parameters.

        .. note::
            Initialises the Robot with the catcher being 'open'
        '''
        super(Robot, self).__init__(x, y, angle, velocity, width, length, height, angle_offset)
        self._zone = zone
        self._catcher = 'closed'

    @property
    def zone(self):
        ''':returns: The zone this Robot belongs to, as an integer index into :func:`planning.models.Pitch.zones`
        '''
        return self._zone

    @property
    def catcher_area(self):
        '''Calculates and returns a polygon representing the catcher area at the
        front of the robot. Rotates the polygon to the robot's angle.

        :returns: A :class:`Polygon.cPolygon.Polygon` object.
        '''
        front_left = (self.x + self._catcher_area['front_offset'] + self._catcher_area['height'], self.y + self._catcher_area['width']/2.0)
        front_right = (self.x + self._catcher_area['front_offset'] + self._catcher_area['height'], self.y - self._catcher_area['width']/2.0)
        back_left = (self.x + self._catcher_area['front_offset'], self.y + self._catcher_area['width']/2.0)
        back_right = (self.x + self._catcher_area['front_offset'], self.y - self._catcher_area['width']/2.0)
        area = Polygon((front_left, front_right, back_left, back_right))
        area.rotate(self.angle, self.x, self.y)
        return area

    @catcher_area.setter
    def catcher_area(self, area_dict):
        '''Assigns a new catcher area to the Robot. 

        :returns: area_dict   A dictionary containing the following keys: 
                - width         Dimension parallel to the front/rear of the robot
                - height        Dimension parallel to the sides of the robot
                - front_offset  Deviation from center of the robot    
        '''
        self._catcher_area = area_dict

    @property
    def catcher(self):
        ''':returns: Returns the catcher state, which should be one of ['closed', 'open']
        '''
        return self._catcher

    @catcher.setter
    def catcher(self, new_position):
        '''Sets a new position for the catcher. 

        :param new_position: Value in ['open', 'closed']
        '''
        assert new_position in ['open', 'closed']
        self._catcher = new_position

    def can_catch_ball(self, ball):
        ''':returns: True if the ball's center falls within the catcher area.
        '''
        box = self.catcher_area.boundingBox()
        return box[0] <= ball.x <= box[1] and box[2] <= ball.y <= box[3]

    def has_ball(self, ball):
        ''':returns: True if the ball is within the catcher area and our catcher is closed.
        '''
        return (self._catcher == 'closed') and self.can_catch_ball(ball)

    def get_rotation_to_point(self, x, y):        
        ''':returns: An angle to which this robot should rotate to be facing the given point, \
            within the range **[-pi,pi]**
        '''
        delta_x = x - self.x
        delta_y = y - self.y
        displacement = hypot(delta_x, delta_y)
        if displacement == 0:
            theta = 0
        else:
            theta = atan2(delta_y, delta_x) - atan2(sin(self.angle), cos(self.angle))
            if theta > pi:
                theta -= 2*pi
            elif theta < -pi:
                theta += 2*pi
        assert -pi <= theta <= pi
        return theta

    def get_displacement_to_point(self, x, y):
        ''':returns: The displacement between our robot's center and the given x,y position.
        '''
        delta_x = x - self.x
        delta_y = y - self.y
        displacement = hypot(delta_x, delta_y)
        return displacement

    def get_direction_to_point(self, x, y):
        ''':returns: A tuple, containing:
                * The displacement to the given x,y position
                * The angle between our robot and the given x,y position
        '''
        return self.get_displacement_to_point(x, y), self.get_rotation_to_point(x, y)

    def get_pass_path(self, target):
        '''Calculates a polygon representing the passing 'channel' between our robot and the 
        given target. This channel is a Polygon instance constructed between the front
        of our robot and the front of the target object.

        :returns: A :class:`Polygon.cPolygon.Polygon` instance containing the above defined area.
        '''
        robot_poly = self.get_polygon()
        target_poly = target.get_polygon()

        # [0] index is Front-left of a Robot, [1] index is Front-Right of a Robot.
        return Polygon([robot_poly[0], robot_poly[1], target_poly[0], target_poly[1]])

    def __repr__(self):
        return ('zone: %s\nx: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self._zone, self.x, self.y,
                 self.angle, self.velocity, (self.width, self.length, self.height)))


class Ball(PitchObject):
    '''
    The Ball is a simple PitchObject with predefined constants for dimensions. Represents
    our ball on the Pitch.
    '''

    def __init__(self, x, y, angle, velocity):
        super(Ball, self).__init__(x, y, angle, velocity, BALL_WIDTH, BALL_LENGTH, BALL_HEIGHT)


class Goal(PitchObject):
    '''
    The Goal class represents a goal in the Pitch and is a PitchObject with the zone
    property to allow querying of the goal's zone.
    '''

    def __init__(self, zone, x, y, angle):
        super(Goal, self).__init__(x, y, angle, 0, GOAL_WIDTH, GOAL_LENGTH, GOAL_HEIGHT)
        self._zone = zone

    @property
    def zone(self):
        ''':returns: Returns the zone this goal belongs to, as an integer index into :func:`planning.models.Pitch.zones` 
        '''
        return self._zone

    def __repr__(self):
        return ('zone: %s\nx: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self._zone, self.x, self.y, self.angle, self.velocity, (self.width, self.length, self.height)))


class Pitch(object):
    '''
    The Pitch class represents the full physical pitch and separates the pitch into zones based upon the
    croppings provided in croppings.json. Zones are specified as Polygon objects.
    '''

    def __init__(self, pitch_num):
        '''Sets up a new Pitch with the given number. Retrieves zones using tools.py get_croppings, thus pitch_num
        must be a valid pitch number.

        :param pitch_num: A number within [0,1,2], with 0: main area pitch, 1: side area pitch, 2: simulator pitch.
        '''

        # Retrieve croppings from croppings.json with the given pitch key
        config_json = tools.get_croppings(pitch=pitch_num)

        # Retrieve width and height as the difference between the x,y extremes
        self._width = max([point[0] for point in config_json['outline']]) - min([point[0] for point in config_json['outline']])
        self._height = max([point[1] for point in config_json['outline']]) - min([point[1] for point in config_json['outline']])
        
        # Invert the y due to differences with CV's y coordinates
        self._zones = []
        self._zones.append(Polygon([(x, self._height - y) for (x, y) in config_json['Zone_0']]))
        self._zones.append(Polygon([(x, self._height - y) for (x, y) in config_json['Zone_1']]))
        self._zones.append(Polygon([(x, self._height - y) for (x, y) in config_json['Zone_2']]))
        self._zones.append(Polygon([(x, self._height - y) for (x, y) in config_json['Zone_3']]))

    def is_within_bounds(self, robot, x, y):
        '''Convenience function, checks whether the given x,y position is within the given robot's 
        zone.
 
        :param robot: The robot to check for containment
        :param x: The target x position
        :param y: The target y position

        :returns: True if the robot's zone contains the given x,y position.
        '''
        zone = self._zones[robot.zone]
        return zone.isInside(x, y)

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def zones(self):
        ''':returns: The full list of zones available on this Pitch.
        '''
        return self._zones

    def __repr__(self):
        return str(self._zones)


class World(object):
    '''
    The World class is the high level model describing the entire world through the Pitch model and the 
    PitchObjects contained within. Heavily passed around, this serves as the querying object for many
    different modules.
    '''

    def __init__(self, our_side, pitch_num):
        '''
        Initialises a new World instance, with the given side serving to identify robot positionings
        and pitch number used to pull the correct croppings for the Pitch.

        :param our_side: Either 'left' or 'right', used to identify robot positions
        :param pitch_num: Value in the range [0,1,2], with 0: main pitch, 1: side pitch, 2: simulator.
        '''
        assert our_side in ['left', 'right']

        self._pitch = Pitch(pitch_num)

        self._our_side = our_side
        self._their_side = 'left' if our_side == 'right' else 'right'

        self._ball = Ball(0, 0, 0, 0)
        self._robots = []
        self._robots.append(Robot(0, 0, 0, 0, 0))
        self._robots.append(Robot(1, 0, 0, 0, 0))
        self._robots.append(Robot(2, 0, 0, 0, 0))
        self._robots.append(Robot(3, 0, 0, 0, 0))

        # Define catchment areas for our robots. If the ball enters these areas then our robot
        # believes it can catch the ball. Width is parallel to front/rear and height is parallel
        # to the sides of our robot.
        self.our_defender.catcher_area = {'width' : 21, 'height' : 23, 'front_offset' : 20}
        self.our_attacker.catcher_area = {'width' : 21, 'height' : 23, 'front_offset' : 20}

        # Calculate the goals as being in fixes positions halfway up the pitch. The latter
        # goal 'faces' towards the left of the pitch.
        self._goals = []
        self._goals.append(Goal(0, 0, self._pitch.height/2.0, 0))
        self._goals.append(Goal(3, self._pitch.width, self._pitch.height/2.0, pi))

        # Initially true, this ensures we only receive one warning for the sides being wrong.
        self._allowWarning = True

    @property
    def our_attacker(self):
        ''':returns: Returns the attacker belonging to our side, as a Robot model.
        '''
        return self._robots[2] if self._our_side == 'left' else self._robots[1]

    @property
    def their_attacker(self):
        ''':returns: Returns the opposing attacker, as a Robot model.
        '''
        return self._robots[1] if self._our_side == 'left' else self._robots[2]

    @property
    def our_defender(self):
        ''':returns: Returns the defender belonging to our side, as a Robot model.
        '''
        return self._robots[0] if self._our_side == 'left' else self._robots[3]

    @property
    def their_defender(self):
        ''':returns: Returns the opposing defender, as a Robot model.
        '''
        return self._robots[3] if self._our_side == 'left' else self._robots[0]

    @property
    def ball(self):
        ''':returns: Returns the Ball model representing the ball in the world.
        '''
        return self._ball

    @property
    def our_goal(self):
        ''':returns: Returns a Goal model representing our goal on the Pitch.
        '''
        return self._goals[0] if self._our_side == 'left' else self._goals[1]

    @property
    def their_goal(self):
        ''':returns: Returns a Goal model representing the opposing goal on the Pitch.
        '''
        return self._goals[1] if self._our_side == 'left' else self._goals[0]

    @property
    def pitch(self):
        ''':returns: Returns the Pitch model this world contains.
        '''
        return self._pitch

    def update_positions(self, pos_dict):
        '''Receives a dictionary containing vectors for each of the PitchObjects
        in the scene and updates the models with the new positionings.

        :param pos_dict: A dictionary containing a vector for each of: \
                ['our_attacker', 'their_attacker', 'our_defender', 'their_defender', 'ball']
        '''
        self.our_attacker.vector = pos_dict['our_attacker']
        self.their_attacker.vector = pos_dict['their_attacker']
        self.our_defender.vector = pos_dict['our_defender']
        self.their_defender.vector = pos_dict['their_defender']
        self.ball.vector = pos_dict['ball']

        # Only warn once
        if self._allowWarning:

            if (self._our_side == 'left' and not(self.our_defender.x < self.their_attacker.x
                < self.our_attacker.x < self.their_defender.x)):
                print "WARNING: The sides are probably wrong!"
                self._allowWarning = False

            if (self._our_side == 'right' and not(self.our_defender.x > self.their_attacker.x
                > self.our_attacker.x > self.their_defender.x)):
                print "WARNING: The sides are probably wrong!"
                self._allowWarning = False
                # 

