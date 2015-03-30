import pdb
from math import tan, pi, hypot, log
from planning.collisions import get_avoidance
from pyparsing import *
from planning.models import Robot, FSM
from Polygon import *
from Polygon.Utils import pointList


def createFSM(parsedInput, sourceFilePath):
    """Takes the results of a successful parse and builds and returns an FSM object adhering to 
    the specs in the given config file."""

    # parsedInput[0][2][1] contains the string read which represents the machine alphabet
    # We create a list, with useless characters like "," stripped
    alphabet = [x for x in parsedInput[0][2][1] if x != ","]

    # parsedInput[0][1][1] contains the string read which represents the machine name
    name = parsedInput[0][1][1]

    # parsedInput[0][4][1] contains the string read which represents the machine start state
    startState = parsedInput[0][4][1]

    # parsedInput[0][5][1] contains the string read which represents the machine final state
    finalState = parsedInput[0][5][1]

    # Create the list which we'll populate with transition tuples of the form
    # (currState, letter1, letter2,..., [TaskName, arg1,arg2,..], newState)
    transitions = []

    # parsedInput[1][1:] contains the raw transitions. "x.asList() if type(x) == ParseResults else x" is
    # necessary because of the way PyParsing returns parse results. We want things as a list, so we call
    # asList()
    for trans in parsedInput[1][1:]:
        t = tuple([(x.asList() if type(x) == ParseResults else x) for x in trans if x != "<" and x != ">" and x != ","])
        transitions.append(t)
    
    # parsedInput[0][3][1] contains the raw state list
    # We create a tuple of the form (state1, state2,...), filtering out useless chars like
    # ","
    states = tuple([x for x in parsedInput[0][3][1] if x != ","])

    # Create the string we'll use to contain the lambdas found during the parse
    lambdas = []
    for lambdaStmt in parsedInput[2][1:]:
        t = ''.join(map(str, lambdaStmt))
        lambdas.append(t)

    # Build the actual letter:lambda dictionary
    dic = consumeLambdas(lambdas)

    # Perform sanity checks on what we've read 
    checkAlphabet(alphabet, lambdas)
    checkTransitions(alphabet, transitions)
    checkLambdas(alphabet, dic)

    parseError = False
    parseError = checkAlphabet(alphabet, lambdas, sourceFilePath)
    parseError = checkTransitions(alphabet, transitions, sourceFilePath)
    parseError = checkLambdas(alphabet, dic, sourceFilePath)

    if parseError:
        return False
    else:
        return FSM(name, alphabet, states, startState, finalState, transitions, dic, lambdas)

    # If we're here, everything is valid and we can go ahead and create the FSM
    # fsm = FSM(name, alphabet, states, startState, finalState, transitions, dic, lambdas)
    # return fsm

def createConfigGrammar():
    """Here, we define the PyParsing grammar to use when executing a parse run of an FSM
    input specification script"""

    # Define the keywords
    inAlphKWD     = Literal("inAlph")
    finalSKWD     = Literal("finalState")
    statesKWD     = Literal("states")
    initSKWD      = Literal("initialState")
    nameKWD       = Literal("name")
    lambdaSecKWD  = Literal("lambdaConditions")
    machineSecKWD = Literal("machineParams")
    transitionsKWD= Literal("transitions")
    lambdaKWD     = Literal("lambda ")
    worldKWD      = Literal("world")
    exisitingKWD  = Literal("EXISTING")
    plannerKWD    = Literal("planner")

    # Define the various single-character tokens we wish to recognise
    separator     = Literal(",")
    leftAngBrkt   = Literal("<")
    rightAngBrkt  = Literal(">")
    assign        = Literal("=")
    sMark         = Literal("\"")
    colon         = Literal(":")
    leftSqBrkt    = Literal("[")
    rightSqBrkt   = Literal("]")

    # A 'state' is an alphanumeric 'word'
    state         = Word(alphanums)

    # A task invocation is either of the form '[EXISTING]' or '[TaskName, arg1, arg2, ...]'
    taskInvocation= Group(leftSqBrkt + exisitingKWD + rightSqBrkt) ^ Group(leftSqBrkt + Word(alphas) + ZeroOrMore(separator + Word(alphanums+"_()")) + rightSqBrkt)

    # An FSM name is an alphanumeric 'word'
    name          = Word(alphanums) 

    # A letter in an FSM alphabet is an alphanumeric 'word'
    letter        = Word(alphanums)

    # A name definition is of the form 'name identifier'
    nameDef       = Group(nameKWD + name)

    # An alphabet definition is of the form  'inAlph letter1,...'
    inAlphabetDef = Group(inAlphKWD + Group(letter + ZeroOrMore(separator + letter)))

    # A states definition list is of the form  'states state1,...'
    statesDef     = Group(statesKWD + Group(state + ZeroOrMore(separator + state)))

    # An initial state definition is of the form  'initialState state1,...'
    initialSDef   = Group(initSKWD + state)

    # A final state definition is of the form  'finalState stateName'
    finalSDef     = Group(finalSKWD + state)

    # A lambda statement is of the form '"alphabetLetter" : lambda planner : code', where code is an alphanumeric 'word', with >< .+-(),\t\n characters
    lambdaStmt    = Group(sMark + letter + sMark + colon + lambdaKWD + plannerKWD + colon + Word(alphanums + ">< .+-(),\t\n"))

    # A transition is of the form '<stateName, letter1, letter2,..., [TaskName, arg1, arg2,...], newState>'
    transition    = Group(leftAngBrkt + state + separator + OneOrMore(letter) + separator + taskInvocation + separator + state + rightAngBrkt)

    machineParamSec= machineSecKWD + nameDef + inAlphabetDef + statesDef + initialSDef + finalSDef
    transitionSec  = transitionsKWD + OneOrMore(transition)
    lambdaSec      = lambdaSecKWD + OneOrMore(lambdaStmt)

    # Config is the 'root' of the parse
    config         = Group(machineParamSec) + Group(transitionSec) + Group(lambdaSec)

    return config

def consumeLambdas(text):
    """This function takes python code representing key:value pairs in string form,
    and for each pair, packages them up into code that will update the dictionay letter-condition
    map.

    Once we have the dictionary update code for each lambda, we cause Python to execute this code_obj
    using exec and return the resulting dictionary
    """
    strippedNewlines = [line.strip("\n") for line in text]

    code = ""
    dictionary={}

    for line in strippedNewlines:
        code = code + "dictionary.update({ " + line + " })\n"
   
    code_obj = compile(code, '<string>', 'exec')
    exec(code_obj)
    return dictionary

def checkTransitions(alphabet, transitions):
    """Ensures there are no transitions which reference unrecognised letters, that is,
    those that have not been speficied in the machine specification. If we enounter an error, we 
    inform the user and terminate."""
    for transition in transitions:
        lettersAppearingInTrans = transition[1:len(transition)-2]   # Extract the letters appearing in a transition table entry
        for candidateLetter in lettersAppearingInTrans:
            """Check each letter valid"""
            if not candidateLetter in alphabet:
                print
                print "Parse error: Found a transition statement which is inconsistent with the FSM alphabet definition - namely:\n'" + str(transition) + "'\nin " + str(sourceFilePath)
                print
                return True

def checkLambdas(alphabet, lambdas):
    """Ensures the letter:lambda pairs do not refer to a non-existent FSM letter. If we encounter an
    error, we inform the user and then terminate"""
    for lambdaStmt, code in lambdas.iteritems():
        if not lambdaStmt in alphabet:
            print "Parse error: Found a lambda statement whose key is inconsistent with the FSM alphabet definition, the offending key being '" + lambdaStmt + "' in " + str(sourceFilePath)
            print
            return True

def checkAlphabet(alphabet, lambdas):
    if not len(alphabet) == len(lambdas):
        print "Parse warning: There are some FSM alphabet letters which do not have corresponding lambda condition triggers in spec file " + str(sourceFilePath)
        print



'''
Utilities contains various convenience functions utilised throughout the
planning module.

Attributes:
    DISTANCE_MATCH_THRESHOLD    Generic distance threshold allowance for considering an object
                                being equal to a position.
    ANGLE_MATCH_THRESHOLD       Generic angle threshold allowance before we consider the object
                                matching an angle.
    BALL_ANGLE_THRESHOLD        Angle threshold for more careful tasks; e.g catching a ball

    MAX_DISPLACEMENT_SPEED      Our maximum power to move at
    MAX_ANGLE_SPEED             Our maximum turning speed

    BALL_VELOCITY               Used by strategies as a lower threshold on when to consider the
                                ball moving.
'''

DISTANCE_MATCH_THRESHOLD = 15
ANGLE_MATCH_THRESHOLD = pi/10
BALL_ANGLE_THRESHOLD = pi/20
CURVE_THRESHOLD = pi/5
CURVE_SPEED_DIFF = 10

FORWARD_SPEED = 80
FORWARD_SPEED_CAREFUL = 40
TURNING_SPEED = 60
TURNING_SPEED_CAREFUL = 40

BALL_VELOCITY = 3

def in_line(robot1, robot2, careful=False):
    '''
    Checks if robot1 and robot2 are horizontally in line
    '''
    if careful:
        threshold = DISTANCE_MATCH_THRESHOLD
    else:
        threshold = DISTANCE_MATCH_THRESHOLD + 10
    return abs(robot1.y - robot2.y) < threshold

def is_facing(robot1, robot2, careful=False):
    '''
    Checks if robot1 is facing robot2
    '''
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD
    angle = robot1.get_rotation_to_point(robot2.x, robot2.y)
    return abs(angle) < threshold

def is_shot_blocked(world, our_robot, their_robot):
    '''Convenience function for checking if one robot's shot is blocked
    by another.

    :param world: The world state to retrieve positions from
    :param our_robot: The Robot model representing our robot
    :param their_robot: The Robot model we wish to check is blocking
    :returns: True, if the shot is blocked by their Robot, False otherwise.
    '''

    # Check for the ball being intersected by their robot
    predicted_y = predict_y_intersection(
        world, their_robot.x, our_robot, full_width=True, bounce=True)
    
    # If direct intersection, return True
    if predicted_y is None:
        return True
    
    print '##########', predicted_y, their_robot.y, their_robot.length
    print abs(predicted_y - their_robot.y) < their_robot.length
    
    # Check if their robot is wide enough to overlap with the predicted
    # y intersection
    return abs(predicted_y - their_robot.y) < their_robot.length

def is_attacker_shot_blocked(world, our_attacker, their_defender):
    '''Check if our is blocked by the other robot by considering if 
    the given robot is parallel to us within some threshold.

    :param world: The world state to retrieve positions from
    :param our_attacker: Our robot to check
    :param their_defender: Their robot to check parallel state.
    :returns: True if the other robot is parallel to us within 40px threshold.
    '''

    # Acceptable distance that the opponent defender can be relative to our
    # shooting position in order for us to have a clear shot.
    distance_threshold = 40

    # Return True if attacker and defender ar close to each other on
    # the y dimension
    return abs(our_attacker.y - their_defender.y) < distance_threshold


def can_score(world, our_robot, their_goal, turn=0):
    '''Determines if our robot is facing, or would be if it turned the given
    amount, the opponent's goal ready to score.

    :param world: The world state
    :param our_robot: The robot we're checking facing direction for
    :param their_goal: The goal we wish to shoot into
    :param turn: [Default: 0], an optional turning offset that allows checking if we \
                    could score if we turned the given amount.

    :returns: True if the ball kicked from our facing direction, with turn offset if given,
        would result in the ball reaching the goal.
    '''

    # Offset the robot angle if need be
    robot_angle = our_robot.angle + turn
    goal_zone_poly = world.pitch.zones[their_goal.zone][0]

    reverse = True if their_goal.zone == 3 else False
    goal_posts = sorted(goal_zone_poly, key=lambda x: x[0], reverse=reverse)[:2]
    # Makes goal be sorted from smaller to bigger
    goal_posts = sorted(goal_posts, key=lambda x: x[1])

    goal_x = goal_posts[0][0]

    robot = Robot(
        our_robot.zone, our_robot.x, our_robot.y, robot_angle % (pi * 2), our_robot.velocity)

    predicted_y = predict_y_intersection(world, goal_x, robot, full_width=True)

    return goal_posts[0][1] < predicted_y < goal_posts[1][1]

def predict_y_intersection(world, predict_for_x, robot, full_width=False, bounce=False):
        '''Calculates where the ball will be placed once it reaches the given x-coordinate
        if it were to be kicked by the robot passed in. Supports calculating the ball
        position even if it were bounced off the wall of the pitch.

        :param world: The world model
        :param predict_for_x: Which x-coordinate to extrapolate the ball's y value for
        :param robot: The robot to judge the kicking angle from
        :param full_width: [Default: False]. If True, uses the full width of the pitch as \
                            the upper and lower y-values for bounce tests, calculated based on \
                            croppings of the pitch.
        :param bounce: [Default: False]. If True, determines if the ball will bounce off the \
                            top/bottom of the Pitch before reaching x-position, and adjust for this.

        :returns: None if the robot is facing the wrong direction for the given x-position, or the predicted \
            y position of the ball if successful.
        '''

        x = robot.x
        y = robot.y
        angle = robot.angle

        # Calculate extremes of the pitch using fixed offset if full_width is True, otherwise, use our goal
        # coordinates as a relative point
        top_y = world._pitch.height - 60 if full_width else world.our_goal.y + (world.our_goal.width/2) - 30
        bottom_y = 60 if full_width else world.our_goal.y - (world.our_goal.width/2) + 30

        # Only predict if the robot is facing the correct location to the predicted x position
        if (robot.x < predict_for_x and not (pi/2 < angle < 3*pi/2)) or (robot.x > predict_for_x and (3*pi/2 > angle > pi/2)):

            if bounce:

                # Check if the change in y given the x displacement would bring us outside of the pitch, indicating
                # a bounce
                if not (0 <= (y + tan(angle) * (predict_for_x - x)) <= world._pitch.height):

                    bounce_pos = 'top' if (y + tan(angle) * (predict_for_x - x)) > world._pitch.height else 'bottom'
                    
                    x += (world._pitch.height - y) / tan(angle) if bounce_pos == 'top' else (0 - y) / tan(angle)
                    y = world._pitch.height if bounce_pos == 'top' else 0
                    
                    angle = (-angle) % (2*pi)

            predicted_y = (y + tan(angle) * (predict_for_x - x))
            # Correcting the y coordinate to the closest y coordinate on the goal line:
            if predicted_y > top_y:
                return top_y
            elif predicted_y < bottom_y:
                return bottom_y
            return predicted_y
        else:
            return None

def grab_ball():
    return {'catcher': 1}


def kick_ball(power):
    return {'kicker': power}


def open_catcher():
    return {'drop': 1}

def has_matched(robot, x=None, y=None, angle=None,
                angle_threshold=ANGLE_MATCH_THRESHOLD, distance_threshold=DISTANCE_MATCH_THRESHOLD):
    '''Convenience function; checks if the given Robot is at the provided x,y position and/or angle
    to within the defined thresholds.

    :param robot: The Robot to check accuracy of
    :param x: The x-position to test, None if testing angle only
    :param y: The y-position to test, None if testing angle only
    :param angle: The angle to test, None if testing position only
    :param angle_threshold: Threshold under which angle will be considered equal, [Default: ANGLE_MATCH_THRESHOLD]
    :param distance_threshold: Threshold under which distance will be considered equal, [Default: DISTANCE_MATCH_THRESHOLD]

    :returns: True if the Robot has matched the desired thresholds to angle and distance.
    '''
    dist_matched = True
    angle_matched = True
    
    if not(x is None and y is None):
        dist_matched = hypot(robot.x - x, robot.y - y) < distance_threshold
    
    if not(angle is None):
        angle_matched = abs(angle) < angle_threshold
    
    return dist_matched and angle_matched


#2015
def get_clear_forward_passing_pos(world, robot, team_mate, obstacle):
    
    if world.pitch.height - obstacle.y > obstacle.y:
        target_y = obstacle.y + ((world.pitch.height - obstacle.y) * 1/2)
    else:
        target_y = obstacle.y - (obstacle.y * 1/2)

    return (robot.x, target_y)

#2015
def current_ball_controller(world):
    ball = world.ball
    if robot_ball_distance(world.their_defender, ball) < 50:
        return world.their_defender
    elif robot_ball_distance(world.their_attacker, ball) < 50:
        return world.their_attacker
    elif robot_ball_distance(world.our_attacker, ball) < 50:
        return  world.our_attacker
    elif robot_ball_distance(world.our_defender, ball) < 50:
        return  world.our_defender

#2015 
def enemy_possess_ball(world):
    """Returns true when a robot on the opposing team has possession of the ball"""
    if robot_ball_distance(world.their_defender, world.ball) < 40:
        return True
    elif robot_ball_distance(world.their_attacker, world.ball) < 40:
        return True

    return False

def robot_ball_distance(robot, ball):
    return robot.get_displacement_to_point(ball.x,ball.y)

#2015
def choose_attacker_destination(world):
    """
    If the attacker isn't already at a point where it could try to score,
    try each point in it's zone to see if we could if we moved there instead.
    If we find such a point, return it. 
    Returns False if we can't find a clear point anywhere (shouldn't really happen),
    and the current position if we can already shoot from there
    """
    OUR_ATTACKER_ZONE=world.our_attacker.zone
    if not(is_attacker_shot_blocked(world, world.our_attacker, world.their_defender)):
        return (world.our_attacker.x, world.our_attacker.y)
    else:
        x = world.our_attacker.x

        # Iterate vertical positions checking if shot would be blocked
        for y in xrange(world.our_attacker.y, 480, 5):

            ghostDefaultAngle=0
            ghostDefaultVel=0
            ghost=Robot(OUR_ATTACKER_ZONE, x, y, ghostDefaultVel, ghostDefaultAngle)
            # Ghost is us if we moved to (x,y)
            if not(is_attacker_shot_blocked(world, ghost, world.their_defender)):
                return (x,y)

        # Iterate vertical positions decreasing to check
        for y in xrange(0, world.our_attacker.y, 5):
            ghostDefaultAngle=0
            ghostDefaultVel=0
            ghost=Robot(OUR_ATTACKER_ZONE, x, y, ghostDefaultVel, ghostDefaultAngle)
            # Ghost is us if we moved to (x,y)
            if not(is_attacker_shot_blocked(world, ghost, world.their_defender)):
                return (x,y)            

    return (None,None)

def calculate_motor_speed(displacement, angle, backwards_ok=False, careful=False):
    '''
    Simplistic view of calculating the speed
    '''

    # LB: potentially calculate careful turning speed based on angle
    # - Slow down as we get closer
    if careful:
        threshold = BALL_ANGLE_THRESHOLD
    else:
        threshold = ANGLE_MATCH_THRESHOLD

    if angle is not None:

        # Multiplier is negative if the motors are to run backwards
        multiplier = 1

        # Check if we can get there with less turning by going backwards
        if backwards_ok and abs(angle) > pi/2:
            moving_backwards = True
            if angle > 0:
                angle = (angle - pi)
            else:
                angle = (angle + pi)
            multiplier = -1
        
        if (careful and abs(angle) < CURVE_THRESHOLD and abs(angle) > threshold and 
                displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD):
            # Move forward curving
            turnSpeedHigh = min(100, TURNING_SPEED_CAREFUL + CURVE_SPEED_DIFF)
            turnSpeedLow = max(0, TURNING_SPEED_CAREFUL - CURVE_SPEED_DIFF)           
            if angle <= 0:
                return {'left_motor': turnSpeedLow, 'right_motor': turnSpeedHigh}
            else:
                return {'left_motor': turnSpeedHigh, 'right_motor': turnSpeedLow}
        elif abs(angle) > threshold:
            if careful:
                turnSpeed = TURNING_SPEED_CAREFUL * multiplier
            else:
                turnSpeed = TURNING_SPEED * multiplier

            if angle <= 0:
                return {'left_motor': -turnSpeed, 'right_motor': turnSpeed}
            else:
                return {'left_motor': turnSpeed, 'right_motor': -turnSpeed}
        elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
            if careful:
                speed = FORWARD_SPEED_CAREFUL * multiplier
            else:
                speed = FORWARD_SPEED * multiplier
            return {'left_motor': speed, 'right_motor': speed}
            
        else:
            return {'left_motor': 0, 'right_motor': 0}
        
    elif displacement is not None and displacement > DISTANCE_MATCH_THRESHOLD:
        if careful:
            speed = FORWARD_SPEED_CAREFUL
        else:
            speed = FORWARD_SPEED
        return {'left_motor': speed, 'right_motor': speed}
    else:
        return {'left_motor': 0, 'right_motor': 0}

def do_nothing():
    return {'left_motor': 0, 'right_motor': 0, 'kicker': 0, 'catcher': 0}

def dist_point_to_line(startX,startY,endX,endY,pntX,pntY):
    ''' 
    Returns the shortest distance between a point (pntX,pntY)
    and a line (NOT line segment) defined by start point (startX,startY) and end point (endX,endY)
    '''
    return abs((endY-startY)*pntX-(endX-startX)*pntY+endX*startY-endY*startX)/sqrt((endY-startY)**2+(endX-startX)**2)