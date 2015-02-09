
from math import hypot, atan2, cos, sin

def get_boundary_path(robot, (x, y, displacement, theta), zone):
    '''Finds the first point inside the given zone that the movement pattern
    specified would result in.

    Attributes:
        robot   The Robot model to extend movement from
        vector  A tuple containing:
            x               Starting x-position of movement
            y               Starting y-position of movement
            displacement    Distance to travel each iteration
            theta           The direction to move in
        zone    The zone to stop movement in

    Returns:
        A tuple containing x,y positions of new point, distance between original
        and destination point, and the angle of movement.
    '''
    while not zone.isInside(x, y):
        # Simulate a movement 'step' of displacement magnitude
        plan_x = robot.x + ((displacement-1) * cos(theta))
        plan_y = robot.y + ((displacement-1) * sin(theta))

        # Store our updated new position, decreasing displacement to simulate
        # friction
        x, y, displacement = plan_x, plan_y, displacement-1

    return x, y, displacement, theta

def get_path_to_point(robot, x, y):
    '''Retrieves a tuple indicating distance and angle between the robot and
    the given x,y point.

    Attributes:
        robot   The Robot model as the origin point
        x       The destination x-position
        y       The destination y-position

    Returns:
        A 'path' point including target x,y, distance to point, and direction to
        travel. 
    '''
    delta_x = x - robot.x
    delta_y = y - robot.y

    displacement = hypot(delta_x, delta_y)

    theta = atan2(delta_y, delta_x) - robot.angle

    return x, y, displacement, theta


def get_interception(path, robot):
    '''Retrieves the travel direction and distance for the robot to intercept 
    the given path of the ball.

    Attributes:
        path    A path object for the ball, returned from models.py, in
                Robot:get_pass_path function
        robot   The Robot model

    Returns:
        A path tuple containing distance and direction to travel to intercept
        the ball path.
    '''

    center_path = path.center()
    return get_path_to_point(robot, center_path[0], center_path[1])


def get_avoidance(path, robot, obstacle):
    '''Detects if the proposed path intersects with the obstacle and calculates
    a nearby non-colliding position if it does.

    Attributes:
        path        A path object for the proposed movement, returned from 
                    models.py, in Robot:get_pass_path function
        robot       The robot model moving to the ball
        obstacle    Another PitchObject to avoid

    Returns:
        None        If the given obstacle doesn't interfere with the path

        or

        Path tuple containing new position, distance, and direction to travel
        to to avoid the obstacle.
    '''
    if path.overlaps(obstacle):

        # calculate the rotated path 'width' 
        path_width_x = path[2][0] - path[3][0]
        path_width_y = path[2][1] - path[3][1]
        path_width = hypot(path_width_x, path_width_y)

        robot_width = obstacle.get_dimensions()[0]

        # minimum distance we have to deviate from the centre to be avoiding
        # the obstacle
        min_distance = (path_width * 0.5) + (robot_width * 0.5)

        # calculate the deviation of obstacle from the centre of our path
        center_path = path.center()
        delta_x = center_path[0] - obstacle.x
        delta_y = center_path[1] - obstacle.y
        displacement = hypot(delta_x, delta_y)

        # calculate distance from path center 
        path_distance = min_distance - displacement

        theta = atan2(delta_y, delta_x)
    
        # calculate our new path centerpoint         
        path_x = robot.x + (path_distance * cos(theta))
        path_y = robot.y + (path_distance * sin(theta))
        
        # finally, return a path that moves the robot to this point
        return get_path_to_point(robot, path_x, path_y)
    else:
        return None