'''
             JOYPAD CONTROLLER FOR SDP ROBOTS
       Author: Ayrton Massey, s1208057@sms.ed.ac.uk

This is SDP 2015 Group 9's joypad controller for SDP robots.
Requires the pygame module and an XBOX 360 controller.

'''

import pygame

def query_yes_no(question,default="no"):
    valid = {"yes" : True, "y" : True, "no" : False, "n" : False}
    
    if default is None:
        prompt = " [y/n] "
    elif default is "yes":
        prompt = " [Y/n] "
    elif default is "no":
        prompt = " [y/N] "
    else:
        raise ValueError("Invalid default answer '%s'" % default)

    while True:
        print question + prompt
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else: 
            print "Please respond with 'yes', 'no', 'y' or 'n'."

def __init__():
    pygame.init()
    pygame.joystick.init()

def __quit__():
    pygame.joystick.quit()
    pygame.quit()

def select_joystick():
    '''Iterate over all present joysticks.'''
    print "Searching for joysticks..."
    joystick_count = pygame.joystick.get_count()
    joy_id = -1;
    for x in range(joystick_count):
        joystick = pygame.joystick.Joystick(x)
        joystick.init()
        q_str = "Use joystick '%s'?" % joystick.get_name()
        if query_yes_no(q_str,"yes"):
            return x

    return -1

def main():
    __init__()

    clock = pygame.time.Clock()

    joy_id = select_joystick()
    if(joy_id == -1):
        print "Failed to select joystick. Exiting..."
        __quit__()

    joystick = pygame.joystick.Joystick(joy_id)
    joystick.init()
    print "Continuing with %s." % joystick.get_name()

    ''' Check for joystick events. '''
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print "Button pressed:  '%d'" % event.button
            if event.type == pygame.JOYBUTTONUP:
                print "Button released: '%d'" % event.button
            if event.type == pygame.JOYAXISMOTION:
                print "Axis motion ({id}): {pos}".format(id = event.axis, pos = joystick.get_axis(event.axis))

        ''' Limit to 20 ticks/s '''
        clock.tick(20)

    __quit__()


''' Run '''
main()
