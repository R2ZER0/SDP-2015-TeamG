'''
             JOYPAD CONTROLLER FOR SDP ROBOTS
       Author: Ayrton Massey, s1208057@sms.ed.ac.uk

This is SDP 2015 Group 9's joypad controller for SDP robots.
Requires the pygame module and an XBOX 360 controller.

'''

import pygame
from action import Action
import math
import serial
import os
import sys

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

robot = None
comm = None

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

    comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    robot = Action(comm)

    clock = pygame.time.Clock()

    joy_id = select_joystick()
    if(joy_id == -1):
        print "Failed to select joystick. Exiting..."
        __quit__()

    joystick = pygame.joystick.Joystick(joy_id)
    joystick.init()
    print "Continuing with %s." % joystick.get_name()
    print "RB: Kick\nLB: Catch\nLeft Analog Stick: Move"

    catching = False
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                if joystick.get_axis(event.axis):
                    pass
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 5:
                    robot.kick()
            if event.type == pygame.JOYBUTTONUP:
                if event.button == 4:
                    robot.catch()

        left_y = -1 * joystick.get_axis(1)
        left_x = joystick.get_axis(0)

        speed = math.sqrt((left_x * left_x) + (left_y * left_y))
        
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1

        theta = math.atan2(left_x,left_y)

        if speed > 0.5 :
            robot.move(theta,speed)
        else:
            robot.stop()

        ''' Limit to 20 ticks/s '''
        clock.tick(20)

    __quit__()


''' Run '''
main()
