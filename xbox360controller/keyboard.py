'''
             JOYPAD CONTROLLER FOR SDP ROBOTS
       Author: Ayrton Massey, s1208057@sms.ed.ac.uk

This is SDP 2015 Group 9's keyboard controller for SDP robots.
Requires the pygame module.

'''

import pygame
from action import Action
import math
import serial

robot = None
comm = None

def __init__():
    pygame.init()

def __quit__():
    pygame.quit()

def main():
    __init__()

    comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    robot = Action(comm)

    pygame.display.set_caption('Keyboard Example')
    size = [640, 480]
    screen = pygame.display.set_mode(size)

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                robot.stop()
                __quit__()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_d:
                    robot.move(math.pi/2,1)
                elif event.key == pygame.K_a:
                    robot.move(-math.pi/2,1)
                elif event.key == pygame.K_w:
                    robot.move(0,-1)
                elif event.key == pygame.K_s:
                    robot.move(math.pi,1)
                elif event.key == pygame.K_e:
                    robot.kick()
                elif event.key == pygame.K_q:
                    robot.catch()
                if event.key == pygame.K_SPACE:
                    robot.stop()

        ''' Limit to 20 ticks/s '''
        clock.tick(20)

    __quit__()


''' Run '''
main()
