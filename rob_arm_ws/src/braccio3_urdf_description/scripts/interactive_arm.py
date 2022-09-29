#!/usr/bin/env python3

import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import tpoly,lspb
from math import pi as pi
from spatialmath.base import transl, trotx, troty, trotz
import numpy as np
from arm_functions import move
from getch import getch
from pynput import keyboard

# DH parameters
a = [0, 0, 0.5, 0, 0.4, 0]
d = [0, 0, 0, 0, 0, 0]
alpha = [0, pi/2, 0, pi/2, pi/2, pi/2]

robot = DHRobot([RevoluteMDH(d[0], a[0], alpha[0]),
    RevoluteMDH(d[1], a[1], alpha[1]),
    RevoluteMDH(d[2], a[2], alpha[2]),
    RevoluteMDH(d[3], a[3], alpha[3]),
    RevoluteMDH(d[4], a[4], alpha[4]),
    RevoluteMDH(d[5], a[5], alpha[5])],
    name='6R')

time = np.array(range(10))

# transl creates an SE(3) matrix given a point in space (x, y ,z)
P1 = transl(0.5, 0.1, 0.3)
print(P1)
rot = trotx(0) @ troty(1.6) @ trotz(0)
print(rot)
P1 = P1 @ rot
print(SE3(P1))

# Calcoliamo una traiettoria nulla per mostrare il plot prima di ricevere input
trajectory = robot.jtraj(T1=SE3(P1), T2=SE3(P1), t=time)
robot.plot(trajectory.q, limits=[-1, 1, -1, 1, -1, 1])

character=''
while (character!='c'):
    character = getch()
    print(character)

    if (character == keyboard.KeyCode.from_char('w')): P1 = move(P1, robot, time, 'up')
    elif (character == keyboard.KeyCode.from_char('s')): P1 = move(P1, robot, time, 'down')
    elif (character == keyboard.KeyCode.from_char('d')): P1 = move(P1, robot, time, 'right')
    elif (character == keyboard.KeyCode.from_char('a')): P1 = move(P1, robot, time, 'left')
    elif (character == keyboard.KeyCode.from_char('H')): P1 = move(P1, robot, time, 'forward')
    elif (character == keyboard.KeyCode.from_char('P')): P1 = move(P1, robot, time, 'backwards')

    elif (character == keyboard.KeyCode.from_char('8')): P1 = move(P1, robot, time, 'pitch-up')
    elif (character == keyboard.KeyCode.from_char('2')): P1 = move(P1, robot, time, 'pitch-down')
    elif (character == keyboard.KeyCode.from_char('6')): P1 = move(P1, robot, time, 'yaw-right')
    elif (character == keyboard.KeyCode.from_char('4')): P1 = move(P1, robot, time, 'yaw-left')
    elif (character == keyboard.KeyCode.from_char('7')): P1 = move(P1, robot, time, 'roll-ccw')
    elif (character == keyboard.KeyCode.from_char('9')): P1 = move(P1, robot, time, 'roll-cw')
