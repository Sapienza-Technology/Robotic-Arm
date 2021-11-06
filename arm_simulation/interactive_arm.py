import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import tpoly,lspb
from math import pi as pi
from spatialmath.base import transl, trotx, troty, trotz
from msvcrt import getch, getwch
import numpy as np
from arm_functions import move

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

time = np.array(range(1,2))

P1 = transl(0.5, 0.1, 0.3)

# Calcoliamo una traiettoria nulla per mostrare il plot prima di ricevere input
trajectory = robot.jtraj(T1=SE3(P1), T2=SE3(P1), t=time)
robot.plot(trajectory.q, limits=[-1, 1, -1, 1, -1, 1])

character=''
while (character!='c'):
    character = getwch()
    print(character)

    if (character == 'w'): P1 = move(P1, robot, time, 'up')
    elif (character == 's'): P1 = move(P1, robot, time, 'down')
    elif (character == 'd'): P1 = move(P1, robot, time, 'right')
    elif (character == 'a'): P1 = move(P1, robot, time, 'left')
    elif (character == 'H'): P1 = move(P1, robot, time, 'forward')
    elif (character == 'P'): P1 = move(P1, robot, time, 'backwards')
