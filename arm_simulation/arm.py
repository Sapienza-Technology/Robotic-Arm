import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import tpoly,lspb
from math import pi as pi
from spatialmath.base import transl, trotx
import numpy as np

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

time = np.array(range(1,100, 5))

P1 = SE3(transl(0.5, 0.1, 0.3) * trotx(pi))
P2 = SE3(transl(0.1, -0.5, -0.3))

trajectory = robot.jtraj(T1=P1, T2=P2, t=time)
print(robot.jtraj(T1=P1, T2=P2, t=time))

# trajectory.qd returns velocity
# trajectory.qdd returns acceleration
DHRobot.plot(robot, trajectory.q, movie='animation_tech_robot.mp4')
