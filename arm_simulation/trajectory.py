import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import tpoly,lspb
from math import pi as pi
from spatialmath.base import transl, trotx, troty
from matplotlib import pyplot as plt
from matplotlib import animation
import PIL.Image
import time
ta.setup_logging("INFO")

# DH parameters
a = [0, 0, 0.6, 0, 0.4, 0]
d = [0, 0, 0, 0, 0, 0]
alpha = [0, pi/2, 0, pi/2, pi/2, pi/2]

robot = DHRobot([RevoluteMDH(d[0], a[0], alpha[0], qlim=[-pi, pi]),
                RevoluteMDH(d[1], a[1], alpha[1], qlim=[-pi, pi/2]),
                RevoluteMDH(d[2], a[2], alpha[2], qlim=[-pi/2, pi/2]),
                RevoluteMDH(d[3], a[3], alpha[3], qlim=[-pi/2, pi/2]),
                RevoluteMDH(d[4], a[4], alpha[4], qlim=[-pi/2, pi/2]),
                RevoluteMDH(d[5], a[5], alpha[5], qlim=[-pi, pi])],
                name='6R')


q0 = np.zeros((6, ))
P1 = SE3(transl(0.3, 0, 0.4))
P2 = SE3(transl(0.5, 0, -0.3))*SE3(troty(pi/2))
P3 = SE3(transl(0.5, 0, -0.5))*SE3(troty(pi/2))

q1 = robot.ikine_min(SE3(P1), qlim=True)
q2 = robot.ikine_min(SE3(P2), qlim=True)
q3 = robot.ikine_min(SE3(P3), qlim=True)

p1_f = robot.fkine(q1[0])


dof = 6

way_pts = [q0, q1[0], q2[0], q3[0]]

vlims = np.ones((6, )) * 20
alims = np.ones((6, )) * 10
ss = np.linspace(0, 1, 4)

path = ta.SplineInterpolator(ss, way_pts)
pc_vel = constraint.JointVelocityConstraint(vlims)
pc_acc = constraint.JointAccelerationConstraint(alims)
instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
jnt_traj = instance.compute_trajectory()
ts_sample = np.linspace(0, jnt_traj.duration, 100)
qs_sample = jnt_traj(ts_sample)
qds_sample = jnt_traj(ts_sample, 1)
qdds_sample = jnt_traj(ts_sample, 2)
'''
fig, axs = plt.subplots(3, 1, sharex=True)
for i in range(path.dof):
   # plot the i-th joint trajectory
    axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
    axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
    axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
axs[2].set_xlabel("Time (s)")
axs[0].set_ylabel("Position (rad)")
axs[1].set_ylabel("Velocity (rad/s)")
axs[2].set_ylabel("Acceleration (rad/s2)")
plt.show()
'''
robot.plot(jnt_traj.eval(ts_sample),block=True)

