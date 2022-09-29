#!/usr/bin/env python3

import roboticstoolbox as rbt
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from math import pi as pi
import numpy as np

xacro = ERobot.URDF("/home/alessio/ROS/Robotic-Arm/rob_arm_ws/src/braccio3_urdf_description/urdf/braccio3_urdf.xacro")
print(xacro)
# we don't need urdf, importing the xacro works ok
#urdf = ERobot.URDF("/home/alessio/ROS/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/urdf/braccio_urdf.urdf")
#print(urdf)

# the problem was just that we were using the wrong ikine function
# ALWAYS USE IKINE_MIN
sol_inv = xacro.ikine_min(SE3([1, 1, 1]))
print(sol_inv)
xacro.plot(sol_inv.q, backend='pyplot', block=True)