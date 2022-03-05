import roboticstoolbox as rbt
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
from spatialmath import SE3
from math import pi as pi
import numpy as np

urdf = ERobot.URDF("/home/alessio/ROS/rob_arm_ws_old/src/braccio_urdf_description/urdf/braccio_urdf_edit.xacro")
print(urdf)

