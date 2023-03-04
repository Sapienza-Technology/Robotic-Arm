#!/usr/bin/env python3

from matplotlib.pyplot import step
from sympy import deg
import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import math
from getch import getch
from arm_functions import *
from sasa_functions import inv_kin_bis

from traj_genv6 import *

def main():
    rospy.init_node("stupid_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_EEM_pos", Float32, queue_size=100)

    while True:

        print("Write three numbers separated by spaces representing the x y z coordinates of the target position")
        points = input()
        if points == 'c':
            break

        target_q = send3DGoal(points, pub_command)

    return target_q

def degToRad(angle):
    return angle/180*math.pi

def send3DGoal(points, pub_command):

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, -degToRad(115), 0])

    q_pres = offsets

    steps = 2

    target_arr = np.array(list(map(lambda x: float(x), points.split())))

    target_q = inv_kin_bis(target_arr, True) 
    print(target_q)

    target_q = np.array([target_q[0], target_q[1], target_q[2], 0, 0, 0])

    command = Float32MultiArray()
    data = target_q - offsets
    data[4] += math.pi/2
    command.data = data
    pub_command.publish(command)
    time.sleep(0.5)

    return target_q
        

if __name__ == "__main__":
    main()       