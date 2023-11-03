#!/usr/bin/env python3

from matplotlib.pyplot import step
from sympy import deg
import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import math
from getch import getch
from arm_functions import *

from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def main():
    rospy.init_node("stupid_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_EEM_pos", Float32, queue_size=100)

    """
    for i in range(100):
        command = Float32MultiArray()
        data = [0, 0, 0, 0, degToRad(0), 0]
        command.data = data
        pub_command.publish(command)
        print("command sent")

    return
    """

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, -degToRad(115), 0])

    q_pres = offsets

    steps = 2

    character=''
    

    while(character != 'c'):
        character = getch()
        print(character)
        if character == 'w': 
            #p1 = move_gazebo(p1, 'up')
            arm_goal_arr = go_straight(True, True, q_pres, "u", steps, 0.03, False)
        elif character == 's': 
            arm_goal_arr = go_straight(True, True, q_pres, "d", steps, 0.03, False)
        elif character == 'd': 
            arm_goal_arr = go_straight(True, True, q_pres, "r", steps, 0.03, False)
        elif character == 'a': 
            arm_goal_arr = go_straight(True, True, q_pres, "l", steps, 0.03, False)
        elif character == 'q': 
            arm_goal_arr = go_straight(True, True, q_pres, "f", steps, 0.03, False)
        elif character == 'e': 
            arm_goal_arr = go_straight(True, True, q_pres, "b", steps, 0.03, False)
        # for rotations
        elif character == '6': 
            arm_goal_arr = rotate(True, True, q_pres, "r_cw", steps, 0.3, False)
        elif character == '4': 
            arm_goal_arr = rotate(True, True, q_pres, "r_ccw", steps, 0.3, False)
        elif character == '2': 
            arm_goal_arr = rotate(True, True, q_pres, "p_cw", steps, 0.3, False)
        elif character == '8': 
            arm_goal_arr = rotate(True, True, q_pres, "p_ccw", steps, 0.3, False)
        elif character == '3':
            arm_goal_arr = rotate(True, True, q_pres, "y_cw", steps, 0.3, False)
        elif character == '1': 
            arm_goal_arr = rotate(True, True, q_pres, "y_ccw", steps, 0.3, False)
        elif character == "o":
            command = Float32()
            command.data = 0
            pub_ee.publish(command)
            time.sleep(1)
            continue
        elif character == "p":
            command = Float32()
            command.data = 1
            pub_ee.publish(command)
            time.sleep(1)
            continue
        elif character == "k":
            arm_goal_arr = q_pres.copy()
            arm_goal_arr[5] += degToRad(3)
        elif character == "l":
            arm_goal_arr = q_pres.copy()
            arm_goal_arr[5] -= degToRad(3)
        elif character == 'z':
            arm_goal_arr = np.zeros(6) + offsets

        q_pres = arm_goal_arr[steps-1]

        for i in range(len(arm_goal_arr)):
            command = Float32MultiArray()
            data = arm_goal_arr[i] - offsets
            data[4] += math.pi/2
            command.data = data
            pub_command.publish(command)
            time.sleep(0.5)

        

main()       