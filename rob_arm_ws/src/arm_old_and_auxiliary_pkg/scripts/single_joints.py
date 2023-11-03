#!/usr/bin/env python3

"""
    Program to move each joint independently
    Choose which joint you want to move and which position you want to get them to
"""

import rospy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
import time
import math

N_JOINTS = 6

def degToRad(angle):
    return angle/180*math.pi


def main():
    rospy.init_node("single_joints")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_EEM_pos", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, -degToRad(115), 0])
    
    values = np.zeros(N_JOINTS)
    prevValues = np.zeros(N_JOINTS)

    while True:
        print("Which joints do you want to move? (Index starting from 1, separated by space, increasing order, \"q\" to quit)")
        s = input("> ")
        if (s == "q"): return
        print("Write their respective angle(in degrees, separated by space, \"q\" to quit)")
        d = input("> ")
        if (d == "q"): return

        joints = list(map(lambda x: int(x), s.split()))
        print(f"joints: {joints}")
        degrees = list(map(lambda x: float(x), d.split()))
        print(f"degrees: {degrees}")

        for i in range(len(joints)):
            values[joints[i]-1] = degToRad(degrees[i])
            #flags[joints[i]-1] = 1

        command = Float32MultiArray()

        data = values
        #data = np.concatenate((data, flags))
        command.data = data
        pub_command.publish(command)
        print(command)

        
if __name__ == '__main__':
    main()       