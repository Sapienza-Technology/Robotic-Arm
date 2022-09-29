#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import time
import math

from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def main():
    rospy.init_node("stupid_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, math.pi/2, 0])

    T = premade_traj("pirulo", 5, offsets) #array of trajectories between points

    print(T)

    print("Traiettoria generata: eseguendo...")


    for point in T:
        arm_goal_arr = point.q
        print("Andando verso il prossimo punto...")
        for i in range(len(arm_goal_arr)):
            print("Step: ")
            print(arm_goal_arr[i])
            command = Float32MultiArray()
            command.data = arm_goal_arr[i] - offsets
            pub_command.publish(command)
            time.sleep(5)
        print("Aspettando di arrivare al punto")
        input("prosegui: ")
        #time.sleep(30)


    print("Traiettoria eseguita")
    """
    print("Fra 10 secondi torniamo a zero")
    for i in range(10):
        print(i+1)
        time.sleep(1)
    """


    print("Tornando allo zero...")
    command = Float32MultiArray()
    command.data = np.zeros(6)
    pub_command.publish(command)


    

main()



        