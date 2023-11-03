#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import math

from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def main():
    rospy.init_node("stupid_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_EEP_pos", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, -degToRad(105), 0])

    steps = 21

    time_durations = [8, 6, 6, 6, 6, 8]

    print("Generando traiettoria...")
    
    time.sleep(5)
    
    """
    command = Float32MultiArray()
    command.data = np.array([0, 0, 0, 0, degToRad(95), 0])
    pub_command.publish(command)
    time.sleep(15)
    command = Float32MultiArray()
    command.data = np.array([0, 0, 0, 0, degToRad(190), 0])
    pub_command.publish(command)
    time.sleep(15)
    command = Float32MultiArray()
    command.data = np.array([0, 0, 0, 0, degToRad(100), 0])
    pub_command.publish(command)
    time.sleep(15)
    command = Float32MultiArray()
    command.data = np.array([0, 0, 0, 0, degToRad(10), 0])
    pub_command.publish(command)
    time.sleep(15)
    """

    #return 

    T = premade_traj_time("pirulo", time_durations, offsets) #array of trajectories between points

    print(T)

    print("Traiettoria generata: eseguendo...")

    curr_pos = np.zeros(6)
    max_time = [0.5, 1, 1, 1, 1, 0.5]


    # For every point in trajectory T
    for j in range(len(T)):
        arm_goal_arr = T[j].q
        arm_vel_arr = T[j].qd
        print("Andando verso il prossimo punto...")

        if j == 2:
            command = Float32()
            command.data = 1.0
            pub_ee.publish(command)
            time.sleep(3)
        if j == 3:
            command = Float32()
            command.data = 0.0
            pub_ee.publish(command)
            time.sleep(3)

        for i in range(len(arm_goal_arr)):
            print("Step: ")
            print(arm_goal_arr[i])
            command = Float32MultiArray()
            if i == 0:
                command.data = np.concatenate((arm_goal_arr[i] - offsets, arm_vel_arr[1]))
            elif i == len(arm_goal_arr)-1:
                command.data = np.concatenate((arm_goal_arr[i] - offsets, arm_vel_arr[len(arm_goal_arr)-2]))
            else:
                command.data = np.concatenate((arm_goal_arr[i] - offsets, arm_vel_arr[i]))
            pub_command.publish(command)
            print(f"{i}. Inviando: {command.data}")
            """
            print("arm_goal_arr[0]: ", arm_goal_arr[0])
            print("curr_pos: ",curr_pos)
            print("arm_goal_arr[0] - curr_pos: ", arm_goal_arr[0]-curr_pos)
            time_to_wait=weighted_norm(arm_goal_arr[0]-curr_pos, np.array([2, 1, 4, 1, 1, 1]))
            print(f"Aspettando: {time_to_wait}")
            time.sleep(time_to_wait)
            curr_pos = arm_goal_arr[i]
            """

            """
            if ((i+1) // (steps/3) == 0):
                time_to_wait += time_resolution
            elif ((i+1) // (steps/3) == 1):
                pass
            else:
                if (time_to_wait - time_resolution < 0):
                    time_to_wait = 0.1
                else:
                    time_to_wait -= time_resolution
            print("time_to_wait: ", time_to_wait)
            time.sleep(time_to_wait)
            """
            time.sleep(1.5)
        print("Aspettando di arrivare al punto")
        user_input = input("prosegui: ")
        while user_input == "n":
            print("A quale punto vuoi che vada l'end-effector?")
            x = float(input("x: "))
            y = float(input("y: "))
            z = float(input("z: "))

            task = True
            c_all = [np.array([x, y, z])]
            R_all = [Ry(-np.pi)]
            elbow_all = [True]
            intermediate_T = era_traj(task, arm_goal_arr[-1] - offsets, c_all, R_all, elbow_all, 1)
            arm_goal_arr = intermediate_T[0].q

            print("Step: ")
            print(arm_goal_arr[-1])
            command = Float32MultiArray()
            command.data = arm_goal_arr[-1] - offsets
            pub_command.publish(command)
            time.sleep(5)


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



        