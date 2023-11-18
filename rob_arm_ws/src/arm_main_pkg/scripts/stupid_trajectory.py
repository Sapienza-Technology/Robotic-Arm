#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32
import time
import math

from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def main():
    ### Initialization ROS Node ###
    rospy.init_node("stupid_trajectory")
    offsets = np.array([0, degToRad(90), degToRad(-90), 0, 0, 0])

    steps = 50

    ### Initialization ROS Publishers ###
    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=1)
    pub_command_vel = rospy.Publisher("/firmware_arm_vel", Float32MultiArray, queue_size=1)
    pub_ee = rospy.Publisher("/firmware_EEP_pos", Float32, queue_size=100)

    trovato=False
    traiettorie = ["paletta", "pirulo", "inPose", "prova", "cubo", "cerchio", "ballo", "vuoto", "vuoto2", "miatraiettoria"]
    while(True):
        print("Scegliere una traiettoria: paletta, pirulo, inPose, prova, cubo, cerchio, ballo, vuoto, vuoto2, miatraiettoria")
        stringa=input()
        print(stringa)
        for i in range(len(traiettorie)):
            if stringa==traiettorie[i]:
                premade_traj(traiettorie[i], steps, offsets)
                trovato=True
                break
        if trovato==False:
            print("Traiettoria non valida")
        if trovato==True:
            break
    

    # change offsets to change starting position
    


    print("Generando traiettoria...")

    #time.sleep(5)
    #T, gripper_state = premade_traj("miatraiettoria", steps, offsets) #array of trajectories between points
    #print(T)

    print("Traiettoria generata: eseguendo...")

    curr_pos = np.zeros(6)
    max_time = 5

    joint_val_pres = -offsets


    # For every point in trajectory T
    for j in range(len(T)):
        arm_goal_arr = T[j].q
        print("Andando verso il prossimo punto...")
        time_resolution = max_time / steps
        time_to_wait = 0

        ### Gripper state change ###
        command = Float32()
        command.data = gripper_state[j]
        pub_ee.publish(command)
        time.sleep(3)

        for i in range(len(arm_goal_arr)):
            print("Step: ")
            print(arm_goal_arr[i])
            command = Float32MultiArray()
            command_vel = Float32MultiArray()
            command.data = arm_goal_arr[i] - offsets
            delta_joint = command.data - joint_val_pres
            joint_val_pres = command.data
            command_vel.data = delta_joint/max_time
            pub_command_vel.publish(command_vel)
            pub_command.publish(command)

            print(f"{i}. Inviando: {command.data}")
            print(f"{i}. Inviando velocità: {command_vel.data}")

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
            #time.sleep(0.03)
        time.sleep(0.2)
        print("Aspettando di arrivare al punto")

        ## Eventually wait for user input to continue
        #user_input = input("prosegui: ")
        #while user_input == "n":
        #    print("A quale punto vuoi che vada l'end-effector?")
        #    x = float(input("x: "))
        #    y = float(input("y: "))
        #    z = float(input("z: "))

        #    task = True
        #    c_all = [np.array([x, y, z])]
        #    R_all = [Ry(-np.pi)]
        #    elbow_all = [True]
        #    intermediate_T = era_traj(task, arm_goal_arr[-1] - offsets, c_all, R_all, elbow_all, 1)
        #    arm_goal_arr = intermediate_T[0].q

        #    print("Step: ")
        #    print(arm_goal_arr[-1])
        #    command = Float32MultiArray()
        #    command_vel = Float32MultiArray()
        #    command.data = arm_goal_arr[-1] - offsets
        #    delta_joint = command.data - joint_val_pres
        #    joint_val_pres = command.data
        #    command_vel.data = delta_joint/max_time
        #    pub_command_vel.publish(command_vel)
        #    pub_command.publish(command)

        #    time.sleep(5)



    print("Traiettoria eseguita")

    print("Chiudendo prima di tornare allo zero...")
    command = Float32()
    command.data = 0.0
    pub_ee.publish(command)
    time.sleep(5)

    print("Tornando allo zero...")
    command = Float32MultiArray()
    command.data = np.zeros(6)
    pub_command.publish(command)


main()
    