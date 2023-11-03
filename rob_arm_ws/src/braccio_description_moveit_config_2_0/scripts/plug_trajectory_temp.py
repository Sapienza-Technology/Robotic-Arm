#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32, Bool
import time
import math
import tf
from getch import getch

from geometry_msgs.msg import PoseStamped, Pose
from sasa_functions import *
from traj_genv6 import *

def degToRad(angle):
    return angle/180*math.pi

def matrix4x4_from_pose(pose):
    t=[pose.position.x, pose.position.y, pose.position.z]
    q=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    print(tf.transformations.euler_from_quaternion(q))
    return np.array(tf.transformations.compose_matrix(translate=t, angles=tf.transformations.euler_from_quaternion(q)))


def main():

    rospy.init_node("button_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_command_vel = rospy.Publisher("/firmware_arm_vel", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_arm_pinza", Float32, queue_size=100)

    request_topic = rospy.Publisher("/button_request", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, 0, 0])
    q_pres = np.array([0, degToRad(90), degToRad(-130), 0, 0, 0])


    tot_time = 2
    steps = 100
    open_pinza = 0.05
    max_time = 2

    joint_val_pres = q_pres-offsets         ### joint values in the real motor world

    last_command = q_pres-offsets

    

    task = False
    c_all= [np.array([0.115, 0.225, 0.06]),
            np.array([0.115, 0.225, 0.03]),
            np.array([0.115, 0.225, 0.06]),
            np.array([0.25,  0.00, 0.25]),]
    print("c_all: ", c_all)
    R_all = [Ry(np.pi),Ry(np.pi),Ry(np.pi),Ry(np.pi/2)]
    elbow_all = [True,True,True,True]


    input("Premi un tasto per continuare...")
    T = era_traj(task, q_pres, c_all, R_all, elbow_all, steps, open_pinza) #array of trajectories between points

    print(T)

    print("Traiettoria generata: eseguendo...")
    

    # For every point in trajectory T
    for j in range(len(T)):
        arm_goal_arr = T[j].q
        print("Andando verso il prossimo punto...")
        time_resolution = max_time / (steps/3)
        if j == 1:
            print("Apertura pinza")
            for k in range(6):
                command = Float32()
                command.data = (open_pinza + 0.02)*1000  ###### aggiusta
                pub_ee.publish(command)
                time.sleep(5)
        if j == 2:
            print("Chiusura pinza")
            for k in range(6):
                command = Float32()
                command.data = (open_pinza - 0.02)*1000  ###### aggiusta
                pub_ee.publish(command)
                time.sleep(5)
        if j == 4:
            max_time = 10

        for i in range(len(arm_goal_arr)):
            print("Step: ")
            print(arm_goal_arr[i])
            command = Float32MultiArray()
            command_vel = Float32MultiArray()
            res, bool_vec = range_check(arm_goal_arr[i,:3])

            if not res:
                print("Range check failed: joint " + str(np.where(bool_vec == 0)[0]+1) + " out of range")
                command.data = last_command
            else:
                command.data = arm_goal_arr[i] - offsets
                last_command = command.data

            delta_joint = command.data - joint_val_pres
            joint_val_pres = command.data
            command_vel.data = delta_joint/max_time
            pub_command_vel.publish(command_vel)
            pub_command.publish(command)

            print(f"{i}. Inviando: {command.data}")
            print(f"{i}. Inviando velocità: {command_vel.data}")

            time.sleep(tot_time/steps)
        print("Aspettando di arrivare al punto")
        user_input = input("prosegui: ")
        while user_input == "n":                                          #### stai attento alla n
            print("A quale punto vuoi che vada l'end-effector?")
            x = float(input("x: "))
            y = float(input("y: "))
            z = float(input("z: "))

            task = True
            c_all = [np.array([x, y, z])]
            R_all = [Ry(-np.pi)]
            elbow_all = [True]
            intermediate_T = era_traj(task, arm_goal_arr[-1] - offsets, c_all, R_all, elbow_all, 1, open_pinza)
            arm_goal_arr = intermediate_T[0].q

            print("Step: ")
            print(arm_goal_arr[-1])
            command = Float32MultiArray()
            command_vel = Float32MultiArray()
            command.data = arm_goal_arr[-1] - offsets
            delta_joint = command.data - joint_val_pres
            joint_val_pres = command.data
            command_vel.data = delta_joint/max_time
            pub_command_vel.publish(command_vel)
            pub_command.publish(command)

            time.sleep(5)



    print("Tornando allo zero...")
    command = Float32MultiArray()
    command.data = np.zeros(6)
    pub_command.publish(command)


main()
