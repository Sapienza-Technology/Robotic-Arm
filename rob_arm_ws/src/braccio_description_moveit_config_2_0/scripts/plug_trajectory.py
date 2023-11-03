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

def command_joint_values(q_des, offsets, pub_command, pub_command_vel):
    for i in range(5):
        command = Float32MultiArray()
        command_vel = Float32MultiArray()
        command.data = q_des-offsets
        command_vel.data = command.data*20
        pub_command_vel.publish(command_vel)
        time.sleep(1)
        pub_command.publish(command)
        #print(f"command sent: {command.data}")

def main():

    rospy.init_node("button_trajectory")

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=100)
    pub_command_vel = rospy.Publisher("/firmware_arm_vel", Float32MultiArray, queue_size=100)
    pub_ee = rospy.Publisher("/firmware_arm_pinza", Float32, queue_size=100)
    pub_detect_req = rospy.Publisher("/tag_start_detect", Bool, queue_size=1)


    request_topic = rospy.Publisher("/button_request", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, 0, 0])
    q_pres = np.array([0, degToRad(90), degToRad(-130), 0, 0, 0])

    ### Inizializzo la posizione del braccio pre vedere i tag
    for i in range(5):
        command = Float32MultiArray()
        command_vel = Float32MultiArray()
        command.data = q_pres-offsets
        command_vel.data = command.data/2
        pub_command_vel.publish(command_vel)
        pub_command.publish(command)
        #print(f"command sent: {command.data}")
        time.sleep(1)
        
    #time.sleep(5)

    tot_time = 2
    steps = 100
    open_pinza = 0.07


    print("Schiaccia un pulsante quando vuoi detectare...")

    getch()

    request_msg = Float32()
    request_msg.data = 1.0
    not_detected = True
    not_finished = True

    fixes = 1

    while not_detected:
        pub_detect_req.publish(Bool())
        go_up = rospy.wait_for_message('/tag_go_up', Bool)
        #getch()

        request_topic.publish(request_msg)
        print("Ti aspecto...")
        try:
            tag_pose = rospy.wait_for_message("/button_to_press", PoseStamped, timeout=rospy.Duration(10))
            tag_pose = tag_pose.pose
        except rospy.exceptions.ROSException as e:
            print("Timeout tag_pose")
            tag_pose = Pose()
            tag_pose.position.x = -1
            tag_pose.position.y = -1
            tag_pose.position.z = -1
            print(e)
        
        if tag_pose.position.x == -1 and tag_pose.position.y == -1 and tag_pose.position.z == -1:
            print("No tag recived, try to move")
            q_pres = np.array([0, degToRad(90), degToRad(-130+5*fixes), 0, 0, 0])
            q_camera = np.array([0, degToRad(90), degToRad(-130+5*fixes), 0, 0, 0])
            command_joint_values(q_pres, offsets, pub_command, pub_command_vel)
            if fixes == 8:
                fixes -= 1
            else:
                fixes += 1
            sleep(5)
        
        elif tag_pose.position.x == 0 and tag_pose.position.y == 0 and tag_pose.position.z == 0:
            print("Error reciving the tag pose")
            not_finished = False
            break
        else:
            not_detected = False

    T_pose = matrix4x4_from_pose(tag_pose)

    print("tag pose in camera frame: ", tag_pose)
    ## -44mm in x, 0 in y, 65mm in z (base_link)
    tf_camera = tf_camera_from_base(q_pres) #### da definire
    tag_pose_in_base = tf_camera@T_pose
    tag_angles = tf.transformations.euler_from_matrix(tag_pose_in_base[:3,:3])
    tag_position = tf_camera@np.array([[tag_pose.position.x], [tag_pose.position.y], [tag_pose.position.z], [1]])
    print("tag position in base link frame: ", tag_position.transpose()[0])
    print("tag angles in base link frame: ", tag_angles)
    if tag_angles[2]< 0 and tag_angles[2]>pi/2:
        tag_angles_temp = abs(tag_angles[2])
    else:
        tag_angles_temp = tag_angles[2]
    angle_shift = tag_angles_temp+pi/2
    if angle_shift > 0:
        angle_shift = angle_shift - pi
    else:
        angle_shift = angle_shift + pi
    #tag_position = tag_position.transpose()

    task = False
    c_all = [np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]]),
            np.array([tag_position[0,0], tag_position[1,0], tag_position[2,0]]),
            np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]])]
    print("c_all: ", c_all)
    R_all = [Rz(angle_shift)@Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2)]
    elbow_all = [True,True,True]

    print("Si continua...")
    time.sleep(5)
    T = era_traj(task, q_pres, c_all, R_all, elbow_all, steps) #array of trajectories between points

    print(T)

    print("Traiettoria generata: eseguendo...")
    
    max_time = 2

    joint_val_pres = q_pres-offsets         ### joint values in the real motor world

    last_command = q_pres-offsets

    # For every point in trajectory T
    for j in range(len(T)):
        arm_goal_arr = T[j].q
        print("Andando verso il prossimo punto...")
        time_resolution = max_time / (steps/3)
        time_to_wait = 0

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
        print("prosegui: ")
        time.sleep(5)


    print("Traiettoria bottone ON/OFF eseguita, si va alla presa")
    
    while not_finished:
        request_topic.publish(request_msg)
        print("aspetto...")
        try:
            tag_pose = rospy.wait_for_message("/button_to_press", PoseStamped, timeout=rospy.Duration(10))
            tag_pose = tag_pose.pose
            not_finished = True
        except rospy.exceptions.ROSException as e:
            print("Timeout tag_pose")
            print(e)
            return
        
        if tag_pose.position.x == 0 and tag_pose.position.y == 0 and tag_pose.position.z == 0:
            print("Error reciving the tag pose")
            not_finished = False
            break

    T_pose = matrix4x4_from_pose(tag_pose)

    print("tag pose in camera frame: ", tag_pose)
    ## -44mm in x, 0 in y, 65mm in z (base_link)
    tf_camera = tf_camera_from_base(q_pres) #### da definire
    tag_pose_in_base = tf_camera@T_pose
    tag_angles = tf.transformations.euler_from_matrix(tag_pose_in_base[:3,:3])
    tag_position = tf_camera@np.array([[tag_pose.position.x], [tag_pose.position.y], [tag_pose.position.z], [1]])
    print("tag position in base link frame: ", tag_position.transpose()[0])
    print("tag angles in base link frame: ", tag_angles)
    angle_shift = tag_angles[2]+pi/2


    task = False
    c_all= [np.array([0.115, 0.225, 0.06]),
            np.array([0.115, 0.225, 0.03]),
            np.array([0.115, 0.225, 0.06]),
            np.array([0.25,  0.00, 0.25]),
            np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]]),
            np.array([tag_position[0,0]+0.03*cos(angle_shift), tag_position[1,0]+0.03*sin(angle_shift), tag_position[2,0]]),
            np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]])]
    print("c_all: ", c_all)
    R_all = [Ry(np.pi),Ry(np.pi),Ry(np.pi),Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2)]
    elbow_all = [True,True,True,True,True,True,True]


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
            command = Float32()
            command.data = 240  ###### aggiusta
            pub_ee.publish(command)
            time.sleep(5)
        if j == 2:
            print("Chiusura pinza")
            command = Float32()
            command.data = 160  ###### aggiusta
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
