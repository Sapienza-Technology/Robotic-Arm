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

def build_cartesian_interpol(X, angle_shift, prec_q):
    n = len(X)
    T = []
    for i in range(n-1):
        T.append(inv_kin_total_bis(X[i], Rz(angle_shift)@Ry(np.pi/2), False, True, prec_q, 0.0))
    return T

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

    pub_command = rospy.Publisher("/firmware_arm_pos", Float32MultiArray, queue_size=1)
    pub_command_vel = rospy.Publisher("/firmware_arm_vel", Float32MultiArray, queue_size=1)
    pub_ee = rospy.Publisher("/firmware_EEP_pos", Float32, queue_size=1)
    pub_ack = rospy.Publisher("/tag_ack", Bool, queue_size=1)
    pub_detect_req = rospy.Publisher("/tag_start_detect", Bool, queue_size=1)

    request_topic = rospy.Publisher("/button_request", Float32, queue_size=100)

    offsets = np.array([0, degToRad(90), degToRad(-90), 0, 0, 0])
    q_pres = np.array([0, degToRad(90), degToRad(-120), 0, 0, 0])
    q_camera = np.array([0, degToRad(90), degToRad(-120), 0, 0, 0])

    ### Inizializzo la posizione del braccio pre vedere i tag
    command_joint_values(q_pres, offsets, pub_command, pub_command_vel)
        
    #time.sleep(5)

    tot_time = 2
    steps = 150
    steps_cart = 2


    print("Schiaccia un pulsante quando vuoi detectare...")

    getch()
    
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
            tag_pose = rospy.wait_for_message("/button_to_press", PoseStamped)
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
            q_pres = np.array([0, degToRad(90), degToRad(-120+5*fixes), 0, 0, 0])
            q_camera = np.array([0, degToRad(90), degToRad(-120+5*fixes), 0, 0, 0])
            command_joint_values(q_pres, offsets, pub_command, pub_command_vel)
            if fixes == 8:
                fixes -= 1
            else:
                fixes += 1
            sleep(5)
        else:
            not_detected = False

    while not_finished:

        T_pose = matrix4x4_from_pose(tag_pose)

        print("tag pose in camera frame: ", tag_pose)
        ## -44mm in x, 0 in y, 65mm in z (base_link)
        tf_camera = tf_camera_from_base(q_camera) #### da definire
        tag_pose_in_base = tf_camera@T_pose
        tag_angles = tf.transformations.euler_from_matrix(tag_pose_in_base[:3,:3])
        tag_position = tf_camera@np.array([[tag_pose.position.x], [tag_pose.position.y], [tag_pose.position.z], [1]])
        print("tag position in base link frame: ", tag_position.transpose()[0])
        print("tag angles in base link frame: ", tag_angles)
        angle_shift = tag_angles[2]+pi/2
        #tag_position = tag_position.transpose()

        current_pos = dir_kin_total(q_pres, False, 0.0)

        #compute a matrix of dimension N x 3 of points from a starting vector of size 3 to a final vector of size 3, X is determined by the desired granularity
        start_pos=np.array([current_pos[0], current_pos[1], current_pos[2]])
        end_pos=np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]])
        print("start_pos: ", start_pos)
        print("end_pos: ", end_pos)
        distance = np.linalg.norm(end_pos-start_pos)
        resolution = int(distance/0.02)
        print(resolution)
        X = np.linspace(start_pos, end_pos, resolution)
        print("X: ", X)

        task = False
        c_all = np.array([np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]]),
                        np.array([tag_position[0,0], tag_position[1,0], tag_position[2,0]]),
                        np.array([tag_position[0,0]-0.05*cos(angle_shift), tag_position[1,0]-0.05*sin(angle_shift), tag_position[2,0]])])
        print("c_all: ", c_all)
        R_all = [Rz(angle_shift)@Ry(np.pi/2)]*(3)#,Rz(angle_shift)@Ry(np.pi/2),Rz(angle_shift)@Ry(np.pi/2)]
        elbow_all = [True]*(3)#,True,True]

        print("Continuare...")
        time.sleep(7)
        arm_goal_arr = build_cartesian_interpol(X, angle_shift, q_pres) #array of trajectories between points

        print("Traiettoria generata: eseguendo...")

        joint_val_pres = q_pres-offsets         ### joint values in the real motor world

        last_command = q_pres-offsets

        for i in range(len(arm_goal_arr)):
            print("Step: ")
            print(arm_goal_arr[i])
            command = Float32MultiArray()
            command_vel = Float32MultiArray()
            res, bool_vec = range_check(arm_goal_arr[i][:3])

            if not res:
                print("Range check failed: joint " + str(np.where(bool_vec == 0)[0]+1) + " out of range")
                command.data = last_command
            else:
                command.data = arm_goal_arr[i] - offsets
                last_command = command.data

            delta_joint = arm_goal_arr[-1] - arm_goal_arr[0]
            joint_val_pres = command.data
            command_vel.data = delta_joint/tot_time
            pub_command_vel.publish(command_vel)
            time.sleep(tot_time/resolution)
            pub_command.publish(command)

            print(f"{i}. Inviando: {command.data}")
            print(f"{i}. Inviando velocità: {command_vel.data}")

            print("time_to_wait: ", tot_time/resolution)


        q_pres = command.data + offsets
        print("Traiettoria bottone eseguita, si va al prossimo")

        T = era_traj(task, q_pres, c_all, R_all, elbow_all, steps) #array of trajectories between points

        # For every point in trajectory T
        for j in range(len(T)):
            arm_goal_arr = T[j].q
            #print("Andando verso il prossimo punto...")


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
                command_vel.data = delta_joint/tot_time
                pub_command_vel.publish(command_vel)
                time.sleep(tot_time/steps)
                pub_command.publish(command)

                print(f"{i}. Inviando: {command.data}")
                print(f"{i}. Inviando velocità: {command_vel.data}")

                print("time_to_wait: ", tot_time/steps)
                
            print("Aspettando di arrivare al punto")
            time.sleep(5)
            print("prosegui: ")
        
        msg_ack = Bool()
        msg_ack.data = True
        pub_ack.publish(msg_ack)
        time.sleep(2)

        request_topic.publish(request_msg)
        print("Ti aspecto...")
        try:
            tag_pose = rospy.wait_for_message("/button_to_press", PoseStamped)
            tag_pose = tag_pose.pose
        except rospy.exceptions.ROSException as e:
            print("Timeout tag_pose")
            print(e)
            return

        if tag_pose.position.x == 0 and tag_pose.position.y == 0 and tag_pose.position.z == 0:
            print("End of buttons task")
            not_finished = False
            break

                
    print("Bottoni richiesti schiacciati")


    """
    print("Fra 10 secondi torniamo a zero")
    for i in range(10):
        print(i+1)
        time.sleep(1)
    """

    print("Aprendo prima di tornare allo zero...")
    command = Float32()
    command.data = 1.0
    pub_ee.publish(command)
    time.sleep(5)

    print("Tornando allo zero...")
    command = Float32MultiArray()
    command.data = q_camera-offsets
    pub_command.publish(command)

    time.sleep(5)
    
    command = Float32MultiArray()
    command.data = np.zeros(6)
    pub_command.publish(command)


main()



        