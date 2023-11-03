#!/usr/bin/env python3
import array
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import numpy as np
import tf
from std_srvs.srv import *
from tag_utils.srv import GetTag
from math import atan, pi, exp, cos, sin, tan
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global step
global gripper_publisher
global x_dist
global next_marker
global center_pose
global left_ob_pos

def add_panel(scene, x_dist):
    if x_dist < 0.41:
        x_dist = 0.41
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = x_dist + 0.005 - 0.012
    box_pose.pose.position.z = 0.270
    box_name = "central_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.3, 0.54))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = 0.275+0.02
    box_pose.pose.position.x = x_dist + 0.005 - 0.145*tan(pi/10)
    box_pose.pose.position.z = 0.180+0.360/2
    box_name = "left_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.250, 0.40))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, -pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = -(0.275+0.01)
    box_pose.pose.position.x = x_dist + 0.005 - (0.145+0.01)*tan(pi/10)
    box_pose.pose.position.z = 0.090+0.04
    box_name = "right_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.250, 0.22))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, -pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = -(0.275+0.01) - 0.05*sin(pi/10) + 0.01*cos(pi/10)
    box_pose.pose.position.x = x_dist + 0.005 - (0.145+0.01+0.01)*tan(pi/10) - 0.05*cos(pi/10) +0.01*sin(pi/10)
    box_pose.pose.position.z = 0.150+0.04+0.015
    box_name = "box_panel"
    scene.add_box(box_name, box_pose, size=(0.105, 0.150, 0.07))



def go_to_see_left(group, scene, tag):
    global left_ob_pos
    
    if tag == 1:
        #### for tag 11
        poseIni = group.get_current_pose().pose
        poseIni.position.x = left_ob_pos - 0.15*sin(4*pi/10)
        poseIni.position.y = 0.150 + 0.190*cos(pi/10) - 0.15*cos(4*pi/10)
        poseIni.position.z = 0.400

        z_tag = 0.475 + 0.02 - poseIni.position.z
        tilt_up = atan(z_tag/0.1)
        print(tilt_up)
        euler=(0.0, pi/2-tilt_up, pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)

    elif tag == 2:
        ### for tag 10
        poseIni_int = group.get_current_pose().pose
        poseIni_int.position.x = 0.146
        poseIni_int.position.y = 0.113
        poseIni_int.position.z = 0.240
        euler=(0.0, pi, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni_int.orientation.x = quaternion[0]
        poseIni_int.orientation.y = quaternion[1]
        poseIni_int.orientation.z = quaternion[2]
        poseIni_int.orientation.w = quaternion[3]


        poseIni = group.get_current_pose().pose
        poseIni.position.x = 0.18
        poseIni.position.y = 0.247
        poseIni.position.z = 0.137
        euler=(0.0, pi, pi/2)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni_int))
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)
    
    else:
        poseIni = group.get_current_pose().pose
        poseIni.position.x = 0.146
        poseIni.position.y = 0.113
        poseIni.position.z = 0.240
        euler=(0.0, pi, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)  




def main():
    global seen
    global step
    global home_pose
    global home_joint
    global gripper_publisher
    global next_marker
    global center_pose
    global left_ob_pos
    global phase
    next_marker = 0.0
    x_dist = 0.0                         #### To try without file fix it
    left_ob_pos = None

    ### Inizializzazione moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ### Inizializzazione publisher
    #request_tag = rospy.Publisher(
    #                    '/obj4_get_tag',
    #                    std_msgs.msg.Float32,
    #                    queue_size=1)
    
    gripper_publisher = rospy.Publisher(
                        '/gripper_command',
                        std_msgs.msg.String,
                        queue_size=20)
    
    
    ### Inizializzazione variabili
    # initial position of the arm [0.0, -120.0, 100.0, 20.0, 90.0, -90.0]
    current_pose  = group.get_current_pose().pose
    current_joint = group.get_current_joint_values()

    goal_angle=int(sys.argv[1])

    rospy.wait_for_service('get_tag')
    try:
        counter = 0
        for id in range(9):
            get_tag = rospy.ServiceProxy('get_tag', GetTag)
            response = get_tag(id+1) #list of 3 elements (x,y,z)
            x_dist = (x_dist*counter + response.tag[0])/(counter+1)
        left_ob_pos = x_dist - 0.190*sin(pi/10)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    

    ### Check right IMU orientation
    #try:
    #    imu_data = rospy.wait_for_message('/imu/data', Imu, timeout=1)
    #    imu_data = [imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w]
    #except rospy.exceptions.ROSException:
    #    imu_data = [0,0,0,1]

    ####### ROBA INVERSA STAI ATTENTO
    #listener=tf.TransformListener()
    #listener.waitForTransform('base_link','imu_panel',rospy.Time(0),rospy.Duration(2.0))
    #(trans,rot_imu)=listener.lookupTransform('base_link','imu_panel', rospy.Time(0))

    #q_abs = tf.transformations.quaternion_multiply(rot_imu,imu_data)                 ### da capire se necessaria

    #ee_in_imu_rotation = tf.transformations.quaternion_from_euler(pi/2,0,0)
    #ee_final_or = tf.transformations.quaternion_multiply(q_abs,ee_in_imu_rotation)

    #ee_inverse = tf.transformations.quaternion_inverse(ee_final_or)
    #rotation_check = tf.transformations.quaternion_multiply(ee_inverse,[current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w])
    #print(rotation_check)
    #if np.allclose(rotation_check, [0, 0, 0, 1]):
    #    print("IMU orientation is correct")
    #    add_angle = -pi/2
    #else:
    #    print("IMU orientation is inverted")
    #    add_angle =  -pi/2


    ###########################################
    ### Go to the imu panel

    poseIni = group.get_current_pose().pose
    poseIni.position.x = left_ob_pos - 0.11*sin(4*pi/10)                            ### da prendere da file
    poseIni.position.y = 0.150 + 0.125*cos(pi/10) - 0.11*cos(4*pi/10)
    poseIni.position.z = 0.3
    #imu_data = rospy.wait_for_message('/imu/data', Imu, timeout=1)

    ee_from_world = tf.transformations.quaternion_from_euler(0,pi/2,pi/10)
    #imu_from_ee = tf.transformations.quaternion_from_euler(0,0,pi/9+add_angle) ### vedi come ottenere goal_angle
    #ee_final_or = tf.transformations.quaternion_multiply(ee_from_world,imu_from_ee)
    poseIni.orientation.x = ee_from_world[0]
    poseIni.orientation.y = ee_from_world[1]
    poseIni.orientation.z = ee_from_world[2]
    poseIni.orientation.w = ee_from_world[3]

    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    ### orient the imu
    try:
        imu_data = rospy.wait_for_message('/imu/data', Imu, timeout=1)
        imu_data = [imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w]
    except rospy.exceptions.ROSException:
        imu_data = tf.transformations.quaternion_from_euler(0,pi/2,0)
    
    cur_rot = poseIni.orientation
    imu_data = tf.transformations.euler_from_quaternion(imu_data)
    angle_err = goal_angle - imu_data[1]
    fb_rotation = tf.transformations.quaternion_from_euler(0,0,angle_err)
    ee_fb_or = tf.transformations.quaternion_multiply([cur_rot.x,cur_rot.y,cur_rot.z,cur_rot.w],fb_rotation)
    poseIni.orientation.x = ee_fb_or[0]
    poseIni.orientation.y = ee_fb_or[1]
    poseIni.orientation.z = ee_fb_or[2]
    poseIni.orientation.w = ee_fb_or[3]

    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)


    ### attach the imu
    poseIni.position.x = left_ob_pos + (0.025 + 0.0625)*sin(4*pi/10)                ### considering gripper dimension
    poseIni.position.y = 0.150 + 0.125*cos(pi/10) + (0.025 + 0.0625)*cos(4*pi/10)
    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    ### open gripper
    i = 0
    while i < 5:
        message_open = std_msgs.msg.String()
        message_open.data = 'open'
        gripper_publisher.publish(message_open)
        rospy.sleep(0.1)
        i += 1
    rospy.sleep(5)

    ### go back
    poseIni.position.x = left_ob_pos - 0.1*sin(4*pi/10)                ### considering gripper dimension
    poseIni.position.y = 0.150 + 0.125*cos(pi/10) - 0.1*cos(4*pi/10)
    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    ### wait for points
    print("waiting for score service...  ", end="")
    rospy.wait_for_service("/erc_score")
    print("found service!")
    try:
        # create service proxy with service name and message type
        service_proxy = rospy.ServiceProxy('/erc_score',SetBool)
        response=service_proxy(True)
        if response.success: print("Success! message:",response.message)
        else: print("mannaggia non funziona")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


    ### return home
    home_joint = [0.0*pi/180, -120.0*pi/180, 100.0*pi/180, 20.0*pi/180, 90.0*pi/180, -90.0*pi/180]
    group.go(home_joint, wait=True)
    rospy.sleep(3)
    

    rospy.sleep(5)
    print("End of the task 4 \n Ctrl+C to exit")
    return

    

if __name__=='__main__':
    main()