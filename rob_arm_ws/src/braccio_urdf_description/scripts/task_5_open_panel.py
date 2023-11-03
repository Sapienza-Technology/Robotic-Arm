#!/usr/bin/env python3
import array
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from math import atan, pi, exp, cos, sin, tan
from tag_utils.srv import GetTag
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global not_finished 
global step
global home_pose
global home_joint
global gripper_publisher
global x_dist
global next_marker
global center_pose
global left_ob_pos
global seen
global phase

def add_panel(scene, x_dist):
    if x_dist < 0.41:
        x_dist = 0.41
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = x_dist + 0.005
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


def go_to_see_right(group, scene, tag):
    global x_dist

    if tag == 1:
        poseIni = group.get_current_pose().pose
        poseIni.position.x =  0.1828
        poseIni.position.y = -0.1643
        poseIni.position.z =  0.0791
        euler=(0.0, pi, -pi/2)
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
        poseIni = group.get_current_pose().pose
        poseIni.position.x =  x_dist - 0.150*sin(pi/10) - 0.16*sin(4*pi/10)
        poseIni.position.y =  -0.150 - 0.150*cos(pi/10) + 0.16*cos(4*pi/10)
        poseIni.position.z =  0.2055
        euler=(0.0, 2*pi/3, -pi/10)
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

    elif tag == 3:
        poseIni = group.get_current_pose().pose
        poseIni.position.x =  x_dist - 0.150*sin(pi/10) - 0.16*sin(4*pi/10) + 0.05*sin(pi/10)
        poseIni.position.y =  -0.150 - 0.150*cos(pi/10) + 0.16*cos(4*pi/10) + 0.05*cos(pi/10)
        poseIni.position.z =  0.2555
        euler=(0.0, 2*pi/3, -pi/10)
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


def move_around(group, scene):
    jointIni = group.get_current_joint_values()
    jointIni[5] -= pi/12
    group.set_joint_value_target(jointIni)
    plan1 = group.plan()
    group.go(wait=True)


def wait_for_message_custom(topic, msg_type, timeout=3):
    mess = None
    try:
        mess = rospy.wait_for_message(topic, msg_type, timeout=timeout)
        mess = mess.data
    except rospy.exceptions.ROSException:
        mess = [-1,0,0,0]
    return mess



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
    global x_dist
    next_marker = 0.0
    x_dist = 0.0
    left_ob_pos = 0.0

    ### Inizializzazione moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    gripper_publisher = rospy.Publisher(
                        '/gripper_command',
                        String,
                        queue_size=20)

    ### Inizializzazione publisher
    request_tag = rospy.Publisher(
                        '/obj1_get_tag',
                        std_msgs.msg.Float32,
                        queue_size=1)
    
    skip_tag    = rospy.Publisher(
                        '/obj1_skip_tag',
                        std_msgs.msg.Float32,
                        queue_size=1)
    
    
    ### Inizializzazione variabili
    not_finished = True
    step = 0
    seen = {}
    # initial position of the arm [0.0, -120.0, 100.0, 20.0, 90.0, -90.0]
    home_pose  = group.get_current_pose().pose
    home_joint = group.get_current_joint_values()

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

    try:
        get_tag = rospy.ServiceProxy('get_tag', GetTag)
        tag_12 = get_tag(12).tag #list of 3 elements (x,y,z)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

    if x_dist != 0.0:
        add_panel(scene,x_dist)
    if tag_12[0] > 0.0:
        ### open gripper
        i = 0
        while i < 5:
            message_open = std_msgs.msg.String()
            message_open.data = 'open'
            gripper_publisher.publish(message_open)
            rospy.sleep(0.1)
            i += 1
        rospy.sleep(5)

        ### go to right
        go_to_see_left(group, scene, 3)
        go_to_see_right(group, scene, 2)
        poseIni = group.get_current_pose().pose
        poseIni.position.x =  x_dist - 0.150*sin(pi/10) - 0.16*sin(4*pi/10)
        poseIni.position.y =  tag_12[1] + 0.16*cos(4*pi/10)
        poseIni.position.z =  tag_12[2] + 0.035 + 0.0175 + 0.071*sin(pi/3)
        euler=(0.0, 2*pi/3, -pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        poseIni2 = group.get_current_pose().pose
        poseIni2.position.x =  x_dist - 0.150*sin(pi/10) - 0.16*sin(4*pi/10) + 0.0323*sin(4*pi/10)
        poseIni2.position.y =  tag_12[1] + 0.16*cos(4*pi/10) - 0.0323*cos(4*pi/10)
        poseIni2.position.z =  tag_12[2] + 0.035 + 0.0175 + 0.071*sin(pi/3)
        euler=(0.0, 2*pi/3, -pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni2.orientation.x = quaternion[0]
        poseIni2.orientation.y = quaternion[1]
        poseIni2.orientation.z = quaternion[2]
        poseIni2.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        waypoints.append(copy.deepcopy(poseIni2))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)

        ### close gripper
        i = 0
        while i < 5:
            message_close = std_msgs.msg.String()
            message_close.data = 'semi_close'
            gripper_publisher.publish(message_close)
            rospy.sleep(0.1)
            i += 1
        rospy.sleep(5)

        ### go up
        poseIni2 = group.get_current_pose().pose
        poseIni2.position.z = poseIni2.position.z + 0.03 
        euler=(0.0, 2*pi/3, -pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni2.orientation.x = quaternion[0]
        poseIni2.orientation.y = quaternion[1]
        poseIni2.orientation.z = quaternion[2]
        poseIni2.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        waypoints.append(copy.deepcopy(poseIni2))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)


    
    
    rospy.sleep(5)
    print("End of the task 1")
    return

    

if __name__=='__main__':
    main()