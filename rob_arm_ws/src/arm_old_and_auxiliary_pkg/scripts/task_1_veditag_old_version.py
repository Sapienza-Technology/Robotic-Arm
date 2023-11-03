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
from std_msgs.msg import String
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
    global left_ob_pos

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

    elif tag == 2:
        poseIni = group.get_current_pose().pose
        poseIni.position.x =  x_dist - 150*sin(pi/10) - 0.15*cos(pi/10)
        poseIni.position.y =  -150 - 150*cos(pi/10) + 0.15*sin(pi/10)
        poseIni.position.z =  0.2055
        euler=(0.0, 2*pi/3, -pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]


def go_to_see_left(group, scene, tag):
    global left_ob_pos
    
    if tag == 1:
        #### for tag 11
        poseIni = group.get_current_pose().pose
        poseIni.position.x = left_ob_pos - 0.1*cos(pi/10)
        poseIni.position.y = 150 + 190*cos(pi/10) - 0.1*sin(pi/10)

        z_tag = 0.475 + 0.02 - poseIni.position.z
        tilt_up = atan(z_tag/0.1)
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
        euler=(0.0, pi, 0.0)
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
    
    else:
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

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)


def move_around(group, scene):
    jointIni = group.get_current_joint_values()
    jointIni[5] += pi/12
    group.set_joint_value_target(jointIni)
    plan1 = group.plan()
    group.go(wait=True)




def callback_got_marker(data, group, scene, gripper_publisher, end_publisher):
    global home_joint
    global next_marker
    global x_dist
    global left_ob_pos
    global center_pose
    global seen
    global phase
    global not_finished

    
    print(data.id)


    if phase == 0:
        #### Fase dei Bottoni
        if data.id < 100:
            posTag = data.pose

            x_dist = data.pose.position.x
            left_ob_pos = x_dist - 190*sin(pi/10)
            

            posTag.position.z = posTag.position.z - 0.055
            posTag.position.y = posTag.position.y*0.8 + group.get_current_pose().pose.position.y*0.2
            posTag.position.x = posTag.position.x - 0.0186 - 0.0485 - 0.015 - 0.040
            euler=(0.0, pi/2, 0.0)
            quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
            posTag.orientation.x = quaternion[0]
            posTag.orientation.y = quaternion[1]
            posTag.orientation.z = quaternion[2]
            posTag.orientation.w = quaternion[3]


            add_panel(scene, x_dist)

            group.set_pose_target(posTag)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            group.clear_pose_targets()
            rospy.sleep(5)


            if data.id not in seen:
                seen[data.id] = posTag
                if len(seen) == 9:
                    phase = 1
                    waypoints = []
                    waypoints.append(copy.deepcopy(center_pose))
                    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
                    group.execute(plan, wait=True)
                    go_to_see_left(group, scene, 1)


        else:
            posTag = group.get_current_pose().pose
            shift_y = -0.095

            if posTag.position.y + shift_y < -0.1:           #0.14
                if posTag.position.z - 0.15 < 0.03:
                    group.set_pose_target(center_pose)
                    plan1 = group.plan()
                    rospy.sleep(1)
                    group.go(wait=True)
                    rospy.sleep(5)
                    posTag = home_pose
                else:
                    posTag.position.z = posTag.position.z - 0.15
                posTag.position.y = 0.095
            else:
                posTag.position.y = posTag.position.y + shift_y
            
            
            group.set_pose_target(posTag)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            rospy.sleep(5)

            not_finished = True

    elif phase == 1:
        #### Fase zone IMU (10 e 11)
        if data.id < 100:
            posTag = data.pose
            if len(seen) == 9:
                seen[data.id] = posTag
                go_to_see_left(group, scene, 2)
            else:
                seen[data.id] = posTag
                go_to_see_left(group, scene, 3)
                phase = 2
                go_to_see_right(group, scene, 1)
        else:
            if data.id < 100:
                move_around(group, scene)

    elif phase == 2:
        #### Fase coperchio e (rompi)scatola
        if data.id < 100:
            posTag = data.pose
            if len(seen) == 11:
                seen[data.id] = posTag
                go_to_see_right(group, scene, 2)
            elif len(seen) == 12:
                seen[data.id] = posTag
                go_to_see_right(group, scene, 3)
            else:
                seen[data.id] = posTag
                phase = 3

    else:
        rospy.sleep(5)
        rospy.signal_shutdown("End of the task")
        return



def main():
    global seen
    global step
    global home_pose
    global home_joint
    global gripper_publisher
    global closed
    global next_marker
    global center_pose
    global left_ob_pos
    global phase
    next_marker = 0.0
    left_ob_pos = None

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

    

    is_movement_finshed = rospy.Publisher(
                                        '/movement_status',
                                        std_msgs.msg.Float32,
                                        queue_size=20)
    

    not_finished = True
    step = 0

    seen = {}

    # initial position of the arm [0.0, -120.0, 100.0, 20.0, 90.0, -90.0]
    
    home_pose  = group.get_current_pose().pose
    home_joint = group.get_current_joint_values()

    
#    message_close = String()
#    message_close.data = 'close'
#    gripper_publisher.publish(message_close)
#    rospy.sleep(12)
    closed = False


    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, 0.0)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.30
    pose_goal.position.y = home_pose.position.y
    pose_goal.position.z = home_pose.position.z
    group.set_pose_target(pose_goal)

    
    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    #group.clear_pose_targets()
    rospy.sleep(5)


    pose_goal.position.y = 0.09
    pose_goal.position.z = 0.40
    group.set_pose_target(pose_goal)

    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    #group.clear_pose_targets()
    rospy.sleep(5)

    first_not_seen = True
    while first_not_seen:
        print("Approaching")
        mess = rospy.wait_for_message('/tag_pose', visualization_msgs.msg.Marker, timeout=1)
        if mess.id == 0:
            pose_goal.position.x += 0.01
            group.set_pose_target(pose_goal)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            #group.clear_pose_targets()
            rospy.sleep(5)
        else:
            first_not_seen = False


    home_pose  = group.get_current_pose().pose
    center_pose = group.get_current_pose().pose
    center_pose.position.y = 0
    center_pose.position.z = 0.2

    print('mi sono mosso?')


    marker_subscriber = rospy.Subscriber(
                                        '/tag_pose',
                                        visualization_msgs.msg.Marker,
                                        queue_size=1,
                                        callback=lambda data: callback_got_marker(data, group, scene, gripper_publisher, is_movement_finshed))

    
    rospy.spin()

    

if __name__=='__main__':
    main()