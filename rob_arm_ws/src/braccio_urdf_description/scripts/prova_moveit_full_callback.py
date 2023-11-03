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
from math import pi, exp, cos, sin, tan
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global not_finished 
global step
global home_pose
global home_joint
global gripper_publisher
global closed
global x_dist
global next_marker
global salendo
global center_pose

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


def callback_got_marker(data, group, scene, gripper_publisher, end_publisher):
    global not_finished 
    global step
    global home_joint
    global closed
    global next_marker
    global salendo
    global x_dist
    global center_pose

    if not closed:
        message_close = String()
        message_close.data = 'close'
        gripper_publisher.publish(message_close)
        rospy.sleep(5)
        closed = True

    print(data.id)
    if data.id > 0 and data.id < 100:
        posTag = data.pose
        x_dist = data.pose.position.x
        posTag.position.z = posTag.position.z - 0.055
        posTag.position.y = posTag.position.y*0.8 + group.get_current_pose().pose.position.y*0.2
        posTag.position.x = posTag.position.x - 0.0186 - 0.0485 - 0.015 - 0.050*exp(-0.15*step)
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

        step = step + 1
        if exp(-0.15*step) < 0.5:
            not_finished = False

        if not not_finished:
            next_marker = next_marker + 1
            print("Next_marker: ")
            print(next_marker)
            end_publisher.publish(next_marker)
            
            posTag = group.get_current_pose().pose
            posTag.position.x = posTag.position.x - 0.02
            group.set_pose_target(posTag)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            rospy.sleep(3)

            #group.set_joint_value_target(home_joint)
            #plan1 = group.plan()
            #rospy.sleep(1)
            #group.go(wait=True)
            #group.clear_pose_targets()
            #rospy.sleep(5)

            not_finished = True
            step = 0
    elif data.id > 100:
        #group.set_pose_target(home_pose)
        #plan1 = group.plan()
        #rospy.sleep(1)
        #group.go(wait=True)
        #group.clear_pose_targets()
        rospy.sleep(5)

        rospy.signal_shutdown("End of the task")
    else:
        posTag = group.get_current_pose().pose
        shift_y = -0.087

        if posTag.position.y + shift_y < -0.095:           #0.14
            if posTag.position.z - 0.15 < 0.03:
                group.set_pose_target(center_pose)
                plan1 = group.plan()
                rospy.sleep(1)
                group.go(wait=True)
                rospy.sleep(5)
                posTag = home_pose
            else:
                posTag.position.z = posTag.position.z - 0.15
            posTag.position.y = 0.087
        else:
            posTag.position.y = posTag.position.y + shift_y
        
        
        group.set_pose_target(posTag)
        plan1 = group.plan()
        rospy.sleep(1)
        group.go(wait=True)
        rospy.sleep(5)

        not_finished = True

    


def main():
    global not_finished
    global step
    global home_pose
    global home_joint
    global gripper_publisher
    global closed
    global next_marker
    global salendo
    global center_pose
    next_marker = 0.0
    salendo = False

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