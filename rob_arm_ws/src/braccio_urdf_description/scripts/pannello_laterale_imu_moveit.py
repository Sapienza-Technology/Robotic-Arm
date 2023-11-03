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
from math import pi, exp, sin, cos
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
global salendo


def add_panel(scene):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = x_dist + 0.05
    box_pose.pose.position.z = 0.270
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.01, 0.3, 0.54))

def callback_got_marker(data, group, gripper_publisher, end_publisher):
    global not_finished 
    global step
    global home_joint
    global x_dist
    global next_marker
    global salendo


    print(data.id)
    if data.id == 12:  #11
        posTag = data.pose
        x_dist = data.pose.position.x
        posTag.position.z = posTag.position.z - 0.055
        posTag.position.y = posTag.position.y - (0.0186 + 0.0485 + 0.015 + 0.040)*sin(pi/10)
        posTag.position.x = posTag.position.x - (0.0186 + 0.0485 + 0.015 + 0.040)*cos(pi/10)
        euler=(0.0, pi/2, pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        posTag.orientation.x = quaternion[0]
        posTag.orientation.y = quaternion[1]
        posTag.orientation.z = quaternion[2]
        posTag.orientation.w = quaternion[3]

        group.set_pose_target(posTag)
        plan1 = group.plan()
        rospy.sleep(1)
        group.go(wait=True)
        group.clear_pose_targets()
        rospy.sleep(5)
    
    elif data.id == 10:
        step = step + 1
        if exp(-0.15*step) < 0.5:
            not_finished = False

        if not not_finished:
            next_marker = next_marker + 1
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
#    else:
#        posTag = group.get_current_pose().pose
#        shift_y = -0.09
#        if posTag.position.z > 0.5:
#            salendo = False
#        if posTag.position.y + shift_y < -0.09:           #0.14
#            if posTag.position.z < 0.05 or salendo:
#                posTag.position.z = posTag.position.z + 0.15      #0.165
#                salendo = True
#            else:
#                posTag.position.z = posTag.position.z - 0.15
#            posTag.position.y = 0.09
#        else:
#            posTag.position.y = posTag.position.y + shift_y
#        
#        
#        group.set_pose_target(posTag)
#        plan1 = group.plan()
#        rospy.sleep(1)
#        group.go(wait=True)
#        rospy.sleep(5)
#
#        not_finished = True

    


def main():
    global not_finished
    global step
    global home_pose
    global home_joint
    global gripper_publisher
    global x_dist
    global next_marker
    global salendo
    next_marker = 0.0
    salendo = False

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    add_panel(scene)

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
    rospy.sleep(3)
    x_dist = 0.42

    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, 0.0)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.35
    pose_goal.position.y = home_pose.position.y
    pose_goal.position.z = home_pose.position.z
    group.set_pose_target(pose_goal)

    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    #group.clear_pose_targets()
    rospy.sleep(5)

    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.29
    pose_goal.position.y = 0.28
    pose_goal.position.z = 0.45
    group.set_pose_target(pose_goal)

    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    #group.clear_pose_targets()
    rospy.sleep(5)


    print('mi sono mosso?')


    marker_subscriber = rospy.Subscriber(
                                        '/tag_pose',
                                        visualization_msgs.msg.Marker,
                                        queue_size=1,
                                        callback=lambda data: callback_got_marker(data, group, gripper_publisher, is_movement_finshed))

    
    rospy.spin()

if __name__=='__main__':
    main()