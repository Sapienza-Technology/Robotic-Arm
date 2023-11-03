#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

gripper_publisher = rospy.Publisher(
                    '/gripper_command',
                    String,
                    queue_size=10)

rospy.sleep(1)

print("============ Sending opening/closing signals to gripper")

message_close = String()
message_close.data = 'open'
gripper_publisher.publish(message_close)
rospy.sleep(12)

message_close = String()
message_close.data = 'close'
gripper_publisher.publish(message_close)
rospy.sleep(12)

message_close = String()
message_close.data = 'open'
gripper_publisher.publish(message_close)
rospy.sleep(12)
