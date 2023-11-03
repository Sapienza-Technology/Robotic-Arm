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
from math import pi, exp
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



is_movement_finshed = rospy.Publisher(
                                    '/movement_status',
                                    std_msgs.msg.Float32,
                                    queue_size=20)



data = rospy.wait_for_message('/tag_pose', visualization_msgs.msg.Marker)
numTag = data.id
not_finished = True
step = 0

home_pose  = group.get_current_pose().pose
home_joint = group.get_current_joint_values()


message_close = String()
message_close.data = 'close'
gripper_publisher.publish(message_close)
rospy.sleep(10)


pose_goal = geometry_msgs.msg.Pose()
euler=(0.0, pi/2, 0.0)
quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
pose_goal.orientation.x = quaternion[0]
pose_goal.orientation.y = quaternion[1]
pose_goal.orientation.z = quaternion[2]
pose_goal.orientation.w = quaternion[3]
print(quaternion)
pose_goal.position.x = 0.35
pose_goal.position.y = home_pose.position.y - 0.2
pose_goal.position.z = home_pose.position.z - 0.2
group.set_pose_target(pose_goal)

plan1 = group.plan()
rospy.sleep(5)
group.go(wait=True)
group.clear_pose_targets()


while not_finished:
    posTag = data.pose
    posTag.position.z = posTag.position.z - 0.055
    posTag.position.x = posTag.position.x - 0.0186 - 0.0485 - 0.060*exp(-0.2*step)
    euler=(0.0, pi/2, 0.0)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    posTag.orientation.x = quaternion[0]
    posTag.orientation.y = quaternion[1]
    posTag.orientation.z = quaternion[2]
    posTag.orientation.w = quaternion[3]

    group.set_pose_target(posTag)
    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    rospy.sleep(10)

    step = step + 1
    if exp(-0.2*step) < 0.3:
        not_finished = False

    posTag = group.get_current_pose().pose
    posTag.position.x = posTag.position.x - 0.03

    group.set_pose_target(posTag)
    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    rospy.sleep(5)

print("============ Done Button")
is_movement_finshed.publish(1.0)
    
#posTag = group.get_current_pose().pose
#posTag.position.y = posTag.position.y - 0.1
#posTag.position.z = posTag.position.z - 0.166



group.set_joint_value_target(home_joint)
plan1 = group.plan()
rospy.sleep(1)
group.go(wait=True)
rospy.sleep(5)