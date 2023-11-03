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

def callback_got_marker(data, group):
    numTag = data.id
    not_finished = True
    step = 0
    while not_finished:
        posTag = data.pose
        posTag.position.z = posTag.position.z - 0.055
        posTag.position.x = posTag.position.x - 0.0186 - 0.0485 - 0.050*exp(-0.2*step)
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
        
    posTag = group.get_current_pose().pose
    posTag.position.y = posTag.position.y - 0.1
    posTag.position.z = posTag.position.z - 0.166

    group.set_pose_target(posTag)
    plan1 = group.plan()
    rospy.sleep(1)
    group.go(wait=True)
    rospy.sleep(5)


def main():
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

    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)

    is_movement_finshed = rospy.Publisher(
                                        '/movement_status',
                                        std_msgs.msg.Float32,
                                        queue_size=20)

    marker_subscriber = rospy.Subscriber(
                                        '/tag_pose',
                                        visualization_msgs.msg.Marker,
                                        queue_size=20,
                                        callback=lambda data: callback_got_marker(data, group))

    rospy.sleep(1)



    print("============ Sending closing gripper")
    message_close = String()
    message_close.data = 'close'
    gripper_publisher.publish(message_close)
    rospy.sleep(10)


    #planning_frame = group.get_planning_frame()
    #print("============ Reference frame: %s" % planning_frame)

    #eef_link = group.get_end_effector_link()
    #print("============ End effector: %s" % eef_link)

    #group_names = robot.get_group_names()
    #print("============ Robot Groups:", robot.get_group_names())

    #print("============ Printing robot state")
    #print(robot.get_current_state())

    #print("============ Printing robot pose")
    #print(group.get_current_pose().pose)

    #print("============ Printing robot joint values")
    #print(group.get_current_joint_values())


    print("============ Generating plan 1")
    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, 0.0)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.35
    pose_goal.position.y = -0.05
    pose_goal.position.z = 0.35
    group.set_pose_target(pose_goal)

    group.num_planning_attempts = 20

    plan1 = group.plan()

    rospy.sleep(5)

    print("============ Executing plan1")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    #display_trajectory_publisher.publish(display_trajectory)

    print("============ Waiting for plan1...")
    rospy.sleep(5)

    group.go(wait=True)

    group.clear_pose_targets()


    print("============ Generating plan 2")
    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, -pi/4)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.3
    pose_goal.position.y = -0.05
    pose_goal.position.z = 0.35
    group.set_pose_target(pose_goal)

    plan3 = group.plan()

    rospy.sleep(5)

    print("============ Executing plan2")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    #display_trajectory_publisher.publish(display_trajectory)

    print("============ Waiting for plan2...")
    rospy.sleep(5)

    group.go(wait=True)

    group.clear_pose_targets()

    print("Sending closing gripper")
    message_open = String()
    message_open.data = 'close'
    gripper_publisher.publish(message_open)
    rospy.sleep(5)


    group_variable_values = group.get_current_joint_values()
    print("============ Joint values: ", group_variable_values)

    group_variable_values[0] = 0.0
    group_variable_values[1] = -pi/2
    group_variable_values[2] = 0.0
    group_variable_values[3] = -pi/2
    group_variable_values[4] = 0.0
    group_variable_values[5] = 0.0
    group.set_joint_value_target(group_variable_values)

    plan2 = group.plan()

    rospy.sleep(5)

    print("============ Executing plan3")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan2)
    #display_trajectory_publisher.publish(display_trajectory)

    print("============ Waiting for plan3")
    rospy.sleep(5)

    group.go(wait=True)

    group.clear_pose_targets()


