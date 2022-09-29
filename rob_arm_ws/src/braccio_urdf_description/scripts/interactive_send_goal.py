#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from roboticstoolbox import ERobot
from spatialmath import SE3
from math import pi as pi
from spatialmath.base import transl, trotx, troty, trotz
import numpy as np
from arm_functions import move, move_gazebo
from getch import getch
from pynput import keyboard

def trajectory_action_test():

    print('test with inverse kinematics')
    xacro = ERobot.URDF("/home/alessio/ROS/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/urdf/braccio_urdf_edit.xacro")

    p0 = transl(0.3, 0, 0.2)
    sol_inv0 = xacro.ikine_min(SE3(p0))

    arm_joint_names = ['giunto0', 'giunto1', 'giunto2', 'giunto3', 'giunto4', 'giunto5']

    arm_client = actionlib.SimpleActionClient('braccio_urdf/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server")
    arm_client.wait_for_server()
    rospy.loginfo("Connected to follow_joint_trajectory server")
    rospy.sleep(1)
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joint_names

    '''
    for i in range(len(arm_goal_arr)):
        traj_point = JointTrajectoryPoint()
        traj_point.positions = arm_goal_arr[i]
        traj_point.time_from_start = rospy.Duration(2*(i+1))
        arm_trajectory.points.append(traj_point)
    '''

    traj_point = JointTrajectoryPoint()
    traj_point.positions = sol_inv0.q
    traj_point.time_from_start = rospy.Duration(1)
    arm_trajectory.points.append(traj_point)

    rospy.sleep(1)
    arm_goal_pos = FollowJointTrajectoryGoal()
    arm_goal_pos.trajectory = arm_trajectory

    arm_goal_pos.goal_time_tolerance = rospy.Duration(0)

    arm_client.send_goal(arm_goal_pos)
    rospy.loginfo("Send goal to the trajectory server successfully!")
    arm_client.wait_for_result()
    print(arm_client.get_result())

    result = command_loop(xacro, arm_trajectory, arm_client, p0)

    return result

def command_loop(xacro, arm_trajectory, arm_client, p0):
    p1 = p0

    '''
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joint_names
    '''

    character=''
    while(character != 'c'):
        character = getch()

        if character == 'w': 
            p1 = move_gazebo(p1, 'up')
        elif character == 's': 
            p1 = move_gazebo(p1, 'down')
        elif character == 'd': 
            p1 = move_gazebo(p1, 'right')
        elif character == 'a': 
            p1 = move_gazebo(p1, 'left')
        elif character == 'q': 
            p1 = move_gazebo(p1, 'forward')
        elif character == 'e': 
            p1 = move_gazebo(p1, 'backwards')
        
        sol_inv1 = xacro.ikine_min(SE3(p1))

        traj_point = JointTrajectoryPoint()
        traj_point.positions = sol_inv1.q
        traj_point.time_from_start = rospy.Duration(1)
        arm_trajectory.points[0] = traj_point

        arm_goal_pos = FollowJointTrajectoryGoal()
        arm_goal_pos.trajectory = arm_trajectory
        arm_goal_pos.goal_time_tolerance = rospy.Duration(0)

        arm_client.send_goal(arm_goal_pos)
        #rospy.loginfo("Send goal to the trajectory server successfully!")
        #arm_client.wait_for_result()
        #print(arm_client.get_result())

    return arm_client.get_result()


def main():
    rospy.init_node('send_trajectory')
    result = trajectory_action_test()
    print('Result: ', result)


main()