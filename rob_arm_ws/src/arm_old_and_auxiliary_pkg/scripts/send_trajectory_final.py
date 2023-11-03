#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from roboticstoolbox import ERobot
from spatialmath import SE3
from spatialmath.base import transl

from traj_genv6 import *

def trajectory_action_test():

    print('test with inverse kinematics')

    '''
    P0 = SE3(transl(0.3, 0, 0.2))
    sol_inv0 = xacro.ikine_min(P0)
    P1 = SE3(transl(0.45, 0.15, 0.3))
    sol_inv1 = xacro.ikine_min(P1)
    P2 = SE3(transl(0.6, -0.1, 0.5))
    sol_inv2 = xacro.ikine_min(P2)
    P3 = SE3(transl(0.6, -0.1, 0.3))
    sol_inv3 = xacro.ikine_min(P3)
    print(P1)
    print(P2)
    print(sol_inv1)
    print(sol_inv2.q)
    '''

    p = []
    arm_goal_arr = []

    p = [SE3(transl(0.3, 0, 0.2)),  #starting point
         SE3(transl(0.42, 0.2, 0.43)), SE3(transl(0.45, 0.2, 0.43)), SE3(transl(0.42, 0.2, 0.43)),      #first point
         SE3(transl(0.42, -0.06, 0.40)), SE3(transl(0.45, -0.06, 0.40)), SE3(transl(0.42, -0.06, 0.40)),   #second point
         SE3(transl(0.42, 0.04, 0.33)), SE3(transl(0.45, 0.04, 0.33)), SE3(transl(0.42, 0.04, 0.33)),   #third point
         SE3(transl(0.3, 0, 0.2))]  #final position = start

    """     for pos in p:
        arm_goal_arr.append(xacro.ikine_min(pos).q)
    """
    T = premade_traj("prova", 100, np.zeros(6))

    print(T)
    print(T[0].q)
    print(arm_goal_arr)

    arm_goal_arr = T[0].q


    arm_joint_names = ['giunto0', 'giunto1', 'giunto2', 'giunto3', 'giunto4', 'giunto5']
    '''
    arm_goal0 = sol_inv0.q
    #arm_velocities0 = [0, 0, 0, 0, 0, 0]
    arm_goal1 = sol_inv1.q
    arm_goal2 = sol_inv2.q
    arm_goal3 = sol_inv3.q
    #arm_velocities1 = [1, 0.5, 0, 0, 0, 0]
    #arm_velocities2 = [0, 0, 0, 0, 0, 0]
    '''

    ''' decommenta per testare senza cinematica inversa
    print('test1')
    arm_joint_names = ['giunto0', 'giunto1', 'giunto2', 'giunto3', 'giunto4', 'giunto5']
    arm_goal0 = [0, 0, 0, 0, 0, 0]
    arm_velocities0 = [0, 0, 0, 0, 0, 0]
    arm_goal1 = [0, 1, 1, 0, 0, 0]
    arm_goal2 = [1, 2, 1.5, 0, 0, 0]
    #arm_velocities1 = [1, 0.5, 0, 0, 0, 0]
    #arm_velocities2 = [0, 0, 0, 0, 0, 0]
    '''

    ''' decommenta per testare un giunto solo
    
    arm_joint_names = ['giunto0']
    arm_goal0 = [0]
    arm_goal1 = [1]
    arm_goal2 = [0]
    
    '''

    
    arm_client = actionlib.SimpleActionClient('braccio_urdf/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server")
    arm_client.wait_for_server()
    rospy.loginfo("Connected to follow_joint_trajectory server")
    rospy.sleep(1)
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joint_names

    for i in range(len(arm_goal_arr)):
        traj_point = JointTrajectoryPoint()
        traj_point.positions = arm_goal_arr[i]
        traj_point.time_from_start = rospy.Duration(2*(i+1))
        arm_trajectory.points.append(traj_point)

    '''
    arm_trajectory.points[2].time_from_start = rospy.Duration(4*i)
    arm_trajectory.points[2].time_from_start = rospy.Duration(4*i)
    '''


    '''
    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory.points[0].positions = arm_goal0
    #arm_trajectory.points[0].velocities = arm_velocities0
    arm_trajectory.points[0].time_from_start = rospy.Duration(5)
    arm_trajectory.points[1].positions = arm_goal1
    #arm_trajectory.points[1].velocities = arm_velocities1
    arm_trajectory.points[1].time_from_start = rospy.Duration(10)
    arm_trajectory.points[2].positions = arm_goal2
    arm_trajectory.points[2].time_from_start = rospy.Duration(15)
    arm_trajectory.points[3].positions = arm_goal3
    arm_trajectory.points[3].time_from_start = rospy.Duration(20)
    # Go back to the starting position
    arm_trajectory.points[4].positions = arm_goal0
    arm_trajectory.points[4].time_from_start = rospy.Duration(25)
    '''
    '''
    arm_trajectory.points[1].positions = arm_goal2
    arm_trajectory.points[1].velocities = arm_velocities2
    arm_trajectory.points[1].time_from_start = rospy.Duration(10)
    rospy.loginfo("Preparing for moving the arm to goal position!")
    '''
    rospy.sleep(1)
    arm_goal_pos = FollowJointTrajectoryGoal()
    arm_goal_pos.trajectory = arm_trajectory

    arm_goal_pos.goal_time_tolerance = rospy.Duration(0)

    while (True):
        arm_client.send_goal(arm_goal_pos)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        arm_client.wait_for_result()


    return arm_client.get_result()

def main():
    rospy.init_node('send_trajectory')
    result = trajectory_action_test()
    print('Result: ', result)

main()