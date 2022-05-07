#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def trajectory_action_test():
    print('test1')
    arm_joint_names = ['giunto0', 'giunto1', 'giunto2', 'giunto3', 'giunto4', 'giunto5']
    arm_goal0 = [0, 0, 0, 0, 0, 0]
    arm_velocities0 = [0, 0, 0, 0, 0, 0]
    arm_goal1 = [0, 1, 1, 0, 0, 0]
    arm_goal2 = [1, 2, 1.5, 0, 0, 0]
    #arm_velocities1 = [1, 0.5, 0, 0, 0, 0]
    #arm_velocities2 = [0, 0, 0, 0, 0, 0]
    
    arm_client = actionlib.SimpleActionClient('braccio_urdf/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for follow_joint_trajectory server")
    arm_client.wait_for_server()
    rospy.loginfo("Connected to follow_joint_trajectory server")
    rospy.sleep(1)
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joint_names
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
    arm_client.send_goal(arm_goal_pos)
    rospy.loginfo("Send goal to the trajectory server successfully!")
    arm_client.wait_for_result()
    return arm_client.get_result()

def main():
    rospy.init_node('send_trajectory')
    result = trajectory_action_test()
    print('Result: ', result)

main()