#!/usr/bin/env python3
import array
from getch import getch
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


def main():
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
    

    # initial position of the arm [0.0, -120.0, 100.0, 20.0, 90.0, -90.0]
    
    home_pose  = group.get_current_pose().pose
    home_joint = group.get_current_joint_values()

    
    character = 'a'
    while(character != 'c'):
        character = getch()
        print(character)

        pose_goal = group.get_current_pose().pose
        euler=euler_from_quaternion([pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w])

        eulerx = euler[0]
        eulery = euler[1]
        eulerz = euler[2]

        if character == 'w': 
            #p1 = move_gazebo(p1, 'up')
            pose_goal.position.z += 0.005
        elif character == 's': 
            pose_goal.position.z -= 0.005
        elif character == 'd': 
            pose_goal.position.y -= 0.005
        elif character == 'a': 
            pose_goal.position.y += 0.005
        elif character == 'q': 
            pose_goal.position.x += 0.005
        elif character == 'e': 
            pose_goal.position.x -= 0.005
        # for rotations
        elif character == 'i': 
            eulery += 0.05
        elif character == 'k': 
            eulery -= 0.05
        elif character == 'l': 
            eulerx += 0.05
        elif character == 'j': 
            eulerx -= 0.05
        elif character == 'o':
            eulerz += 0.05
        elif character == 'u': 
            eulerz -= 0.05

        quaternion = quaternion_from_euler(eulerx, eulery, eulerz)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        print(quaternion)
        group.set_pose_target(pose_goal)
        #plan1 = group.plan()
        #rospy.sleep(0.5)
        group.go(wait=False)
        #group.clear_pose_targets()


    rospy.spin()

    

if __name__=='__main__':
    main()