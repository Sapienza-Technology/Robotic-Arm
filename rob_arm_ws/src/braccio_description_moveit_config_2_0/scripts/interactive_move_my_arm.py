#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from getch import getch

from traj_genv6 import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_my_arm", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

#move_group.go([0, -1, -1, 0, 0, 0], wait=True)

q_pres = move_group.get_current_joint_values()
q_pres[1] = (q_pres[1] + pi/2)

print("============ Current joint states (che saranno gli offset)")
print(np.degrees(q_pres))
print(np.degrees(move_group.get_current_joint_values()))
print("")

steps = 4

character=''


while(character != 'c'):
    character = getch()
    print(character)
    if character == 'w': 
        #p1 = move_gazebo(p1, 'up')
        arm_goal_arr = go_straight(True, True, q_pres, "u", steps, 0.01, False)
    elif character == 's': 
        arm_goal_arr = go_straight(True, True, q_pres, "d", steps, 0.03, False)
    elif character == 'd': 
        arm_goal_arr = go_straight(True, True, q_pres, "r", steps, 0.03, False)
    elif character == 'a': 
        arm_goal_arr = go_straight(True, True, q_pres, "l", steps, 0.03, False)
    elif character == 'q': 
        arm_goal_arr = go_straight(True, True, q_pres, "f", steps, 0.03, False)
    elif character == 'e': 
        arm_goal_arr = go_straight(True, True, q_pres, "b", steps, 0.03, False)
    # for rotations
    elif character == '6': 
        arm_goal_arr = rotate(True, True, q_pres, "r_cw", steps, 0.3, False)
    elif character == '4': 
        arm_goal_arr = rotate(True, True, q_pres, "r_ccw", steps, 0.3, False)
    elif character == '2': 
        arm_goal_arr = rotate(True, True, q_pres, "p_cw", steps, 0.3, False)
    elif character == '8': 
        arm_goal_arr = rotate(True, True, q_pres, "p_ccw", steps, 0.3, False)
    elif character == '3':
        arm_goal_arr = rotate(True, True, q_pres, "y_cw", steps, 0.3, False)
    elif character == '1': 
        arm_goal_arr = rotate(True, True, q_pres, "y_ccw", steps, 0.3, False)
    elif character == 'c':
        break

    for i in range(len(arm_goal_arr)):
        print(f"Step {i}: {arm_goal_arr[i]}")
        arm_goal_arr[i][1] = (arm_goal_arr[i][1] - pi/2)
        ret = move_group.go(arm_goal_arr[i], wait = True)
        print(f"Step {i}: {np.degrees(arm_goal_arr[i])}")
        #sleep(0.5)

    q_pres = arm_goal_arr[-1]
    q_pres[1] += pi/2

    print("============ Returned: ", ret)

    # We get the joint values from the group and change some of the values:
    #joint_goal = move_group.get_current_joint_values()
    #print("============ Joint values: ", joint_goal)

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    



#move_group.execute(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

