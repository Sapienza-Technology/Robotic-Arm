#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import termios, tty, select

from geometry_msgs.msg import Twist

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

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Set up keyboard input
old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

# Set up dictionary of key bindings
bindings = {
    'w': [0, 0.1, 0, 0, 0, 0],
    'a': [0.1, 0, 0, 0, 0, 0],
    's': [0, -0.1, 0, 0, 0, 0],
    'd': [-0.1, 0, 0, 0, 0, 0],
    'q': [0, 0, 0.1, 0, 0, 0],
    'e': [0, 0, -0.1, 0, 0, 0],
    'z': [0, 0, 0, 0.1, 0, 0],
    'x': [0, 0, 0, -0.1, 0, 0],
    'c': [0, 0, 0, 0, 0.1, 0],
    'v': [0, 0, 0, 0, -0.1, 0],
    'b': [0, 0, 0, 0, 0, 0.1],
    'n': [0, 0, 0, 0, 0, -0.1],
}

# Print instructions
print("Use WASD keys to move the arm. Press '.' to quit.")

joint_goal = move_group.get_current_joint_values()

# Loop until user quits
while True:
    # Read keyboard input
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
        if key == '.':
            break
        if key in bindings:
            joint_goal = [sum(x) for x in zip(joint_goal, bindings[key])]
            move_group.go(joint_goal, wait=True)
            #move_group.stop()
            print("Joint values:", joint_goal)
    else:
        move_group.stop()

    # Publish zero Twist message to stop the arm from moving
    pub.publish(Twist())

# Restore keyboard settings
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
print("============ Joint values: ", joint_goal)
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
ret = move_group.go(joint_goal, wait=True)

print("============ Returned: ", ret)

#move_group.execute(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

