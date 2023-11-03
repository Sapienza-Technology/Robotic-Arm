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
from math import atan, pi, exp, cos, sin, tan
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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


print(group.get_current_pose().pose)

initPose = group.get_current_pose().pose
poseNear = group.get_current_pose().pose
msg_request=Float32()
msg_request.data = 1
step = 0
async_step = 0
variation = [[0.01,0],[0,0.01],[-0.01,0],[0,-0.01]]
counter = 0

while counter < 4:
    if step == 4:
        step = 0
    if async_step == 5:
        async_step = 1
        counter += 1

    poseNear.position.y += variation[step][0]
    poseNear.position.z += variation[step][1]

    waypoints = []
    waypoints.append(copy.deepcopy(poseNear))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(2)
    
    if async_step == 3:
        pass
    else:
        step += 1
    async_step += 1

waypoints = []
waypoints.append(copy.deepcopy(initPose))
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
group.execute(plan, wait=True)
rospy.sleep(2)



#poseIni = group.get_current_pose().pose
#poseIni.position.x =  0.4 - 0.150*sin(pi/10) - 0.15*sin(4*pi/10)
#poseIni.position.y =  -0.150 - 0.150*cos(pi/10) + 0.15*cos(4*pi/10)
#poseIni.position.z =  0.2055
#euler=(0.0, 2*pi/3, -pi/10)
#quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
#poseIni.orientation.x = quaternion[0]
#poseIni.orientation.y = quaternion[1]
#poseIni.orientation.z = quaternion[2]
#poseIni.orientation.w = quaternion[3]
#
#waypoints = []
#waypoints.append(copy.deepcopy(poseIni))
#(plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
#group.execute(plan, wait=True)
#rospy.sleep(3)
#
#print(group.get_current_pose().pose)
#
#poseIni = group.get_current_pose().pose
#poseIni.position.x = 0.146
#poseIni.position.y = 0.113
#poseIni.position.z = 0.240
#euler=(0.0, pi, 0.0)
#quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
#poseIni.orientation.x = quaternion[0]
#poseIni.orientation.y = quaternion[1]
#poseIni.orientation.z = quaternion[2]
#poseIni.orientation.w = quaternion[3]
#group.set_pose_target(poseIni)
#plan = group.go(wait=True)



#waypoints = []
#
#wpose = group.get_current_pose().pose
#wpose.position.x += 0.1  
#waypoints.append(copy.deepcopy(wpose))
#
#(plan, fraction) = group.compute_cartesian_path(
#    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
#)  # jump_threshold
#
#group.execute(plan, wait=True)