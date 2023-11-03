#!/usr/bin/env python3
import array
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import tf
from tag_utils.srv import GetTag
from visualization_msgs.msg import Marker
from math import atan, pi, exp, cos, sin, tan
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from moveit_commander.conversions import pose_to_list
#quaternion to euler
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global not_finished 
global step
global home_pose
global home_joint
global gripper_publisher
global x_dist
global next_marker
global center_pose
global left_ob_pos
global seen
global phase

def add_panel(scene, x_dist):
    if x_dist < 0.41:
        x_dist = 0.41
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = x_dist + 0.005 - 0.012
    box_pose.pose.position.z = 0.270
    box_name = "central_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.3, 0.54))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = 0.275+0.02
    box_pose.pose.position.x = x_dist + 0.005 - 0.145*tan(pi/10)
    box_pose.pose.position.z = 0.180+0.360/2
    box_name = "left_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.250, 0.40))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, -pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = -(0.275+0.01)
    box_pose.pose.position.x = x_dist + 0.005 - (0.145+0.01)*tan(pi/10)
    box_pose.pose.position.z = 0.090+0.04
    box_name = "right_panel"
    scene.add_box(box_name, box_pose, size=(0.01, 0.250, 0.22))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    euler=(0.0, 0.0, -pi/10)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    box_pose.pose.position.y = -(0.275+0.01) - 0.05*sin(pi/10) + 0.01*cos(pi/10)
    box_pose.pose.position.x = x_dist + 0.005 - (0.145+0.01+0.01)*tan(pi/10) - 0.05*cos(pi/10) +0.01*sin(pi/10)
    box_pose.pose.position.z = 0.150+0.04+0.015
    box_name = "box_panel"
    scene.add_box(box_name, box_pose, size=(0.105, 0.150, 0.07))



def go_to_see_left(group, scene, tag):
    global left_ob_pos
    
    if tag == 1:
        #### for tag 11
        poseIni = group.get_current_pose().pose
        poseIni.position.x = left_ob_pos - 0.15*sin(4*pi/10)
        poseIni.position.y = 0.150 + 0.190*cos(pi/10) - 0.15*cos(4*pi/10)
        poseIni.position.z = 0.400

        z_tag = 0.475 + 0.02 - poseIni.position.z
        tilt_up = atan(z_tag/0.1)
        print(tilt_up)
        euler=(0.0, pi/2-tilt_up, pi/10)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)

    elif tag == 2:
        ### for tag 10
        poseIni_int = group.get_current_pose().pose
        poseIni_int.position.x = 0.146
        poseIni_int.position.y = 0.113
        poseIni_int.position.z = 0.120
        euler=(0.0, pi, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni_int.orientation.x = quaternion[0]
        poseIni_int.orientation.y = quaternion[1]
        poseIni_int.orientation.z = quaternion[2]
        poseIni_int.orientation.w = quaternion[3]


        poseIni = group.get_current_pose().pose
        poseIni.position.x = 0.18
        poseIni.position.y = 0.247
        poseIni.position.z = 0.137
        euler=(0.0, pi, pi/2)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni_int))
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)
    
    else:
        poseIni = group.get_current_pose().pose
        poseIni.position.x = 0.146
        poseIni.position.y = 0.113
        poseIni.position.z = 0.240
        euler=(0.0, pi, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        poseIni.orientation.x = quaternion[0]
        poseIni.orientation.y = quaternion[1]
        poseIni.orientation.z = quaternion[2]
        poseIni.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)  


def wait_for_message_custom(topic, msg_type, timeout=3):
    tag = None
    try:
        tag = rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.exceptions.ROSException:
        tag = Marker()
    return tag



def main():
    global seen
    global step
    global home_pose
    global home_joint
    global gripper_publisher
    global next_marker
    global center_pose
    global left_ob_pos
    global phase
    next_marker = 0.0
    left_ob_pos = None

    ### Inizializzazione moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ### Inizializzazione publisher
    request_tag = rospy.Publisher(
                        '/obj3_get_tag',
                        std_msgs.msg.Float32,
                        queue_size=1)
    
    gripper_publisher = rospy.Publisher(
                        '/gripper_command',
                        std_msgs.msg.String,
                        queue_size=20)
    
    
    ### Inizializzazione variabili
    not_finished = True
    step = 0
    seen = {}
    # initial position of the arm [0.0, -120.0, 100.0, 20.0, 90.0, -90.0]
    home_pose  = group.get_current_pose().pose
    home_joint = group.get_current_joint_values()

    i = 0
    while i < 5:
        message_open = std_msgs.msg.String()
        message_open.data = 'open'
        gripper_publisher.publish(message_open)
        rospy.sleep(0.1)
        i += 1
    rospy.sleep(5)


    ### Inizializzazione posizione iniziale per vedere primo tag
    go_to_see_left(group, scene, 2)

    msg_request=Float32()
    msg_request.data = 1
    request_tag.publish(msg_request)
    tag_req= wait_for_message_custom('/obj3_status', Marker, timeout=1)
    mess = [tag_req.id, tag_req.pose.position.x, tag_req.pose.position.y, tag_req.pose.position.z]
    ori = tag_req.pose.orientation
    variation = [[0.01,0],[0,0.01],[-0.01,0],[0,-0.01]]
    poseIni = group.get_current_pose().pose
    step = 0
    async_step = 0
    times = 1
    counter = 0

    while mess[0] < 0 and times < 6:
        if step == 4:
            step = 0
        if async_step == 5:
            async_step = 1
            counter += 1

        poseIni.position.x += variation[step][0]*times
        poseIni.position.y += variation[step][1]*times

        waypoints = []
        waypoints.append(copy.deepcopy(poseIni))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)

        request_tag.publish(msg_request)
        tag_req = wait_for_message_custom('/obj3_status', Marker, timeout=1)
        mess = [tag_req.id, tag_req.pose.position.x, tag_req.pose.position.y, tag_req.pose.position.z]
        ori = tag_req.pose.orientation


        if async_step == 3:
            pass
        else:
            step += 1
        async_step += 1
    
    print("IMU tag seen")
    rospy.sleep(3)

    ### reading imu data to align to imu
    poseIni = group.get_current_pose().pose
    poseIni.position.x = mess[1]
    poseIni.position.y = mess[2]
    poseIni.position.z = mess[3]+0.1
    
    #try:
    #    imu_data = rospy.wait_for_message('/imu/data', Imu, timeout=1)
    #    imu_data = imu_data.orientation
    #except rospy.exceptions.ROSException:
    #    imu_data = [0,0,0,1]

    ori = [ori.x, ori.y, ori.z, ori.w]
    ori = tf.transformations.euler_from_quaternion(ori)

    #listener=tf.TransformListener()
    #listener.waitForTransform('base_link','imu',rospy.Time(0),rospy.Duration(2.0))
    #(trans_imu,rot_imu)=listener.lookupTransform('base_link','imu', rospy.Time(0))
    #listener.waitForTransform('imu','imu_panel',rospy.Time(0),rospy.Duration(2.0))
    #(trans_panel,rot_panel)=listener.lookupTransform('imu','imu_panel', rospy.Time(0))

    #q_int = tf.transformations.quaternion_multiply(rot_imu,imu_data) ### da capire se necessaria
    #q_abs = tf.transformations.quaternion_multiply(q_int,rot_panel)

    #ee_in_imu_rotation = tf.transformations.quaternion_from_euler(pi/2,0,0)
    #ee_final_or = tf.transformations.quaternion_multiply(q_abs,ee_in_imu_rotation)
    #ee_final_or = tf.transformations.euler_from_quaternion(ee_final_or)
    if ori[2] < -pi/4 : 
        mult = pi
    elif ori[2] > 3*pi/4:
        mult = -pi
    else:
        mult = 0
    ee_final_or = tf.transformations.quaternion_from_euler(0, pi, ori[2]+mult)
    poseIni.orientation.x = ee_final_or[0]
    poseIni.orientation.y = ee_final_or[1]
    poseIni.orientation.z = ee_final_or[2]
    poseIni.orientation.w = ee_final_or[3]

    #group.set_pose_target(poseIni)
    #plan1 = group.plan()
    #rospy.sleep(1)
    #group.go(wait=True)
    #rospy.sleep(3)
    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    ### going down
    z_for_pick = mess[3] - 0.025 + 0.0625  ### considering gripper dimension
    poseIni.position.z = z_for_pick
    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    ### closing gripper on imu
    i = 0
    while i < 5:
        message_close = std_msgs.msg.String()
        message_close.data = 'semi_open'
        gripper_publisher.publish(message_close)
        rospy.sleep(0.1)
        i += 1
    rospy.sleep(5)


    ### going up
    poseIni.position.z = mess[3] + 0.1
    waypoints = []
    waypoints.append(copy.deepcopy(poseIni))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)


    

    rospy.sleep(5)
    print("End of the task 3 \n Ctrl+C to exit")
    return

    

if __name__=='__main__':
    main()