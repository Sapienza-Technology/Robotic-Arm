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
import numpy as np
from tag_utils.srv import GetTag
from math import atan, pi, exp, cos, sin, tan
from std_msgs.msg import String, Float32
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
global buttons_clicked
global current_tag

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
        poseIni_int.position.z = 0.240
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
        euler=(0.0, pi, 0.0)
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


def move_around_buttons(request_tag, group, scene):
    initPose = group.get_current_pose().pose
    poseNear = group.get_current_pose().pose
    msg_request=Float32()
    msg_request.data = 1
    request_tag.publish(msg_request)
    mess = wait_for_message_custom('/obj2_status', std_msgs.msg.Float32MultiArray, timeout=3)
    #mess = mess.data
    step = 0
    async_step = 0
    variation = [[0.01,0],[0,0.01],[-0.01,0],[0,-0.01]]
    counter = 0

    while mess[0] == -1 and counter < 4:
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

        request_tag.publish(msg_request)
        mess = wait_for_message_custom('/obj2_status', std_msgs.msg.Float32MultiArray, timeout=3)
        #mess = mess.data

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
    
    return mess


def buttons_to_see(data, group, scene):
    global x_dist
    global left_ob_pos
    global center_pose

    tag_id = data[0]
    tag_pos = data[1:]
    print(tag_id)


    #### Mi muovo avanti al bottone visto e memorizzo x_dist
    if tag_id > 0:
        posTag = geometry_msgs.msg.Pose()

        x_dist = tag_pos[0]
        left_ob_pos = x_dist - 0.190*sin(pi/10)

        posTag.position.z = tag_pos[2] - 0.055
        posTag.position.y = tag_pos[1]*0.8 + group.get_current_pose().pose.position.y*0.2
        posTag.position.x = tag_pos[0] - 0.0186 - 0.0485 - 0.015 - 0.05
        euler=(0.0, pi/2, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        posTag.orientation.x = quaternion[0]
        posTag.orientation.y = quaternion[1]
        posTag.orientation.z = quaternion[2]
        posTag.orientation.w = quaternion[3]

        add_panel(scene, x_dist)

        waypoints = []
        waypoints.append(copy.deepcopy(posTag))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)
    
    if tag_id < -1:
        x_dist = tag_pos[0]
        left_ob_pos = x_dist - 0.190*sin(pi/10)

    #### Mi sposto a destra del bottone
    posTag = group.get_current_pose().pose
    shift_y = -0.095

    if posTag.position.y + shift_y < -0.1:           #0.14
        posTag.position.y = 0.095
        if posTag.position.z - 0.15 < 0.03:
            waypoints = []
            waypoints.append(copy.deepcopy(center_pose))
            waypoints.append(copy.deepcopy(home_pose))
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(3)
        else:
            waypoints = []
            waypoints.append(copy.deepcopy(posTag))
            posTag.position.z = posTag.position.z - 0.15
            waypoints.append(copy.deepcopy(posTag))
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(3)
            posTag = home_pose
    else:
        posTag.position.y = posTag.position.y + shift_y
        waypoints = []
        waypoints.append(copy.deepcopy(posTag))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)


def align_to_button(data, group, scene):
    global center_pose

    tag_id = data[0]
    tag_pos = data[1:]
    print(tag_id)


    #### Mi muovo avanti al bottone visto e memorizzo x_dist
    if tag_id > 0:
        posCurr = group.get_current_pose().pose
        posCurr.position.y = 0.095

        posInt = group.get_current_pose().pose
        posInt.position.y = 0.095
        posInt.position.z = tag_pos[2] - 0.055

        posTag = geometry_msgs.msg.Pose()
        posTag.position.z = tag_pos[2] - 0.055
        posTag.position.y = tag_pos[1]*0.8 + group.get_current_pose().pose.position.y*0.2
        posTag.position.x = tag_pos[0] - 0.0186 - 0.0485 - 0.015 - 0.05
        euler=(0.0, pi/2, 0.0)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        posTag.orientation.x = quaternion[0]
        posTag.orientation.y = quaternion[1]
        posTag.orientation.z = quaternion[2]
        posTag.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(copy.deepcopy(posCurr))
        waypoints.append(copy.deepcopy(posInt))
        waypoints.append(copy.deepcopy(posTag))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(5)


def click_button(data, group, scene, click_pub):
    global buttons_clicked
    global current_tag

    ### to be safe to tag number different from button number
    mat_buttons = ['/button1','/button2','/button3','/button4','/button5','/button6','/button7','/button8','/button9']
    button = mat_buttons[int(data[0])-1]

    current_pose = group.get_current_pose().pose
    clicked = False
    step = 1
    while not clicked and step < 6:
        current_pose.position.x = current_pose.position.x + 0.010*exp(-0.15*step)
        waypoints = []
        waypoints.append(copy.deepcopy(current_pose))
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.001, 0.0)
        group.execute(plan, wait=True)
        rospy.sleep(3)
        step += 1
        try:
            clicked = rospy.wait_for_message(button, std_msgs.msg.Bool, timeout=3)
            clicked = clicked.data
        except rospy.exceptions.ROSException:
            clicked = False
        if clicked or step == 5:            ### togli per la competizione
            msg = Float32()
            msg.data = data[0]
            click_pub.publish(msg)
    
    #TO DO
    ##################################################### ATTENTO RITARDATO
    clicked = True ###################################### PROVVISORIO
    ##################################################### ELIMINALO
    if clicked and current_tag < len(buttons_clicked):
        buttons_clicked[current_tag] = 1
        current_tag += 1
    
    
def wait_for_message_custom(topic, msg_type, timeout=3):
    mess = None
    try:
        mess = rospy.wait_for_message(topic, msg_type, timeout=timeout)
        mess = mess.data
    except rospy.exceptions.ROSException:
        mess = [-1,0,0,0]
    return mess




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
    global buttons_clicked
    global current_tag
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
                        '/obj2_get_tag',
                        std_msgs.msg.Float32,
                        queue_size=1)
    
    skip_tag    = rospy.Publisher(
                        '/obj2_skip_tag',
                        std_msgs.msg.Float32,
                        queue_size=1)
    
    click_tag  = rospy.Publisher(
                        '/obj2_click_tag',
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
    possible_tags = np.zeros((9,3))

    objective=sys.argv[1]
    tags_to_consider=[]
    if objective:
        #objective contains tags to consider in form "1,2,3,4"
        tags_to_consider=[int(objective)]
    buttons_clicked = np.zeros(len(tags_to_consider))
    current_tag = 0


    #rospy.wait_for_service('get_tag')
    #try:
    #    counter = 0
    #    for id in range(9):
    #        get_tag = rospy.ServiceProxy('get_tag', GetTag)
    #        response = get_tag(id+1) #list of 3 elements (x,y,z)
    #        possible_tags[id,:] = np.array(response)
    #        x_dist = (x_dist*counter + response[0])/(counter+1)
    #    print(response)
    #except rospy.ServiceException as e:
    #    print(f"Service call failed: {e}")

    ### Chiusura gripper
    i = 0
    while i < 5:
        message_close = std_msgs.msg.String()
        message_close.data = 'close'
        gripper_publisher.publish(message_close)
        rospy.sleep(0.1)
        i += 1
    rospy.sleep(3)


    ### Inizializzazione posizione iniziale per vedere primo tag
    pose_goal = geometry_msgs.msg.Pose()
    euler=(0.0, pi/2, 0.0)
    quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    print(quaternion)
    pose_goal.position.x = 0.32
    pose_goal.position.y = home_pose.position.y
    pose_goal.position.z = home_pose.position.z
    group.set_pose_target(pose_goal)

    
    waypoints = []
    waypoints.append(copy.deepcopy(pose_goal))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    pose_goal.position.y = 0.095
    pose_goal.position.z = 0.410
    group.set_pose_target(pose_goal)

    waypoints = []
    waypoints.append(copy.deepcopy(pose_goal))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)


    msg_request=Float32()
    msg_request.data = 1

    ### Approccio fino a vedere il primo tag


    first_not_seen = True
    while first_not_seen:
        print("Approaching")
        request_tag.publish(msg_request)
        mess = wait_for_message_custom('/obj2_status', std_msgs.msg.Float32MultiArray, timeout=3)
        
        if mess[0] < 0 and group.get_current_pose().pose.position.x > 0.55:
            ### Se non vede niente provo a sinistra, altrimenti torno indietro
            pose_goal.position.x -= 0.10
            group.set_pose_target(pose_goal)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            rospy.sleep(3)

            x_dist = 0.60
            left_ob_pos = x_dist - 0.190*sin(pi/10)
            go_to_see_left(group, scene, 1)

            print("Try to see 11")
            request_tag.publish(msg_request)
            mess = wait_for_message_custom('/obj2_status', std_msgs.msg.Float32MultiArray, timeout=3)
            #mess = mess.data
            if mess[0]>0:
                x_dist = mess[1]+190*sin(pi/10)
                left_ob_pos = mess[1]
                pose_goal.position.x = x_dist + 0.05
                first_not_seen = False
            waypoints = []
            waypoints.append(copy.deepcopy(pose_goal))
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(3)
            first_not_seen = False
        elif mess[0] < 0 or (mess[0] > 0 and (mess[3] > 0.495 or mess[3] < 0.4)):                       ### Già mi arriva solo il tag 1
            pose_goal.position.x += 0.01
            #waypoints = []
            #waypoints.append(copy.deepcopy(pose_goal))
            #(plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
            #group.execute(plan, wait=True)
            #rospy.sleep(3)
            group.set_pose_target(pose_goal)
            plan1 = group.plan()
            rospy.sleep(1)
            group.go(wait=True)
            #group.clear_pose_targets()
            rospy.sleep(3)
        else:
            posTag = geometry_msgs.msg.Pose()
            x_dist = mess[1]
            posTag.position.z = mess[3] - 0.055
            posTag.position.y = mess[2]*0.8 + group.get_current_pose().pose.position.y*0.2
            posTag.position.x = mess[1] - 0.0186 - 0.0485 - 0.015 - 0.05
            euler=(0.0, pi/2, 0.0)
            quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
            posTag.orientation.x = quaternion[0]
            posTag.orientation.y = quaternion[1]
            posTag.orientation.z = quaternion[2]
            posTag.orientation.w = quaternion[3]

            add_panel(scene, x_dist)

            waypoints = []
            waypoints.append(copy.deepcopy(posTag))
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
            group.execute(plan, wait=True)
            rospy.sleep(3)
            first_not_seen = False
            skip_tag.publish(msg_request)


    ### Inizializzazione poszione home
    home_pose  = group.get_current_pose().pose
    center_pose = group.get_current_pose().pose
    center_pose.position.y = 0
    center_pose.position.z = 0.2


    
    ### Schiacciamo i bottoni
    phase = 1
    while phase == 1:
        if current_tag >= len(tags_to_consider):
            phase = 2
            break
        if possible_tags[tags_to_consider[current_tag]-1,0] != 0:
            mess = [tags_to_consider[current_tag], possible_tags[tags_to_consider[current_tag]-1,0], possible_tags[tags_to_consider[current_tag]-1,1], possible_tags[tags_to_consider[current_tag]-1,2]]
            print(mess)
            align_to_button(mess, group, scene)
        else:
            request_tag.publish(msg_request)
            mess = wait_for_message_custom('/obj2_status', std_msgs.msg.Float32MultiArray, timeout=3)
            #print(mess)
            #mess = mess.data
            rospy.loginfo(mess)
        if mess[0] > 100:
            phase = 2
        elif mess[0] < -1:
            possible_tags[int(-mess[0]-1)-1,:] = mess[1:4]
            buttons_to_see(mess, group, scene)
            rospy.sleep(1)
        elif mess[0] <  0:
            mess = move_around_buttons(request_tag, group, scene)
            if mess[0] > 0 and mess[0] < 10:
                click_button(mess, group, scene, click_tag)
            buttons_to_see(mess, group, scene)
            rospy.sleep(1)
        elif mess[0] < 10:
            click_button(mess, group, scene, click_tag)
            buttons_to_see(mess, group, scene)
            rospy.sleep(1)

    center_pose = group.get_current_pose().pose
    center_pose.position.y = 0
    center_pose.position.z = 0.2
    waypoints = []
    waypoints.append(copy.deepcopy(center_pose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
    group.execute(plan, wait=True)
    rospy.sleep(3)

    #group.go(home_joint, wait=True)
    #rospy.sleep(3)

    rospy.sleep(3)
    print("End of the task 9 \n Ctrl+C to exit")
    return

    

if __name__=='__main__':
    main()