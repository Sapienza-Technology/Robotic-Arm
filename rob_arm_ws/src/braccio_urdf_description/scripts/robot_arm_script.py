#!/usr/bin/env python3
# license removed for brevity
from copy import copy
import rospy
import roboticstoolbox as rbt
from std_msgs.msg import String, Float64
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS, RevoluteDH
from spatialmath import SE3
from math import pi as pi
import numpy as np

def talker():
    pub0 = rospy.Publisher('/braccio_urdf/giunto0_position_controller/command', Float64, queue_size=10)
    pub1 = rospy.Publisher('/braccio_urdf/giunto1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/braccio_urdf/giunto2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/braccio_urdf/giunto3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/braccio_urdf/giunto4_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/braccio_urdf/giunto5_position_controller/command', Float64, queue_size=10)
    pubs = [pub0, pub1, pub2, pub3, pub4, pub5]
    rospy.init_node('robot_arm_script', anonymous=True)
    hz = 100
    rate = rospy.Rate(hz)
        
    robot = ERobot.URDF("/home/alessio/ROS/Robotic-Arm/rob_arm_ws/src/braccio_urdf_description/urdf/braccio_urdf_edit.xacro")
    
    # posizione non raggiungibile senza errore
    P = SE3([0.2, 0, 0.8])
    
    sol = robot.ikine_min(P)
    print(sol)
    arr = sol.q
    coeff = 0
    duration = 5

    for i in range(len(pubs)):
        rospy.loginfo(arr[i])
        pubs[i].publish(arr[i])

    '''
    while not rospy.is_shutdown() and coeff < 1:
        for i in range(6):
            rospy.loginfo(arr[i]*coeff)
            pubs[i].publish(arr[i]*coeff)
            print(coeff)
        '''
    '''
        #rospy.loginfo(position[0])
        pub0.publish(position[0]*coeff)
        #rospy.loginfo(position[1])
        pub1.publish(position[1]*coeff)
        #rospy.loginfo(position[2])
        pub2.publish(position[2]*coeff)
        #rospy.loginfo(position[3])
        pub3.publish(position[3]*coeff)
        #rospy.loginfo(position[4])
        pub4.publish(position[4]*coeff)
        #rospy.loginfo(position[5])
        pub5.publish(position[5]*coeff)
    '''

        #coeff += duration/hz
        
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

