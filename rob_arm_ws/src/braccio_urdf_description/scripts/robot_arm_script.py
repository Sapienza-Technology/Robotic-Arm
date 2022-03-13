#!/usr/bin/env python3
# license removed for brevity
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
    rospy.init_node('robot_arm_script', anonymous=True)
    rate = rospy.Rate(1)
    position = 0.0
    
    
    # valori DH calcolati a mano
    
    a = [0.0547, 0.4009, 0, 0, 0, 0]
    d = [0.051, 0, 0, 0.3419, 0, 0.1775]
    alpha = [-pi/2, 0, -pi/2, -pi/2, pi/2, -pi/2]

    robot = DHRobot([RevoluteMDH(d[0], a[0], alpha[0]),
        RevoluteMDH(d[1], a[1], alpha[1]),
        RevoluteMDH(d[2], a[2], alpha[2]),
        RevoluteMDH(d[3], a[3], alpha[3]),
        RevoluteMDH(d[4], a[4], alpha[4]),
        RevoluteMDH(d[5], a[5], alpha[5])],
    name='6R')
    
    
    #robot = ERobot.URDF("/home/alessio/ROS/rob_arm_ws_old/src/braccio_urdf_description/urdf/braccio_urdf_edit.xacro")
    #P = SE3([0.8555, -0.005, 0.04886])
    
    # posizione non raggiungibile senza errore
    P = SE3([0.5, 0.5, 0.5])
    
    sol = robot.ikine_LM(P)
    print(sol)
    arr = sol.q
    
    # plottare il robot per vedere che DH table è sbagliata 
    
    DHRobot.plot(robot, [0, 0, -pi/2, pi/2, pi/2, 0], block=True)
    '''
    while not rospy.is_shutdown():
        rospy.loginfo(arr[0])
        pub0.publish(arr[0])
        rospy.loginfo(arr[1])
        pub1.publish(arr[1])
        rospy.loginfo(arr[2])
        pub2.publish(arr[2])
        rospy.loginfo(arr[3])
        pub3.publish(arr[3])
        rospy.loginfo(arr[4])
        pub4.publish(arr[4])
        rospy.loginfo(arr[5])
        pub5.publish(arr[5])
        
        rate.sleep()
    '''

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

