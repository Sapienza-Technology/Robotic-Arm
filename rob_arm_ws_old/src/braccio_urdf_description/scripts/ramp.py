#!/usr/bin/env python3
# license removed for brevity
import rospy
import roboticstoolbox as rbt
from std_msgs.msg import String, Float64
from roboticstoolbox import DHRobot, RevoluteMDH, ERobot, ELink, ETS
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
    rospy.init_node('ramp', anonymous=True)
    rate = rospy.Rate(100)
    initial_position = 0.0
    position = initial_position
    final_position = 1.6
    time = 5.0
    
    delta = final_position/(time*100)    

    while (not rospy.is_shutdown() and position < final_position):
        rospy.loginfo(0)
        pub0.publish(0)      
        rospy.loginfo(position)
        pub1.publish(position)
        pub2.publish(0)
                
        position += delta

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

