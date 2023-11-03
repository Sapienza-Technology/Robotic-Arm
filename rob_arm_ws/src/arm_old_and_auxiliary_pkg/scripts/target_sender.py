#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from braccio_urdf_description.msg import Float6Array

def callback_ack(ack):
    rospy.loginfo(f"ack = {ack}")

if __name__ == '__main__':
    rospy.init_node('target_sender', anonymous=True)

    pub_target = rospy.Publisher('/braccio_urdf/q_target', Float6Array, queue_size=10)
    
    rospy.Subscriber('/braccio_urdf/q_ack', Float64, callback_ack)

    i = ""
    while input != "q":
        i = float(input())
        msg = Float6Array()
        msg.x1 = i
        msg.x2 = i
        msg.x3 = i
        msg.x4 = i
        msg.x5 = i
        msg.x6 = i

        pub_target.publish(msg)