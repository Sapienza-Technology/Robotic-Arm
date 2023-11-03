#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos
from math import sin
import numpy as np
from math import sqrt
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
This node take the estimated position and the ground truth position and compute the error between them
It's important to notice that the ground truth position initially is with respect to the global frame of gazebo, 
we need it to be with respect to the robot starting position
'''

def measure_error(est_odom,gt_odom):
    
    #estimated position
    x=est_odom.pose.pose.position.x
    y=est_odom.pose.pose.position.y
    z=est_odom.pose.pose.position.z
    print("estimated position|  x:%.4f  y:%.4f  z:%.4f" % (x,y,z))

    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
 
    #euclidean distance between ground truth position and estimated position
    error=sqrt((gt_x-x)**2+(gt_y-y)**2)
    print("error: %.4f" % error)
    print()
    


def check_odom_error():
    print("Starting odom error node")
    rospy.init_node('check_odom', anonymous=True)

    odom_topic=rospy.get_param('~odom_topic')
    print("getting odom from topic:",odom_topic)

    est_odom = message_filters.Subscriber(odom_topic, Odometry)
    gt_odom = message_filters.Subscriber('/ground_truth', Odometry)

    queue_size=10
    ts = message_filters.ApproximateTimeSynchronizer([est_odom, gt_odom],queue_size,0.1)
    ts.registerCallback(lambda est_odom, gt_odom: measure_error(est_odom,gt_odom))


    rospy.spin()



if __name__ == '__main__':
    check_odom_error()