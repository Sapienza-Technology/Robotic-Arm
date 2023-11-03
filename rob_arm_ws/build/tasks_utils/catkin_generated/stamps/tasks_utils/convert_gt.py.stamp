#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos
from math import sin
import numpy as np
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
This node take the ground truth position and convert it in position w.r.t the starting position of the robot in gazebo
'''

def convert_gt(gt_odom,start_gt_pose):
    #global coordinates
    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
    gt_z=gt_odom.pose.pose.position.z

    #robot starting position
    r_x= start_gt_pose.pose.pose.position.x
    r_y= start_gt_pose.pose.pose.position.y
    r_z= start_gt_pose.pose.pose.position.z


    #covnert quaternion to euler
    q = start_gt_pose.pose.pose.orientation

    '''
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    '''

    orientation_list = [q.x, q.y, q.z, q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    

    theta= yaw
    
    '''
    #homogeneous matrix 2d
    #starting pose of robot
    pose=[
        [cos(theta), -sin(theta), r_x],
        [sin(theta), cos(theta), r_y],
        [0,0,1]
    ]
    '''
    #inverse computed with matlab
    inv_pose=[
        [cos(theta), sin(theta), -r_x*cos(theta) - r_y*sin(theta) ],
        [-sin(theta), cos(theta), r_x*sin(theta) - r_y*cos(theta)],
        [0,0,1]
    ]

    global_pos=np.transpose([gt_x,gt_y,1])
    
    #ground truth position with respect to starting position
    local_pos=np.matmul(inv_pose,global_pos)
    gt_x=local_pos[0]
    gt_y=local_pos[1]

    #print("Real position|  x:%.4f  y:%.4f  z:%.4f " % (gt_x,gt_y,gt_z))
    
    #header
    
    new_gt=Odometry()   
    new_gt.header.stamp = rospy.Time.now()
    new_gt.header.frame_id = "odom"
    new_gt.pose.pose.position.x=gt_x
    new_gt.pose.pose.position.y=gt_y
    new_gt.pose.pose.position.z=gt_z
    
    #current_gt_pose.child_frame_id = "base_footprint"

    return new_gt

#callback function, get ground_truth convert it and republish
def callback(gt_odom,start_gt_pose, pub):
    new_gt=convert_gt(gt_odom,start_gt_pose)
    pub.publish(new_gt)

def main():
    rospy.init_node('convert_gt', anonymous=True)

    #get only a sinlge message, used to set the global starting position of the robot in gazebo
    start_gt_pose = rospy.wait_for_message('/ground_truth', Odometry, timeout=20)

    #fixed ground truth, wth respect to starting position
    pub = rospy.Publisher('/ground_truth_local', Odometry, queue_size=10)

    rospy.Subscriber('/ground_truth',Odometry, lambda msg: callback(msg,start_gt_pose,pub))
 
    rospy.spin()



if __name__ == '__main__':
    main()