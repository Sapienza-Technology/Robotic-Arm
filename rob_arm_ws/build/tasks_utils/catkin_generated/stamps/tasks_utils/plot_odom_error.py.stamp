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
from matplotlib import pyplot as plt



fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('time (s)')
ax.set_ylabel('error (m)')
#max y scale
ax.set_ylim(0, 1.25)


'''
This node take the estimated position and the ground truth position and compute the error between them and plot it
It's important to notice that the ground truth position initially is with respect to the global frame of gazebo, 
we need it to be with respect to the robot starting position
'''

def measure_error(est_odom,gt_odom,params):
    
    start_time=params["start_time"]
    params["counter_received"]+=1

    #estimated position
    x=est_odom.pose.pose.position.x
    y=est_odom.pose.pose.position.y
    z=est_odom.pose.pose.position.z
    #print("estimated position|  x:%.4f  y:%.4f  z:%.4f" % (x,y,z))

    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
 
    #euclidean distance between ground truth position and estimated position
    error=sqrt((gt_x-x)**2+(gt_y-y)**2)
    print(params["topic"],"error: %.4f" % error)

    #check if is an outlier by comparing the error with the previous one
    if len(params["errors"])>0 and error>0.1 and error>params["errors"][-1]*3:
        print("Outlier detected")
        #plot a blue circle with a certain size
        ax.plot((rospy.Time.now()-start_time).to_sec(),error,'bo',markersize=10)
        return
        
    #if error graeter than limit change limit
    if error>ax.get_ylim()[1]:
        ax.set_ylim(0, error*1.2)
    

    #plot the error
    params["errors"].append(error)
    params["times"].append((rospy.Time.now()-start_time).to_sec())
    #plot every x messages
    if  params["counter_received"]%20==0:
        ax.plot(params["times"],params["errors"],'r')
        #plot figure if file does not exist, create it
        fig.savefig(params["saving_path"])
    print()
    


def check_odom_error():
    print("Starting odom error node")
    rospy.init_node('check_odom_error_plot', anonymous=True)

    start_time=rospy.Time.now()

    odom_topic=rospy.get_param('~odom_topic')
    params={
        "start_time":start_time,
        "errors":[],
        "times":[],
        "counter_received":0,
        "saving_path": rospy.get_param("~saving_path", "figures/odom_error.png"),
        "topic": odom_topic
    }
    plot_title=rospy.get_param('~plot_title', "odom error")
    ax.set_title(plot_title)
    print("getting odom from topic:",odom_topic)

    est_odom = message_filters.Subscriber(odom_topic, Odometry)

    gt_odom_topic="/ground_truth_local"
    try:
        gt_odom_topic=rospy.get_param('~gt_odom_topic')
    except:
        print("No ground truth topic specified, using /ground_truth_local")
    gt_odom = message_filters.Subscriber(gt_odom_topic, Odometry)

    queue_size=10
    ts = message_filters.ApproximateTimeSynchronizer([est_odom, gt_odom],queue_size,0.1)
    ts.registerCallback(lambda est_odom, gt_odom: measure_error(est_odom,gt_odom,params))


    rospy.spin()



if __name__ == '__main__':
    check_odom_error()