#!/usr/bin/env python
import rospy
import numpy as np

from matplotlib import pyplot as plt
from rtabmap_ros.msg import OdomInfo


fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('time (s)')
ax.set_ylabel('Quality')
#max y scale
#ax.set_ylim(0, 1000)
ax.set_title("odom quality")

""" fig2 = plt.figure()
ax2 = fig.add_subplot(111)
ax2.set_xlabel('time (s)')
ax2.set_ylabel('Quality')
ax2.set_title("odom quality")

 """
#explain the node
'''
This node subscribes to the odom info topic and plots the number of inliers
'''

def plotQuality(data,params):
    
    start_time=params["start_time"]
    params["counter_received"]+=1
    #plot one point every other 2
    if  params["counter_received"]%3!=0: return

    #get number of inliers
    inliers=data.inliers
    lost=data.lost
    #plot the error
    params["qualities"].append(inliers)
    params["times"].append((rospy.Time.now()-start_time).to_sec())
    #plot every 50 messages
    if  lost or params["counter_received"]%20==0:
        ax.plot(params["times"],params["qualities"],'b')
        #plot figure if file does not exist, create it
        fig.savefig(params["saving_path"])

    print()
    


def plot_odom_quality():
    print("Starting odom quality node")
    rospy.init_node('plot_odom_quality', anonymous=True)

    start_time=rospy.Time.now()

    params={
        "start_time":start_time,
        "qualities":[],
        "times":[],
        "counter_received":0,
        "saving_path": rospy.get_param("~saving_path", "figures/odom_quality.png")
    }

    #subscribe to odom info to get the number of inliers
    odom_info_topic="/odom_info_lite"
    odom_info = rospy.Subscriber(odom_info_topic, OdomInfo, lambda x: plotQuality(x,params))
    rospy.spin()

if __name__ == '__main__':
    plot_odom_quality()