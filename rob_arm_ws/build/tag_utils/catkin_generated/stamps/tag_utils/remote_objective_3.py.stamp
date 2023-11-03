#ROS node that subscribes to the image topic, detects the arTag, and publishes the tag's pose with respect to the base_link frame

import rospy
import cv2
import numpy as np
#import math
#import tf
#import tf2_ros
#import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
#ros float32
from std_msgs.msg import Float32,Float32MultiArray,Bool
#import pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
#import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
#from cv_bridge import CvBridge, CvBridgeError
import os
import json


try:
    import moveit_commander
except:
    moveit_commander=None
    print("moveit not installed, using ros transform to compute tag position")
import time

from tag_utils.tagDetector import Tag, TagDetector



def callback(x,params,group):
    print("\n")   

    tagDetector=params["tagDetector"]
    tag_time=time.time()
    #get an image
    print("getting an image...", end="")
    data=rospy.wait_for_message(params["img_topic"], Image, timeout=rospy.Duration(1))
    print("Done!")

    tags=tagDetector.detect_from_ros_image(data)
    if len(tags)>0 and params["verbosity"]:
        print(f"Found {len(tags)} tags in {time.time()-tag_time} seconds")
        print("tags found:",[tag.id for tag in tags])
        pass
    else:
        print("couldn't find any tag")

    #processing_time=time.time()
    #published_tag = False
    msg = Marker()
    msg.id=-1
    for tag in tags:

        if tag.id not in params["tags_to_consider"]:
            print("found a tag, but not the one I am looking for")
            continue

        print("found the tag I am looking for! it is:",tag.id)
        status=tag.id
        #transform_time=time.time()
        q=None
        if group is not None:
            q=group.get_current_joint_values()
        tag.transform_tag_frame(params["base_frame"],q=q)
        msg = tag.get_marker_msg()
        #tag_position=tag.T
        #Store value for tag

    #create a 4 number array and publish it
    params["tag_pub"].publish(msg)



def main():
    rospy.init_node('remote_tag_detection', anonymous=True)
    verbosity=rospy.get_param("~verbose",0)
    marker_size=rospy.get_param("~tag_size",0.05)
    tagDetector=TagDetector(marker_size,verbose=verbosity)
    #subscriber
    camera_info_topic="camera_info_topic"
    image_topic="img_topic"
    
    print("Waiting for camera_info topic...")
    #get one message from camera_info topic to get camera matrix
    camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=rospy.Duration(10))
    tagDetector.set_camera_from_camera_info(camera_info)
    print("Got camera_info topic")
    detectorParams = cv2.aruco.DetectorParameters_create()
    detectorParams.markerBorderBits = rospy.get_param("border_thickness",1)
    #detectorParams.perspectiveRemoveIgnoredMarginPerCell=0.4

    tagDetector.set_detector_params(detectorParams)
    tagDetector.set_tag_size(0.04)
    print("params set")


    tag_pub=rospy.Publisher("obj3_status",Marker, queue_size=1)

    group_name = "manipulator"
    group=None
    if moveit_commander:
        group = moveit_commander.MoveGroupCommander(group_name)



    params= {
        "base_frame":  rospy.get_param("~target_frame","base_link"),
        "tag_pub": tag_pub,
        "img_topic": image_topic,
        "tagDetector": tagDetector,
        "tags_to_consider": [10],
        "verbosity": verbosity
    }

    rospy.Subscriber("/obj3_get_tag", Float32, lambda msg: callback(msg, params, group))

    rospy.spin()


if __name__=='__main__':
    main()
