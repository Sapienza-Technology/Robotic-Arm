#ROS node that subscribes to the image topic, detects the arTag, and publishes the tag's pose with respect to the base_link frame

import rospy
import cv2
import numpy as np
import sys
#import math
#import tf
#import tf2_ros
#import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
#ros float32
from std_msgs.msg import Float32, Float32MultiArray
#import pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
#import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
#from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import time

from tag_utils.tagDetector import Tag, TagDetector


def click_tag(x,params):
    id_click = int(x.data)
    params["clicked_tag"][id_click-1] = 1
    params["current_tag"] += 1
    print(f"Clicked tag {id_click}")

def skip_tag(x,params):
    #if a message is received on this topic, current tag could not be detected, so it is skipped
    params["phase_0_end"] = True


def callback(data,params,group):
    clicked_tag=params["clicked_tag"] ####
    tags_to_consider=params["tags_to_consider"]

    if sum(clicked_tag) == 4:
        end_tag=Float32MultiArray()
        end_tag.data=[999,0,0,0]
        params["tag_pub"].publish(end_tag)
        time.sleep(5)
        rospy.signal_shutdown("End of the task")
        return
    
    current_tag=[tags_to_consider[params["current_tag"]]]

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

    processing_time=time.time()
    status=-1
    tag_position=[0,0,0]
    for tag in tags:
        if not params["phase_0_end"] and tag.id != 1:
            continue
        if params["phase_0_end"] and len(tags_to_consider)>0 and (tag.id not in current_tag or clicked_tag[int(tag.id)-1] == 1):
            status = -1 - tag.id
        else:
            #transform_time=time.time()
            status=tag.id
        q=None
        if group is not None:
            q=group.get_current_joint_values()
        tag.transform_tag_frame(params["base_frame"],q=q)
        tag_position=tag.T
        #print(f"\t transformed in {time.time()-transform_time} seconds")

    msg=Float32MultiArray()
    msg.data=[status,tag_position[0],tag_position[1],tag_position[2]]
    params["tag_pub"].publish(msg)

    if len(tags)>0 and params["verbosity"]:
        #print(f"Processed {len(tags)} tags in {time.time()-processing_time} seconds")
        pass


def main():
    rospy.init_node('remote_tag_detection', anonymous=True)

    objective=sys.argv[2]
    tags_to_consider=[]
    if objective:
        #objective contains tags to consider in form "1,2,3,4"
        tags_to_consider=[objective]

    clicked_tag=np.zeros(9)
    
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
    print("params set")
    #publisher of pose stamped message
    tag_pub=rospy.Publisher("/obj2_status",Float32MultiArray, queue_size=2)

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)


    params= {
        "base_frame":  rospy.get_param("~target_frame","base_link"),
        "tag_pub": tag_pub,
        "img_topic": image_topic,
        "tagDetector": tagDetector,
        "last_received": None,
        "tags_to_consider": tags_to_consider, #if empty, consider all tags
        "clicked_tag": clicked_tag,
        "current_tag": 0,
        "phase_0_end": False,
        "verbosity": verbosity
    }

    rospy.Subscriber("/obj2_get_tag", Float32, lambda msg: callback(msg, params, group))
    rospy.Subscriber("/obj2_skip_tag", Float32, lambda msg: skip_tag(msg, params))
    rospy.Subscriber("/obj2_click_tag", Float32, lambda msg: click_tag(msg, params))
    rospy.spin()


if __name__=='__main__':
    main()
