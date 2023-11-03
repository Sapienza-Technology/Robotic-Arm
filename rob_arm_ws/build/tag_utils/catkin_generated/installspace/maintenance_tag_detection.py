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
from std_msgs.msg import Float32
#import pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
#import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
#from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import time

from tag_utils.tagDetector import Tag, TagDetector

def set_target_tag_cb(msg, params):
    if int(msg.data) >= len(params["tags_to_consider"]):
        print("End of the task")
        params["current_tag"]=999
        return
    next_tag=params["tags_to_consider"][int(msg.data)]
    params["current_tag"]=next_tag
    print("\nNOW TARGETING TAG {}\n".format(next_tag))


def callback(data,params,group):
    goal_tag=[params["current_tag"]]
    tags_to_consider=params["tags_to_consider"]

    if goal_tag[0] > 100:
        end_tag = Marker()
        end_tag.id = 999
        params["tag_pub"].publish(end_tag)
        time.sleep(5)
        rospy.signal_shutdown("End of the task")
        return


    last_received=params["last_received"]
    if last_received is not None:
        time_receiving=time.time()-last_received
        rate=1/time_receiving
        if params["verbosity"]:
            #print(f"Received image at {rate} Hz")
            pass
    params["last_received"]=time.time()

    tagDetector=params["tagDetector"]
    tag_time=time.time()
    tags=tagDetector.detect_from_ros_image(data)
    if len(tags)>0 and params["verbosity"]:
        #print(f"Found {len(tags)} tags in {time.time()-tag_time} seconds")
        pass

    processing_time=time.time()
    published_tag = False
    for tag in tags:
        if len(tags_to_consider)>0 and tag.id not in goal_tag:
            continue
        #transform_time=time.time()
        tag.transform_tag_frame(params["base_frame"],q=group.get_current_joint_values())
        #print(f"\t transformed in {time.time()-transform_time} seconds")
        params["tag_pub"].publish(tag.get_marker_msg())
        published_tag = True
    if not published_tag:
        #publish empty marker
        params["tag_pub"].publish(Marker())

    if len(tags)>0 and params["verbosity"]:
        #print(f"Processed {len(tags)} tags in {time.time()-processing_time} seconds")
        pass


def main():
    rospy.init_node('remote_tag_detection', anonymous=True)

    objective=rospy.get_param("~objective")
    tags_to_consider=[]
    if objective:
        #objective contains tags to consider in form "1,2,3,4"
        tags_to_consider=[int(tag) for tag in objective.split(",")]
    
    
    verbosity=rospy.get_param("~verbose",0)
    marker_size=rospy.get_param("~tag_size",0.05)
    tagDetector=TagDetector(marker_size,verbose=verbosity)
    #subscriber
    camera_info_topic="camera_info_topic"
    image_topic="img_topic"
    status_topic="status_topic"
    
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
    tag_pub=rospy.Publisher("tag_pose",Marker, queue_size=10)

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)


    params= {
        "base_frame":  rospy.get_param("~target_frame","base_link"),
        "tag_pub": tag_pub,
        "tagDetector": tagDetector,
        "last_received": None,
        "tags_to_consider": tags_to_consider, #if empty, consider all tags
        "current_tag": tags_to_consider[0],# if len(tags_to_consider)>0 else None,
        "verbosity": verbosity
    }

    rospy.Subscriber(status_topic, Float32, lambda msg: set_target_tag_cb(msg, params))
    rospy.Subscriber(image_topic, Image, lambda msg: callback(msg, params, group))
    rospy.spin()


if __name__=='__main__':
    main()
