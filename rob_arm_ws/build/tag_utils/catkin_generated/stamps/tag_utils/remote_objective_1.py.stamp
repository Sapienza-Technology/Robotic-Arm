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


def save_tag_to_file(tag_pos):
    #save the tag positions to a file
    #tag_pos is a list of 15 elements, each element is a list of 3 elements (x,y,z)
    #get the path of the file
    path=os.path.dirname(os.path.realpath(__file__))
    path=os.path.dirname(path)
    path=os.path.dirname(path)
    path=path+"/files/tag_pos.json"
    print("saving tag position to file: ",path)
    #convert the list to a dictionary
    tag_pos_dict={}
    for i in range(1,15):
        tag_pos_dict[i]=tag_pos[i].tolist()
    #save the dictionary to a file
    with open(path, 'w') as fp:
        json.dump(tag_pos_dict, fp)


def call_aruco_checker(tag_pos):
    print("saving positions to file")
    #save to file
    save_tag_to_file(tag_pos)
    print("calling score checker service")
    try:
        from erc_aruco_msg.srv import ErcArucoRequest, ErcArucoResponse, ErcAruco
    except ModuleNotFoundError as e:
        print("WARNING: erc checker service not found!")
        return

    
    print("waiting for score service...  ", end="")
    rospy.wait_for_service("erc_aruco_score")
    print("found!")
    try:
        # create service proxy with service name and message type
        service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
        # create object of the request type for the Service (14 tags)
        service_msg = ErcArucoRequest()

        # #######################################################################
        
        service_msg.tag1=tag_pos[1]
        service_msg.tag2=tag_pos[2]
        service_msg.tag3=tag_pos[3]
        service_msg.tag4=tag_pos[4]
        service_msg.tag5=tag_pos[5]
        service_msg.tag6=tag_pos[6]
        service_msg.tag7=tag_pos[7]
        service_msg.tag8=tag_pos[8]
        service_msg.tag9=tag_pos[9]
        service_msg.tag10=tag_pos[10]
        service_msg.tag11=tag_pos[11]
        service_msg.tag12=tag_pos[12]
        service_msg.tag13=tag_pos[13]
        service_msg.tag14=tag_pos[14]
        

        ##################################################################################

        # call the service with your message through service proxy
        # and receive the response, which happens to be your score
        service_response = service_proxy(service_msg)
        print(f"You received score {service_response.score}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    

def phase_update(params):
    current_phase=params["current_phase"]
    tags_in_phase=params["tags_phases"].get(current_phase)
    if tags_in_phase is None:
        print("all tags detected")
        return
    current_tag_in_phase=params["current_tag_in_phase"]
    if ( (current_phase==1 and params["current_tag_in_phase"]>=len(tags_in_phase)- params["skipped_buttons"]) or
        (current_phase!=1 and params["current_tag_in_phase"]>=len(tags_in_phase)) ):
        print("FINISHED PHASE: ",params["current_phase"])
        params["current_phase"]=current_phase+1
        status=(params["current_phase"]+1)*100 #phase 2 is 200, phase 3 is 300
        params["current_tag_in_phase"]=0
        #if all tags have been detected convert to ERC format and send to checker
        if params["tags_phases"].get(params["current_phase"]) is None:
            print("all tags detected")
            params["current_phase"]=999
            call_aruco_checker(params["tags_pos"])
            status=999

def skip_tag(x,params):
    #if a message is received on this topic, current tag could not be detected, so it is skipped
    current_phase=params["current_phase"]
    tags_in_phase=params["tags_phases"].get(current_phase)
    if tags_in_phase is None:
        print("all tags detected, why reset a tag?")
        return
    if current_phase==1:
        print("Skipping a button, for a total of {} buttons skipped".format(params["skipped_buttons"]+1))
    else:
        print(f"Skipping tag {tags_in_phase[int(params['current_tag_in_phase'])]}")
        print(f"it is the {params['current_tag_in_phase']}° tag in phase {current_phase}") 

    if current_phase!=1:
        params["current_tag_in_phase"]=params["current_tag_in_phase"]+1
    else:
        params["skipped_buttons"]=params["skipped_buttons"]+1

    phase_update(params)


def callback(x,params,group):
    print("\n")
    current_phase=params["current_phase"]
    tags_in_phase=params["tags_phases"].get(current_phase)
    if tags_in_phase is None:
        print("all tags detected")
        return
    current_tag_in_phase=params["current_tag_in_phase"]
    if current_phase!=1:
        print(f"Currently considering {int(current_tag_in_phase)}° tag of phase {current_phase}: \
            {tags_in_phase[int(current_tag_in_phase)]}")
    else:
        print("PHASE 1: currently found buttons:",params["buttons_found"])
        buttons_to_found=len(tags_in_phase)-sum(params["buttons_found"]) - params["skipped_buttons"]
        print("I need to find: {} more buttons".format(int(buttons_to_found)))

    tag_goal=None #not used in phase 1
    if current_phase != 1:
        tag_goal=tags_in_phase[int(current_tag_in_phase)]
        print("I am looking for tag:",tag_goal)
    else:
        print("I am looking for tags:",tags_in_phase)

    

    tagDetector=params["tagDetector"]
    tag_time=time.time()
    #get an image
    print("getting an image...", end="")
    data=rospy.wait_for_message(params["img_topic"], Image, timeout=rospy.Duration(1))
    print("Done!")

    #if tag that should be detected is a smaller tag, update the parameters of the tagdetector
    if tag_goal and tag_goal in params["tag_4cm"]:
        tagDetector.set_tag_size(0.04)
    else:
        tagDetector.set_tag_size(0.05)

    tags=tagDetector.detect_from_ros_image(data)
    if len(tags)>0 and params["verbosity"]:
        print(f"Found {len(tags)} tags in {time.time()-tag_time} seconds")
        print("tags found:",[tag.id for tag in tags])
        pass
    else:
        print("couldn't find any tag")

    #processing_time=time.time()
    #published_tag = False
    status=-1
    tag_position=[0,0,0]
    for tag in tags:
        #should i consider this tag?
        if ((current_phase==1 and tag.id not in tags_in_phase) or
            (current_phase!=1 and tag.id != tag_goal) ):
            print("found tag:", tag.id, "continue...")
            continue
        #was this button already found?
        #if (current_phase==1 and params["buttons_found"][tag.id-1]==1):
        #    continue

        print("found the tag I am looking for! it is:",tag.id)
        
        status=tag.id
        #transform_time=time.time()
        q=None
        if group is not None:
            q=group.get_current_joint_values()
        tag.transform_tag_frame(params["base_frame"],q=q)
        tag_position=tag.T
        #Store value for tag
        params["tags_pos"][tag.id]=tag_position.reshape(3)

        #UPDATE
        if current_phase==1:
            params["buttons_found"][tag.id-1]=1
            params["current_tag_in_phase"]=sum(params["buttons_found"])
        else:
            params["current_tag_in_phase"]=params["current_tag_in_phase"]+1

        phase_update(params)

    #create a 4 number array and publish it
    msg=Float32MultiArray()
    msg.data=[status,tag_position[0],tag_position[1],tag_position[2],params["current_phase"]]
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
    print("params set")


    tag_pub=rospy.Publisher("obj1_status",Float32MultiArray, queue_size=1)

    group_name = "manipulator"
    group=None
    if moveit_commander:
        group = moveit_commander.MoveGroupCommander(group_name)

    #15 tag with x,y,z
    tags_pos=np.zeros((15,3))
    #for each phase consider only some tags
    tags_phases={
        1: [1,2,3,4,5,6,7,8,9], #buttons
        2: [11,10],             #imu 
        3: [14,12,13]           #box
    }

    #id of tags with marker of size 4cm isntead of 5
    tag_4cm=[10,13]

    buttons_found=np.zeros(9)

    params= {
        "base_frame":  rospy.get_param("~target_frame","base_link"),
        "tag_pub": tag_pub,
        "img_topic": image_topic,
        "tagDetector": tagDetector,
        "tags_pos": tags_pos,
        "tags_phases": tags_phases,
        "current_phase": 1,
        "current_tag_in_phase":0,
        "tag_4cm": tag_4cm,
        "buttons_found": buttons_found,
        "skipped_buttons": 0,
        "verbosity": verbosity
    }

    rospy.Subscriber("/obj1_get_tag", Float32, lambda msg: callback(msg, params, group))
    rospy.Subscriber("/obj1_skip_tag", Float32, lambda msg: skip_tag(msg, params))

    rospy.spin()


if __name__=='__main__':
    main()
