#ROS node that subscribes to the image topic, detects the arTag, and publishes the tag's pose with respect to the base_link frame

import rospy
import cv2
import numpy as np
#import math
from sensor_msgs.msg import Image, CameraInfo
#import pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tag_utils.tagDetector import Tag, TagDetector
import math

try:
    from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
except ImportError:
    AprilTagDetection = None

# return parameters for aruco dictionary
def set_parameters():

    parameters = cv2.aruco.DetectorParameters_create()

    parameters.markerBorderBits = 2
    parameters.minMarkerPerimeterRate = 0.005
    #parameters.perspectiveRemoveIgnoredMarginPerCell=0.33  #spiegazione https://github.com/zsiki/Find-GCP
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    #parameters.maxErroneousBitsInBorderRate = 0.5

    return parameters

#TODO correct landmark pose
def correct_landmark_pose(tag,params):
    #print("Tag pose before correction:")
    #tag.print_tag(1)
    cube_offset=params["landmark_size"]/2
    old_x = tag.T[0]
    old_y = tag.T[1]
    yaw = tag.R[1]
    #occhio che yaw deve essere l'asse giusto
    new_x = old_x + (cube_offset)*math.cos(yaw)
    new_y = old_y + (cube_offset)*math.sin(yaw)
    tag.T[0] = new_x
    tag.T[1] = new_y
    #print("Tag pose after correction:")
    #tag.print_tag(1)

#TODO estimate covariance 
def compute_covariance(tag,params):
    #based on the distance of the tag compute covariance, the closer the tag the more precise the measurement
    # at 10 meter of distance the covariance is 0.4 meter, at 1 meter of distance the covariance is 0.05 cm
    distance=math.sqrt(tag.T[0]**2+tag.T[1]**2)
    #print("distance: ", distance)
    #TODO this is valid for the simulation, to be recomputed for the real world
    covariance= distance*0.02 + 0.02
    return covariance


def callback(data,params):
    marker_poses = Float32MultiArray()  
    default_cov=params["default_tag_cov"]
    marker_poses.data = [[0.0, 0.0, default_cov] for i in range(15)]
    tagDetector=params["tagDetector"]
    tags=tagDetector.detect_from_ros_image(data)

    rtabmap_tags = None
    if AprilTagDetection is not None:
        rtabmap_tags=AprilTagDetectionArray()
        rtabmap_tags.header.stamp = rospy.Time.now()
        rtabmap_tags.header.frame_id = params["base_frame"]
        rtabmap_tags.detections = []

    for tag in tags:

        tag.transform_tag_frame(params["base_frame"])
        correct_landmark_pose(tag,params)
        #publish
        if rtabmap_tags is not None:
            aprilTagMsg=AprilTagDetection()
            aprilTagMsg.id=tag.id
            aprilTagMsg.size=params["marker_size"]
            aprilTagMsg.pose=tag.get_pose_with_covariance_stamped_msg(cov_T=default_cov,cov_R=0.1) #R to be set very big?
            rtabmap_tags.detections.append(aprilTagMsg)

        params["tag_pub"].publish(tag.get_marker_msg())
        id=tag.id
        print("found tag with id: ", id)
        print("tag pose: ", tag.T)
        marker_poses.data[id-1]=[tag.T[0], tag.T[1], compute_covariance(tag,params)]
    if rtabmap_tags is not None:
        params["rtabmap_tag_pub"].publish(rtabmap_tags)
    marker_poses.data = [item for sublist in marker_poses.data for item in sublist]
    params["marker_poses_pub"].publish(marker_poses)

def main():
    rospy.init_node('remote_tag_detection', anonymous=True)
    verbosity=rospy.get_param("~verbose",0)
    marker_size=rospy.get_param("~tag_size")
    landmark_size=rospy.get_param("~landmark_size")
    tagDetector=TagDetector(marker_size,verbose=verbosity)
    #subscriber
    camera_info_topic="camera_info_topic"
    image_topic="img_topic"
    
    print("Waiting for camera_info topic...", end="", flush=True)
    #get one message from camera_info topic to get camera matrix
    camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=rospy.Duration(10))
    tagDetector.set_camera_from_camera_info(camera_info)
    print("Received!")
    print("Camera K matrix: ", np.round(tagDetector.camera_K,4))
    print("Camera distortion coefficients: ", np.round(tagDetector.camera_D,4))
    detectorParams = set_parameters()
    #detectorParams.perspectiveRemoveIgnoredMarginPerCell=0.4
    dictionary_path=rospy.get_param("~dictionary_path")
    tagDetector.set_detector_params(detectorParams,dictionary=dictionary_path,custom=True)

    #publisher of pose stamped message
    tag_pub=rospy.Publisher("tag_pose",Marker, queue_size=10)
    rtabmap_tag_pub=rospy.Publisher("/rtabmap_tag_detection",AprilTagDetectionArray, queue_size=10)
    #multiarray publisher
    marker_poses_pub=rospy.Publisher("marker_poses",Float32MultiArray, queue_size=10)
    base_frame=rospy.get_param("~target_frame","")
    if base_frame=="":
        print("target_frame param not set,using camera one")
        base_frame=tagDetector.camera_frame

    print("Base frame: ", base_frame)
    params= {
        "base_frame": base_frame,
        "tag_pub": tag_pub,
        "rtabmap_tag_pub": rtabmap_tag_pub,
        "marker_poses_pub": marker_poses_pub,
        "tagDetector": tagDetector,
        "default_tag_cov": 1e-3,
        "verbosity": verbosity,
        "landmark_size": landmark_size,
        "marker_size": marker_size
    }
    
    rospy.Subscriber(image_topic, Image, lambda msg: callback(msg, params))
    rospy.spin()


if __name__=='__main__':
    main()
