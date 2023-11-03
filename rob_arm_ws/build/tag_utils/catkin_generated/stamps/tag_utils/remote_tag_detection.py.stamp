#ROS node that subscribes to the image topic, detects the arTag, and publishes the tag's pose with respect to the base_link frame

import rospy
import cv2
import numpy as np
import math
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
#import pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import time

def matrix_to_angles(R):
    #from a rotation matrix, return the euler angles
    #R is a 3x3 rotation matrix
    #return a 3x1 vector of euler angles
    
    R=np.array(R)
    print("R shape: ", R.shape)
    print("R: ", R)

    alpha=math.atan2(R[2][1],R[2][2])
    beta=math.atan2(-R[2][0],math.sqrt(R[2][1]**2+R[2][2]**2))
    gamma=math.atan2(R[1][0],R[0][0])

    return np.array([alpha,beta,gamma])

def process_button(t,params):

    #buttons are 3 cm under the tag
    offset_z=0.03 + params["marker_size"]/2
    offset_x=0.021
    offset_y=0

    return t-np.array([offset_x,offset_y,offset_z])

def process_tag(corners,id, params):
    
    tag_size=params["marker_size"]
    #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(arTag[0], tag_size, params["camera_matrix"], params["dist_coeff"])
    #q = tf.transformations.quaternion_from_euler(*rvec[0][0], axes='sxyz')

    #use cv2 solve PnP
    #get corners of the tag
    _, rvec, tvec = cv2.solvePnP(params["marker_points"], corners, params["camera_matrix"], params["dist_coeff"])
    rvec=np.array(rvec).reshape(3,)
    tvec=np.array(tvec).reshape(3,)
    #print("rvec: ", rvec, "\ntvec: ", tvec)
    
    q = tf.transformations.quaternion_from_euler(*rvec, axes='sxyz')
        
    #transform pose from camera frame to base_link frame
    #create transform listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    time.sleep(1)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose.position.x = tvec[0]
    pose_stamped.pose.position.y = tvec[1]
    pose_stamped.pose.position.z = tvec[2]
    pose_stamped.pose.orientation.x = q[0]
    pose_stamped.pose.orientation.y = q[1]
    pose_stamped.pose.orientation.z = q[2]
    pose_stamped.pose.orientation.w = q[3]
    pose_stamped.header.frame_id = params["camera_frame"]
    pose_stamped.header.stamp = rospy.Time.now()
    #get transform from camera frame to base_link frame
    try:
        transform = tfBuffer.transform(pose_stamped, params["base_frame"], rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print("error in transform", e)
        return


    #print marker pose
    print("Marker ID: " + str(id))
    #print original aruco marker pose
    print("\tOriginal pose: ")
    print("\t\tx:",tvec[0])
    print("\t\ty:",tvec[1])
    print("\t\tz:",tvec[2])

    #print transformed aruco marker pose
    print("\tTransformed pose: ")
    print("\t\tx: " + str(transform.pose.position.x))
    print("\t\ty: " + str(transform.pose.position.y))
    print("\t\tz: " + str(transform.pose.position.z))
    

    #Apply offsets 
    if params["use_offsets"]:
        if id in params["buttons_id"]:
            x=transform.pose.position.x
            y=transform.pose.position.y
            z=transform.pose.position.z

            new_pos=np.array([x,y,z])
            new_pos=process_button(new_pos,params)
            transform.pose.position.x=new_pos[0]
            transform.pose.position.y=new_pos[1]
            transform.pose.position.z=new_pos[2]

    marker_msg=Marker()
    marker_msg.header.frame_id=params["base_frame"]
    marker_msg.header.stamp=rospy.Time.now()
    marker_msg.id=id
    marker_msg.pose=transform.pose
    
    if id in params["buttons_id"] and params["use_offsets"]:
        marker_msg.type=Marker.CYLINDER
    else:
        marker_msg.type=Marker.CUBE

    marker_msg.scale.x=params["marker_size"]
    marker_msg.scale.y=params["marker_size"]
    marker_msg.scale.z=0.01

    #color and scale
    marker_msg.color.r=1.0
    marker_msg.color.g=0.0
    marker_msg.color.b=0.5
    marker_msg.color.a=0.5

    #marker_msg.action=Marker.ADD
    params["tag_pub"].publish(marker_msg)

def callback(data,params):
    print("received a message from image topic")
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
        print(e)

    #detect arTag original Aruco
    arTag = cv2.aruco.detectMarkers(cv_image, cv2.aruco.Dictionary_get( cv2.aruco.DICT_ARUCO_ORIGINAL), parameters=params["detectorParams"])

    if arTag :
        for i in range(len(arTag[0])):
 
            id=arTag[1][i][0]
            corners=arTag[0][i]
            process_tag(corners,id,params) 

            
        
    else:
        print("no tags detected")

       


def main():
    rospy.init_node('remote_tag_detection', anonymous=True)
    #subscriber
    camera_info_topic="/camera_image/camera_info"
    image_topic="/camera_image/image_raw"
    
    print("Waiting for camera_info topic...")
    #get one message from camera_info topic to get camera matrix
    camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=rospy.Duration(10))
    camera_matrix = np.array(camera_info.K).reshape(3,3)

    print("received a message from camera_info topic")
    detectorParams = cv2.aruco.DetectorParameters_create()
    detectorParams.markerBorderBits = 1
    #detectorParams.perspectiveRemoveIgnoredMarginPerCell=0.4

    #publisher of pose stamped message
    tag_pub=rospy.Publisher("tag_pose",Marker, queue_size=10)
    marker_size=0.05
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    params= {
        'camera_matrix': camera_matrix,
        'dist_coeff': camera_info.D,
        'image_width': camera_info.width,
        'image_height':  camera_info.height,
        'marker_size': marker_size,
        "camera_frame": camera_info.header.frame_id,
        "base_frame": "base_link",
        "detectorParams": detectorParams,
        "tag_pub": tag_pub,
        "marker_points": marker_points,
        "use_offsets": False,
        "buttons_id": [1,2,3,4],
        "lid_id": 13
    }
    print(params)

    
    rospy.Subscriber(image_topic, Image, lambda msg: callback(msg, params))
    rospy.spin()


if __name__=='__main__':
    main()
