# export ROS_IP=192.168.106.197
# export ROS_MASTER_URI=http://192.168.106.69:11311
# export ROS_MASTER_URI=http://192.168.106.69:11311

import numpy as np
import cv2
import time
import json
import math
from math import pi
import os
from pathlib import Path
import rospy
from sensor_msgs.msg import Image, CameraInfo

#import float32MultiArray
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from geometry_msgs.msg import Pose

#for position conversion
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from cv_bridge import CvBridge
#import utils.custom_utils as utils

class Marker():

    def __init__(self, pos,rot, params):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]

        self.roll  = rot[0]
        self.pitch = rot[1]
        self.yaw   = rot[2]

        self.covariance=0

        self.cube_side = params["cube_dimension"]
        self.marker_dimension = params["marker_dimension"]


    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_z(self): return self.z
    def get_roll(self):  return self.roll
    def get_pitch(self): return self.pitch
    def get_yaw(self):   return self.yaw
    def get_orientation(self): return {'roll': self.get_roll, 'pitch': self.get_pitch, 'yaw': self.yaw}

    def get_distance(self):
        x = self.get_x()
        y = self.get_y()
        z = self.get_z()

        estimated_distance = math.sqrt(x*x + y*y + z*z)

        return estimated_distance

    def correct_tag_position(self):
        #measure landmark center instead of tag center
        
        x = self.x
        y = self.y
        z = self.z
        yaw = self.yaw

        cube_offset= self.cube_side/2

        new_x = x + (cube_offset)*math.cos(yaw)
        new_y = y + (cube_offset)*math.sin(yaw)

        new_pos=[new_x,new_y,z]
        self.update_pos(new_pos)

        return  new_pos

    def get_text(self):

        x = self.get_x()
        y = self.get_y()
        z = self.get_z()

        roll = self.get_roll()
        pitch = self.get_pitch()
        yaw = self.get_yaw()
        
        text = "x=%4.2f  y=%4.2f z=%4.2f pitch=%4.2f roll=%4.2f yaw=%4.2f"%(x, y, z, math.degrees(pitch),
                                                                       math.degrees(roll),math.degrees(yaw))

        return text
    
    def update_pos(self,pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
    
    def update_rot(self,rot):
        self.roll = rot[0]
        self.pitch = rot[1]
        self.yaw = rot[2]

    def estimate_covariance(self):
        #fill corresponding marker pose
        distance= self.get_distance()
        
        #estimate covariance: error=0.1 * x-0.1 da riestimare
        covariance=max(0,1e-3*distance - 1e-3)
        covariance+=0.1 #some constant noise

        covariance*=covariance #covariance has sigma^2
        self.covariance=covariance  
        return covariance

    def transform_pose(self,from_frame,to_frame,params):
        pos=[self.x,self.y,self.z]
        rot=[self.roll,self.pitch,self.yaw]

        if from_frame == to_frame:
            return pos,rot
        my_pose = Pose()
        my_pose.position.x = pos[0]
        my_pose.position.y = pos[1]
        my_pose.position.z = pos[2]

        #from euler to quaternion
        q=quaternion_from_euler(rot[0],rot[1],rot[2])
        my_pose.orientation.x = q[0]
        my_pose.orientation.y = q[1]
        my_pose.orientation.z = q[2]
        my_pose.orientation.w = q[3]

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = my_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = params["buffer"].transform(pose_stamped, to_frame, rospy.Duration(1))
            new_pos= output_pose_stamped.pose.position
            q= output_pose_stamped.pose.orientation
            new_rot= euler_from_quaternion([q.x,q.y,q.z,q.w])
            
            pos= [new_pos.x,new_pos.y,new_pos.z]
            rot= [new_rot[0],new_rot[1],new_rot[2]]    

            self.update_pos(pos)
            self.update_rot(rot)
            return pos,rot

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

# write on image some information and return it
# as a class
def put_text_coordinates(marker, img, id,scale, width, height, offsety):
    # draw text on the image
    font = cv2.FONT_HERSHEY_PLAIN
    

    text = marker.get_text()
    
    # calculates text size based on font
    text_size = cv2.getTextSize(text, font, 1, 2)[0]

    text_x = (width- text_size[0]) // 2
    text_y = height - text_size[1] - offsety
    text=f"id: {id} |  "+text
    #cv2.putText(img, tvec_str, (20,260), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),2,cv2.LINE_AA)
    cv2.putText(img, text, (text_x, text_y), font, scale, (0, 0, 255), 2,cv2.LINE_AA)
    
    return img

  
# we could also use aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

# aruco.custom_dictionary(	nMarkers, markerSize[, randomSeed]	) -
#	-nMarkers: the number of markers used
#	-markerSize: marker dimension (example tag 5x5)
#	-randomSeed: it's the seed for generate random value. Do not use
# create custom dictionary
def create_custom_dictionary(MARK_NUMBER, ARRAY_MARKERS):
    aruco_dict = cv2.aruco.custom_dictionary(MARK_NUMBER, 5)

    # add empty bytesList which contain MARKER_NUMBER marker
    aruco_dict.bytesList = np.empty(shape = (MARK_NUMBER, 4, 4), dtype = np.uint8)

    # adding all marker into dictionary
    for i, key in enumerate(ARRAY_MARKERS): 
        mark = ARRAY_MARKERS[key]
        mybits = np.array(mark, dtype = np.uint8)
        aruco_dict.bytesList[i] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
        
    return aruco_dict

# function for draw line around the markerj  
def aruco_display(corners, ids, rejected, image, old_id=0):

    if len(corners) > 0:
		
        ids = ids.flatten()
        
        for (markerCorner, markerID) in zip(corners, ids):
			
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
			
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
            cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
   
            
            #print("[Inference] ArUco marker ID: {}".format(markerID))
			
    return image

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Elementar rotations matrix 
def elemetar_Rx(theta):
    return np.matrix([[ 1, 0           , 0           ],
    [ 0, math.cos(theta),-math.sin(theta)],
    [ 0, math.sin(theta), math.cos(theta)]])

def elemetar_Ry(theta):
    return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
    [ 0           , 1, 0           ],
    [-math.sin(theta), 0, math.cos(theta)]])

def elemetar_Rz(theta):
    return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
    [ math.sin(theta), math.cos(theta) , 0 ],
    [ 0           , 0            , 1 ]])

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def resize_img(img):
    #resize to specified size
    h, w = img.shape[:2]
    OUTPUT_IMAGE_SIZE=  (1920//2,1080//2)
    img= cv2.resize(img, OUTPUT_IMAGE_SIZE, interpolation=cv2.INTER_CUBIC)
    return img

# return parameters for aruco dictionary
def set_parameters(params):

    parameters = cv2.aruco.DetectorParameters_create()

    parameters.markerBorderBits = params["border_thickness"]
    parameters.minMarkerPerimeterRate = 0.005
    parameters.perspectiveRemoveIgnoredMarginPerCell=0.33  #spiegazione https://github.com/zsiki/Find-GCP
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    #parameters.maxErroneousBitsInBorderRate = 0.5

    return parameters

# draw x and y axis on image
def draw_rf_lines(img, width, height):
    # -- draw RF of camera, just for debug
    # compute the center of the image
    center = (width // 2, height // 2)

    # draw x axis
    cv2.line(img, center, (center[0] - 100, center[1]), (0, 255, 0), 2)

    # draw y axis
    cv2.line(img, center, (center[0],center[1] - 100), (255, 0, 0), 2)

    return img

# return arrays of vectors with position and orientation of markers seen
def estimate_marker_pose(corners, camera_matrix, camera_distortion, side_marker_size):

    # cv2.aruco.drawDetectedMarkers(img, corners, ids ) #draw a box around all the detected markers
    # get pose of all single markers
    # rvec: rotation vectors - how much i must rotate the camera to match the marker RF pose(is a axis-angle
    #       rappresentation) -
    # tvec: traslation vectors - how far is the tag in x,y,z from the camera -

    # https://stackoverflow.com/questions/53277597/fundamental-understanding-of-tvecs-rvecs-in-opencv-aruco
               

    # SIZE_MARKER_SIZE if dimension markes's side
    # function take input as meter
    rvec_list, tvec_list, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength=side_marker_size , cameraMatrix=camera_matrix,  distCoeffs=camera_distortion)

    return rvec_list, tvec_list

# return R_axis(theta) and rotation matric respect to camera from R(theta, r)
def matrix_marker_pose(rvec):

    # from vector rotation to rotation matrix
    rotation_matrix_markerWRTcamera = cv2.Rodrigues(rvec)[0]
    

    # we want the same initial pose as the camera
    #rotation_matrix_markerWRTcamera = np.dot(rotation_matrix_markerWRTcamera, utils.elemetar_Rx(math.pi))
    rotation_matrix_markerWRTcamera = np.dot(rotation_matrix_markerWRTcamera, elemetar_Rx(math.pi))

    # get the position of the marker WRT camera
    #tvecWRTcamera = np.dot(rotation_matrix_markerWRTcamera, tvec_flipped)

    # we only need yaw angle
    #pitch, roll, yaw = utils.rotationMatrixToEulerAngles(rotation_matrix_markerWRTcamera)
    roll, pitch, yaw = rotationMatrixToEulerAngles(rotation_matrix_markerWRTcamera)

    return roll, pitch, yaw, rotation_matrix_markerWRTcamera

# return position and orientation of camera respect to object
def camera_pose_wrt_object(rotation_matrix_markerWRTcamera, tvec):

    # -- now let's compute the position and orientation of the camera wrt the object.
    # -- in our case is more useful because the camera is the object that moves
    
    rotation_matrix_cameraWRTmarker = np.transpose(rotation_matrix_markerWRTcamera)

    tvec_cameraWRTmarker = np.dot(rotation_matrix_markerWRTcamera, tvec)

    # Convert from 2D to a 1D array
    tvec_cameraWRTmarker = np.squeeze(np.array(tvec_cameraWRTmarker))


    #pitch, roll, yaw = utils.rotationMatrixToEulerAngles(rotation_matrix_cameraWRTmarker)
    pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix_cameraWRTmarker)

    return roll, pitch, yaw


def process_img(img,params):
    if img is None: return

    img=params["bridge"].imgmsg_to_cv2(img,desired_encoding="rgb8",)

    height, width, _ = img.shape

    #The order of the corners is clockwise.
    corners, ids, rejected = cv2.aruco.detectMarkers(img, dictionary=params["aruco_dict"], parameters=params["parameters"])
    if len(corners) > 0:
        print("\n\n")
        print(f"found  {len(corners)} markers and {len(rejected)} rejected markers")

    #convert to rgb for visualization
    #opencv uses bgr
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if ids is not None:
        
        #initialize array of marker poses
        marker_poses = Float32MultiArray()  

        #initialize data
        #format: x y covariance
        marker_poses.data = [[0.0, 0.0, 0.0] for i in range(15)]

        ids = ids + 1
        rvec_list, tvec_list = estimate_marker_pose(corners, params["camera_matrix"], 
            params["camera_distortion"],params["marker_dimension"])
        if rvec_list is None:
            return

        
        #for each marker detected
        for i in range(len(rvec_list)):
            rvec = rvec_list[i][0]
            tvec = tvec_list[i][0]  

            print("\nID:", ids[i]) 
            #convert in degrees
            rvec_deg=np.degrees(rvec)
            print("Rotations (deg):",np.round(rvec_deg,3)," Translations (m):",np.round(tvec,3))

            roll, pitch, yaw, rotation_matrix_markerWRTcamera = matrix_marker_pose(rvec)
            rvec=np.array([roll,pitch,yaw])
            rvec_deg=np.degrees(rvec)
            print("Rotation after processing: ",np.round(rvec_deg,3))
            
            #Create marker object
            marker=Marker(tvec,rvec,params)

            try:

                #transform tag pose in other tf frame
                marker.transform_pose(params["camera_frame"],params["target_frame"],params)
                print(f"marker pose w.r.t {params['target_frame']}--> ",end="")
                print(" xyz",np.round([marker.x,marker.y,marker.z],3),end="")
                #rotations
                rots=[marker.roll,marker.pitch,marker.yaw]
                #add 90 degrees to yaw to have it in the same frame as the rover
                rots[2]+=pi/2
                rots[0]+=pi/2
                marker.update_rot(rots)

                print(" rpy: ",np.round(np.degrees(rots),3))
                
                #estiamte covariance based on distance from rover
                cov=marker.estimate_covariance()

                #get landmark center rather than tag center
                correct_tag_pos=marker.correct_tag_position()
                print("corrected tag pos",np.round(correct_tag_pos,3))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                return
            
            marker_pose=[marker.x,marker.y,marker.covariance]
                
            marker_poses.data[ids[i][0]-1]=marker_pose

            #write on image all information about marker
            text_scale=1.5
            img = put_text_coordinates( marker, img,ids[i][0],text_scale, width, height-(50*i), 0)
            # draw marker on image
            detected_markers = aruco_display(corners, ids, rejected, img)
            # for debug
            img = draw_rf_lines(img, width, height)
        
        #publish on topic
        
        #put in monodimensional array, easier to publish
        #to be unwrapped in other nodes
        marker_poses.data = [item for sublist in marker_poses.data for item in sublist]
        
        #publish marker poses
        params["pub"].publish(marker_poses)
        
        #reduce image size
        detected_markers=resize_img(detected_markers)
        cv2.imshow("camera", detected_markers)
    else:
        #reduce image size
        img = resize_img(img)
        cv2.imshow("camera", img)

    key = cv2.waitKey(1) & 0xFF


def active_subscriber():
    rospy.init_node('detect_tag', anonymous=True)
    
    #get one message from camera_info topic
    camera_info = rospy.wait_for_message("camera_info", CameraInfo)
    
    #extract distortion matrices
    camera_matrix = np.array(camera_info.K).reshape(3,3)
    camera_distortion = np.array(camera_info.D)
    camera_frame=camera_info.header.frame_id
    print("camera_info: \n", camera_info)

    custom_tag_path=rospy.get_param("~custom_tag_path", "")
    if not custom_tag_path:
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # open json file with all marker
        #go to parent folder of dir_path
        dir_path= str(Path(dir_path).parents[0])
        custom_tag_path=dir_path + '/config/src/custom_tags.json'

    MARKERS_JSON=open(custom_tag_path)
    ARRAY_MARKERS = json.load(MARKERS_JSON)     # read all marker and store it
    MARK_NUMBER = len(ARRAY_MARKERS)            # number of marker. Should be 15.

    aruco_dict = create_custom_dictionary(MARK_NUMBER, ARRAY_MARKERS)

    
    bridge=CvBridge()

    #publish 1D array on ROS
    pub = rospy.Publisher('marker_poses', Float32MultiArray, queue_size=10)
    tf_buffer=tf2_ros.Buffer()
    tf_listener=tf2_ros.TransformListener(tf_buffer)
    params={
        "aruco_dict":aruco_dict,
        "camera_matrix":camera_matrix,
        "camera_distortion":camera_distortion,
        "camera_frame":camera_frame,
        "bridge":bridge,
        "pub":pub,
        "rate":rospy.Rate(10), # 10hz
        "target_frame": rospy.get_param("~target_frame", ""),
        "buffer": tf_buffer,
        "listener": tf_listener,
        "cube_dimension": rospy.get_param("~landmark_size", 0.22), #landmark dimension
        "marker_dimension":rospy.get_param("~tag_size", 0.15), #tag dimension
        "border_thickness": rospy.get_param("~border_thickness", 2), #num of black bits of padding 
    }
    parameters = set_parameters(params)
    params["parameters"]=parameters

    rospy.Subscriber("img_topic", Image, lambda img: process_img(img,params))
    rospy.spin()



active_subscriber()
""" try:

    active_subscriber()
except Exception as e:
    print("-- error")
    print(e)
    #cv2.destroyAllWindows()
    #cap.release() """