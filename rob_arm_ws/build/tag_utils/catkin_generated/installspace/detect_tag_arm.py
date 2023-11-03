# export ROS_IP=192.168.106.197
# export ROS_MASTER_URI=http://192.168.106.69:11311
# export ROS_MASTER_URI=http://192.168.106.69:11311

import numpy as np
import cv2
import time
import json
import math
import os
from pathlib import Path
import rospy
from sensor_msgs.msg import Image, CameraInfo

#import float32MultiArray
from std_msgs.msg import Float32MultiArray


from cv_bridge import CvBridge
#import utils.custom_utils as utils
 
#### pasted ###

import math
import cv2
import numpy as np


class Marker():

    def __init__(self, x, y, z, roll, pitch, yaw ):
        self.x = x
        self.y = y
        self.z = z
        
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw

    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_z(self): return self.z
    def get_roll(self):  return self.roll
    def get_pitch(self): return self.pitch
    def get_yaw(self):   return self.yaw

    def get_distance(self): return { 'x': self.get_x(), 'y':self.get_y(), 'z': self.get_z()}
    def get_orientation(self): return {'roll': self.get_roll, 'pitch': self.get_pitch, 'yaw': self.yaw}

    def get_estimated_distance(self):
        x = self.get_x()
        y = self.get_y()
        z = self.get_z()

        estimated_distance = math.sqrt(x*x + y*y + z*z)

        return estimated_distance

    def get_text(self):

        x = self.get_x()
        y = self.get_y()
        z = self.get_z()

        roll = self.get_roll()
        pitch = self.get_pitch()
        yaw = self.get_yaw()

        estimated_distance = self.get_estimated_distance()

        text = "x=%4.2f  y=%4.2f z=%4.2f pitch=%4.2f roll=%4.2f yaw=%4.2f, vec_dist=%4.2f"%(x, y, z, math.degrees(pitch),
                                                                       math.degrees(roll),math.degrees(yaw), estimated_distance)

        return text
    

def solve_cube_error(x, y, z):
    return

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

                                    ########################

###############


#todo: usare solo import ... from
########################
#### INITIALIZATION ####
########################

dir_path = os.path.dirname(os.path.realpath(__file__))


# INPUT_VIDEO = str("ath(dir_path).parents[0]) + "/tag_utils_camera/marker_video/mars1.ogv"      # input video. Set 0 for computer camera

INPUT0 = 0
INPUT1 = "data/video_marker/mars3.mp4"

INPUT_VIDEO = INPUT0

# open json file with all marker
#go to parent folder of dir_path
dir_path= str(Path(dir_path).parents[0])
MARKERS_JSON = open(dir_path + '/config/custom_tags.json')
ARRAY_MARKERS = json.load(MARKERS_JSON)     # read all marker and store it
MARK_NUMBER = len(ARRAY_MARKERS)            # number of marker. Should be 18.
BORDER_THICKNESS = 2                        # border's thickness of marker
SIDE_MARKER_SIZE = 0.045                  # the marker size in meters
SCALE_SIDE_DIM = 1                       # to scale dimension when calculate it with tvec

# name of windows used for display frames of camera
WINDOWS_NAME = "Camera"

#show image size in 16:9
OUTPUT_IMAGE_SIZE=  (1920//2,1080//2)


""" 

def set_camera(img, h, w, camera_size = CAMERA_SIZE):

    width = camera_size
    height = int(width*(h/w))
    img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

    return img, width, height

# initialization of some parameters

def initialization():
    calib_json = open(dir_path + '/config/camera_settings.json')
    calib_json = json.load(calib_json)

    camera_matrix = calib_json["K"]["data"]
    camera_distortion = calib_json["D"]["data"]

    camera_matrix = np.array(camera_matrix)
    camera_distortion = np.array(camera_distortion)


    return camera_matrix, camera_distortion

"""
def resize_img(img):
    #resize to specified size
    h, w = img.shape[:2]
    img= cv2.resize(img, OUTPUT_IMAGE_SIZE, interpolation=cv2.INTER_CUBIC)
    return img


# return parameters for aruco dictionary
def set_parameters():

    # parameters in which we specify shape of the tag:
    #   - markerBorderBits: the thickness of the black border
    #   - I tried to change other param to detect
    #     when marker is really near to camera, test with zed

    """
    adaptiveThreshWinSizeMin: minimum window size for adaptive thresholding before finding contours (default 3).
    adaptiveThreshWinSizeMax: maximum window size for adaptive thresholding before finding contours (default 23).
    adaptiveThreshWinSizeStep: increments from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax during the thresholding (default 10).
    adaptiveThreshConstant: constant for adaptive thresholding before finding contours (default 7)
    minMarkerPerimeterRate: determine minimum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image (default 0.03).
    maxMarkerPerimeterRate: determine maximum perimeter for marker contour to be detected. This is defined as a rate respect to the maximum dimension of the input image (default 4.0).
    polygonalApproxAccuracyRate: minimum accuracy during the polygonal approximation process to determine which contours are squares. (default 0.03)
    minCornerDistanceRate: minimum distance between corners for detected markers relative to its perimeter (default 0.05)
    minDistanceToBorder: minimum distance of any corner to the image border for detected markers (in pixels) (default 3)
    minMarkerDistanceRate: minimum mean distance beetween two marker corners to be considered similar, so that the smaller one is removed. The rate is relative to the smaller perimeter of the two markers (default 0.05).
    cornerRefinementMethod: corner refinement method. (CORNER_REFINE_NONE, no refinement. CORNER_REFINE_SUBPIX, do subpixel refinement. CORNER_REFINE_CONTOUR use contour-Points, CORNER_REFINE_APRILTAG use the AprilTag2 approach). (default CORNER_REFINE_NONE)
    cornerRefinementWinSize: window size for the corner refinement process (in pixels) (default 5).
    cornerRefinementMaxIterations: maximum number of iterations for stop criteria of the corner refinement process (default 30).
    cornerRefinementMinAccuracy: minimum error for the stop cristeria of the corner refinement process (default: 0.1)
    markerBorderBits: number of bits of the marker border, i.e. marker border width (default 1).
    perspectiveRemovePixelPerCell: number of bits (per dimension) for each cell of the marker when removing the perspective (default 4).
    perspectiveRemoveIgnoredMarginPerCell: width of the margin of pixels on each cell not considered for the determination of the cell bit. Represents the rate respect to the total size of the cell, i.e. perspectiveRemovePixelPerCell (default 0.13)
    maxErroneousBitsInBorderRate: maximum number of accepted erroneous bits in the border (i.e. number of allowed white bits in the border). Represented as a rate respect to the total number of bits per marker (default 0.35).
    minOtsuStdDev: minimun standard deviation in pixels values during the decodification step to apply Otsu thresholding (otherwise, all the bits are set to 0 or 1 depending on mean higher than 128 or not) (default 5.0)
    errorCorrectionRate error correction rate respect to the maximun error correction capability for each dictionary. (default 0.6).
    aprilTagMinClusterPixels: reject quads containing too few pixels. (default 5)
    aprilTagMaxNmaxima: how many corner candidates to consider when segmenting a group of pixels into a quad. (default 10)
    aprilTagCriticalRad: Reject quads where pairs of edges have angles that are close to straight or close to 180 degrees. Zero means that no quads are rejected. (In radians) (default 10*PI/180)
    aprilTagMaxLineFitMse: When fitting lines to the contours, what is the maximum mean squared error allowed? This is useful in rejecting contours that are far from being quad shaped; rejecting these quads "early" saves expensive decoding processing. (default 10.0)
    aprilTagMinWhiteBlackDiff: When we build our model of black & white pixels, we add an extra check that the white model must be (overall) brighter than the black model. How much brighter? (in pixel values, [0,255]). (default 5)
    aprilTagDeglitch: should the thresholded image be deglitched? Only useful for very noisy images. (default 0)
    aprilTagQuadDecimate: Detection of quads can be done on a lower-resolution image, improving speed at a cost of pose accuracy and a slight decrease in detection rate. Decoding the binary payload is still done at full resolution. (default 0.0)
    aprilTagQuadSigma: What Gaussian blur should be applied to the segmented image (used for quad detection?) Parameter is the standard deviation in pixels. Very noisy images benefit from non-zero values (e.g. 0.8). (default 0.0)
    """

    parameters = cv2.aruco.DetectorParameters_create()
    parameters.markerBorderBits = BORDER_THICKNESS
    #parameters.errorCorrectionRate = 0.01
    #parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    #parameters.polygonalApproxAccuracyRate = 0.01
    #parameters.cornerRefinementWinSize = 3
    #parameters.cornerRefinementMaxIterations = 10
    #parameters.cornerRefinementMinAccuracy = 0.01
    #parameters.adaptiveThreshWinSizeMin = 3
    #parameters.adaptiveThreshWinSizeMax = 23
    #parameters.adaptiveThreshWinSizeStep = 10
    #parameters.adaptiveThreshConstant = 7
    #parameters.minMarkerDistanceRate = 0.01
    #parameters.minMarkerPerimeterRate = 0.005
    #parameters.minCornerDistanceRate = 0.01
    parameters.perspectiveRemoveIgnoredMarginPerCell=0.4   #spiegazione https://github.com/zsiki/Find-GCP


    
    #parameters.minMarkerDistanceRate = 0.01
    #parameters.minMarkerPerimeterRate = 0.01
    #parameters.minCornerDistanceRate = 0.01
    #parameters.minSideLengthCanonicalImg = 1

    return parameters


# draw x and y axis on image
def draw_rf_lines(img, width, height):
    # -- draw RF of camera, just for debug
    # compute the center of the image
    center = (width // 2, height // 2)

    # draw x axis
    cv2.line(img, center, (center[0] + 100, center[1]), (0, 0, 255), 2)

    # draw y axis
    cv2.line(img, center, (center[0],center[1] + 100), (0, 255, 0), 2)

    return img

# return arrays of vectors with position and orientation of markers seen
def estimate_marker_pose(corners, camera_matrix, camera_distortion, side_marker_size = SIDE_MARKER_SIZE):

    # cv2.aruco.drawDetectedMarkers(img, corners, ids ) #draw a box around all the detected markers
    # get pose of all single markers
    # rvec: rotation vectors - how much i must rotate the camera to match the marker RF pose(is a axis-angle
    #       rappresentation) -
    # tvec: traslation vectors - how far is the tag in x,y,z from the camera -

    # https://stackoverflow.com/questions/53277597/fundamental-understanding-of-tvecs-rvecs-in-opencv-aruco
               

    # SIZE_MARKER_SIZE if dimension markes's side: 10 cm
    # function take input as meter
    rvec_list, tvec_list, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength=side_marker_size , cameraMatrix=camera_matrix,  distCoeffs=camera_distortion)

    #use cv2.solvePnP to estimate pose of marker
    """     
    rvec_list, tvec_list = [], []
    for i in range(len(corners)):
        # get rotation and traslation vector
        rvec, tvec, _objPoints = cv2.solvePnP(corners[i], markerLength=side_marker_size , cameraMatrix=camera_matrix,  distCoeffs=camera_distortion)
        rvec_list.append(rvec)
        tvec_list.append(tvec) """
    


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
    pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix_markerWRTcamera)

    return roll, pitch, yaw, rotation_matrix_markerWRTcamera

# return marker object
def create_marker_object(tvec, roll, pitch, yaw, scale = SCALE_SIDE_DIM):
    x_coordinate = tvec[0] #*scale
    y_coordinate = tvec[1] #*scale
    z_coordinate = tvec[2] #*scale

    #marker = utils.Marker(x_coordinate, y_coordinate, z_coordinate, roll, pitch, yaw)
    marker = Marker(x_coordinate, y_coordinate, z_coordinate, roll, pitch, yaw)

    return marker

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


def active_subscriber():
    rospy.init_node('detect_tag_jacopo', anonymous=True)
    
    #get one message from camera_info topic
    camera_info = rospy.wait_for_message("zed2i/zed_node/left/camera_info", CameraInfo)
    #extract distortion matrices
    camera_matrix = np.array(camera_info.K).reshape(3,3)
    camera_distortion = np.array(camera_info.D)

    print("camera_info: \n", camera_info)

    aruco_dict = create_custom_dictionary(MARK_NUMBER, ARRAY_MARKERS)
    #camera_matrix, camera_distortion = initialization()
    parameters = set_parameters()
    bridge=CvBridge()

    #publish 1D array on ROS
    pub = rospy.Publisher('marker_poses', Float32MultiArray, queue_size=10)
    params={
        "aruco_dict":aruco_dict,
        "camera_matrix":camera_matrix,
        "camera_distortion":camera_distortion,
        "bridge":bridge,
        "parameters":parameters,
        "pub":pub
    }
    #"zed2i/zed_node/left/image_rect_color"
    rospy.Subscriber("img_topic", Image, lambda img: process_img(img,params))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def process_img(img,params):
    
    if img is not None:

        
        img=params["bridge"].imgmsg_to_cv2(img,desired_encoding="rgb8",)

        #print("took ..")
        h, w, _ = img.shape

        ### modified ###

       # img=img[:,:int(w/2),:]
        #w = int(w/2)

        ########

        #img, width, height = set_camera(img, h, w)
        width=w
        height=h


        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #The order of the corners is clockwise.
        corners, ids, rejected = cv2.aruco.detectMarkers(img, dictionary=params["aruco_dict"], parameters=params["parameters"])

        print(f"found  {len(corners)} markers and {len(rejected)} rejected markers")

        #convert to rgb for visualization
        #opencv uses bgr
        img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #print("check ..")

        if ids is not None:
            
            
            #create multiarray
            marker_pose = Float32MultiArray()

            
            ids = ids + 1
            rvec_list, tvec_list = estimate_marker_pose(corners, params["camera_matrix"], params["camera_distortion"])
            if rvec_list is None:
                return
            
            #for rvec, tvec in rvec_list, tvec_list:
            #    img = extract_marker_routine(rvec, tvec, width, height, corners, ids, rejected)

            #print(rvec_list,"\n---\n",tvec_list)

            #for each marker detected
            for i in range(len(rvec_list)):
                rvec = rvec_list[i][0]
                tvec = tvec_list[i][0]  
                print("ID:", ids[i]) 
                print("\trotations:",rvec,"\n \t translations",tvec)
                print()

                marker_pose.data = tvec
                params["pub"].publish(marker_pose)

                roll, pitch, yaw, rotation_matrix_markerWRTcamera = matrix_marker_pose(rvec)
            
                # marker contain all information about marker
                marker = create_marker_object(tvec, roll, pitch, yaw)
                
                #write on image all information about marker
                text_scale=1.8
                img = put_text_coordinates( marker, img,ids[i][0],text_scale, width, height-(50*i), 0)

                # print the distance between marker-camera - #### to scale ####
                #cv2.putText(img, "d=%4.2f" % np.linalg.norm(tvec),(width-100, 25), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 2, cv2.LINE_AA)

                # draw marker on image
                detected_markers = aruco_display(corners, ids, rejected, img)

                # for debug
                img = draw_rf_lines(img, width, height)
            
            #reduce image size
            detected_markers=resize_img(detected_markers)
            cv2.imshow(WINDOWS_NAME, detected_markers)
        else:
            #reduce image size
            img = resize_img(img)
        
            cv2.imshow(WINDOWS_NAME, img)

        key = cv2.waitKey(1) & 0xFF

def extract_marker_routine(rvec, tvec, width, height, corners, ids, rejected): 
            
    roll, pitch, yaw, rotation_matrix_markerWRTcamera = matrix_marker_pose(rvec)
        

            # marker contain all information about marker
    marker = create_marker_object(tvec, roll, pitch, yaw)

            # write on image all information about marker
            #img = utils.put_text_coordinates( marker, img, width, height, 0, SCALE_SIDE_DIM)
    img = put_text_coordinates( marker, img, width, height, 0, SCALE_SIDE_DIM)

            # print the distance between marker-camera - #### to scale ####
    cv2.putText(img, "d=%4.2f" % np.linalg.norm(tvec),(width-100, 25), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 2, cv2.LINE_AA)

            # draw marker on image
            
            #detected_markers = utils.aruco_display(corners, ids, rejected, img)
    detected_markers = aruco_display(corners, ids, rejected, img)

            # for debug
    img = draw_rf_lines(img, width, height)

    return img

def open_camera_and_detect_marker(cap):

    aruco_dict = utils.create_custom_dictionary(MARK_NUMBER, ARRAY_MARKERS)

    camera_matrix, camera_distortion = initialization()

    parameters = set_parameters()
    data
    while cap.isOpened():

        ret, img = cap.read()

        # subscribe to the topic and take image from zed, to work with ros
        #img = active_subscriber()

        if img is not None:

            h, w, _ = img.shape

            ### modified ###

            img=img[:,:int(w/2),:]
            #w = int(w/2)

            ########

            img, width, height = set_camera(img, h, w)

            corners, ids, rejected = cv2.aruco.detectMarkers(img, dictionary=aruco_dict, parameters=parameters)
                

            if ids is not None:

                ids = ids + 1

                rvec_list, tvec_list = estimate_marker_pose(corners, camera_matrix, camera_distortion)

                if rvec_list is None:
                    continue

                rvec = rvec_list[0][0]
                tvec = tvec_list[0][0]        
                print(rvec)
                
                roll, pitch, yaw, rotation_matrix_markerWRTcamera = matrix_marker_pose(rvec)
            

                # marker contain all information about marker
                marker = create_marker_object(tvec, roll, pitch, yaw)

                # write on image all information about marker
                img = utils.put_text_coordinates( marker, img, width, height, 0, SCALE_SIDE_DIM)

                # print the distance between marker-camera - #### to scale ####
                cv2.putText(img, "d=%4.2f" % np.linalg.norm(tvec),(width-100, 25), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 2, cv2.LINE_AA)

                # draw marker on image
                
                detected_markers = utils.aruco_display(corners, ids, rejected, img)
                
                # for debug
                img = draw_rf_lines(img, width, height)

                cv2.imshow(WINDOWS_NAME, detected_markers)
            else:
                cv2.imshow(WINDOWS_NAME, img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or img is None:
            break
    cv2.destroyAllWindows()
    cap.release()


try:
    #cap = cv2.VideoCapture(INPUT_VIDEO)
    #cap = 0
    #open_camera_and_detect_marker(cap)
    active_subscriber()
except Exception as e:
    print("-- error")
    print(e)
    #cv2.destroyAllWindows()
    #cap.release()