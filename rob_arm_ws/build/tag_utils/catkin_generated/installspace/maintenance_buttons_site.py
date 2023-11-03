#ROS node that subscribes to the image topic, detects the arTag, and publishes the tag's pose with respect to the base_link frame

import rospy
import cv2
#check if opencv-python or opencv-python-headless is installed
GUI=False
try:
    cv2.namedWindow("Tag", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Tag", 600,600)
    GUI=True
    cv2.destroyAllWindows()
except:
    print("opencv-python-headless installed, GUI not available")
    pass
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
import time
import easyocr
from itertools import combinations
import tf
from tag_utils.tagDetector import Tag, TagDetector
from cv_bridge import CvBridge, CvBridgeError
import os


#trying new approach, tags are detected not by using size of tag in pixel as in detect_numbers.py
#here I use the estiamted position of the button converted in pixel
cv2_4_7=False
try:
    cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
except AttributeError:
    cv2_4_7=True
    pass

N_BUTTONS=8
margin=0
button_width=0.025 #original button width
button_height=0.05
#b_w=button_width + margin*2 #button width and margin
#b_h=button_height + margin*2 #button height
d_x=0.084 #distance between tag/button element on the right and the one on the left
d_x=0.11 #on our panel
d_y=0.071 #distance between tag/button element on the top and the one on the bottom
d_y=0.077 #on our panel
button_coordinates=np.zeros((N_BUTTONS,3))

to_press=np.zeros((5,3))


#Main buttons offset
#main_button_offset=[d_x-b_w/2,0]

#compute button offsets
def compute_button_offsets(tag,params,margin=0):
    '''
    Offsets of the buttons with respect to the tag in the panel
        x,y in image (right,up)
        -y,z in base_link frame (left,up)
    '''
    off_x=button_width + margin*2 #button width and margin
    buttons_offset=[
    [-off_x/2 ,      -d_y],
    [off_x/2,       -d_y],
    [d_x - off_x/2, -d_y],
    [d_x + off_x/2, -d_y],
    [-off_x/2,      -d_y*2],
    [off_x/2,       -d_y*2],
    [d_x - off_x/2, -d_y*2],
    [d_x + off_x/2, -d_y*2],
    ]
    print("buttons offset: ",buttons_offset)
    return buttons_offset

def compute_main_position(params,margin=0):
    main_button_offset=[d_x-button_width/2, -(button_height/2 - margin)]
    return main_button_offset

def viz_pos_to_press(image,to_press,params):
    #convert each position to pixel coordinates and put a number on each position to press
    K=params["tagDetector"].camera_K
    D=params["tagDetector"].camera_D
    for i in range(len(to_press)):
        #convert to pixel
        to_press_pixel=find_pixel_coordinates(K,D,to_press[i])
        #put a number on the image
        cv2.putText(image, str(i), (to_press_pixel[0],to_press_pixel[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
    #show image
    viz_img=image
    #resize
    if GUI:
        viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('Tag', viz_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

#preprocess the image
def preprocess_image(image,params,viz=False):
    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #determine threshold based on maximum value
    #thresh = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[0]
    #thresh = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY)[1]
    #apply threshold
    #(T, blackAndWhiteImage) = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)
    
    gray_image = cv2.medianBlur(gray_image,5)
    #blackAndWhiteImage = cv2.adaptiveThreshold(gray_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,11,2)
 
    #divide
    blackAndWhiteImage = cv2.divide(gray_image, 255, scale=255)
    #otsu threshold
    blackAndWhiteImage = cv2.threshold(blackAndWhiteImage, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
    #apply morphology
    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    blackAndWhiteImage = cv2.morphologyEx(blackAndWhiteImage, cv2.MORPH_CLOSE, kernel)
    
    #blackAndWhiteImage = cv2.morphologyEx(blackAndWhiteImage, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
    #dst = cv2.fastNlMeansDenoisingColored(blackAndWhiteImage,None,10,10,7,21)
    
    if GUI and viz:
        viz_img=gray_image
        #resize
        viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
        #cv2.imwrite('black_and_white.png', viz_img)
        cv2.imshow('Preprocessed image', viz_img)
        cv2.waitKey(0)
        #cv2.destroyAllWindows()
    return image

#Given the K and D matrices of the camera and an image, find the corresponding pixel coordinates of a given point
def find_pixel_coordinates(K,D,point):
    #print("find pixel coords of:",point)
    point=np.array(point)
    point=np.array([point])
    point_2d=cv2.projectPoints(point,np.zeros((3,1)),np.zeros((3,1)),K,D)[0][0][0]
    #convert to int
    point_2d=np.array([int(point_2d[0]),int(point_2d[1])])
    #print("found:",point_2d)
    
    return point_2d

#given the 4x4 roto-translatoin matrix of the tag center apply the offset to find the button center
def apply_offset(tag,offset_x,offset_y):
    #offset_x is the distance of the button wrt to the side
    #offset_y is the offset in the up direction
    tag_R=tag.R #roll,pitch,yaw
    tag_T=tag.T #x,y,z
    print("Tag RPY: ",tag_R)


    #get the rotation matrix
    R=np.zeros((3,3))
    #use rodriguez
    R=cv2.Rodrigues(tag_R,R)[0]

    #get the translation vector
    T=np.zeros((3,1))
    T=tag_T.reshape((3,1))

    #get the offset vector
    offset=np.array([offset_x,offset_y,0])
    offset=offset.reshape((3,1))

    #apply the offset
    #print("original offset: ",offset)
    #print("offset after rotation: ",np.matmul(R,offset))
    button_T=T+np.matmul(R,offset)
    button_T=button_T.reshape((3,))
    return button_T

def apply_offset2(tag,offset_x,offset_y,T=None):
    #offset_x is the distance of the button wrt to the side
    #offset_y is the offset in the up direction
    
    R=tag.R #roll,pitch,yaw
    if T is None:
        T=tag.T #x,y,z  
    #print("Tag RPY: ",R)
    #print("applying offset [x,y]",offset_x,offset_y)

    Rot=tf.transformations.euler_matrix(R[0],R[1],R[2])
    Rot=np.array(Rot)
    Rot=Rot[:3,:3]
    #Rot=np.transpose(Rot)
    new_offset=np.matmul(Rot,[offset_x,offset_y,0])
    new_T= np.array(T) + new_offset
    #print("new T: ",new_T)
    return new_T  

def process_button_image(button_image,level,params):
    if level==0:
        return button_image
    
    #to balck and white
    button_image=cv2.cvtColor(button_image, cv2.COLOR_BGR2GRAY)
    if level==1:
        return button_image
    #slight blur
    button_image = cv2.medianBlur(button_image,5)
    #apply dynamic thresholding
    button_image = cv2.adaptiveThreshold(button_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)
    if level==2:
        return button_image
    #apply morphology
    kernel=cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    button_image = cv2.morphologyEx(button_image, cv2.MORPH_CLOSE, kernel)

    return button_image
    
#given the coordinates of a button, get a subset of the image with only the button
def get_button_image(image,button_coordinates,tag,params,viz=False):
    #viz=True
    K=params["tagDetector"].camera_K
    D=params["tagDetector"].camera_D
    
    viz=True
    #get the four corners of the button, convert them to pixel and get a croop of the image
    button_width_margin=0.005
    button_offset_w=button_width/2#+ button_width_margin/2
    button_offset_h=button_height/2 #+ button_width_margin/2

    corner1= apply_offset2(tag,button_offset_w,-button_offset_h,button_coordinates)
    corner2= apply_offset2(tag,-button_offset_w,-button_offset_h,button_coordinates)
    corner3= apply_offset2(tag,-button_offset_w,button_offset_h,button_coordinates)
    corner4= apply_offset2(tag,button_offset_w,button_offset_h,button_coordinates)
    #corner1=[button_coordinates[0]+button_offset_w,button_coordinates[1]-button_offset_h,button_coordinates[2]]
    #corner2=[button_coordinates[0]-button_offset_w,button_coordinates[1]-button_offset_h,button_coordinates[2]]
    #corner3=[button_coordinates[0]-button_offset_w,button_coordinates[1]+button_offset_h,button_coordinates[2]]
    #corner4=[button_coordinates[0]+button_offset_w,button_coordinates[1]+button_offset_h,button_coordinates[2]]
    button_corners=np.array([corner1,corner2,corner3,corner4])

    button_corners_pixel=np.zeros((4,2))
    for i in range(4):
        button_corners_pixel[i]=find_pixel_coordinates(K,D,button_corners[i])
    #print("button coordinates: ",button_coordinates)
    #print("button corners: ",button_corners)
    #print("button corners in pixel: ",button_corners_pixel, "\n\tbutton_pixel[:,0]",button_corners_pixel[:,0],"\n\tbutton_pixel[:,1]",button_corners_pixel[:,1])
    #get the min and max x and y
    min_x=int(min(button_corners_pixel[:,0]))
    max_x=int(max(button_corners_pixel[:,0]))

    min_y=int(min(button_corners_pixel[:,1]))
    max_y=int(max(button_corners_pixel[:,1]))

    #crop the image
    #button_image=image[min_y:max_y,min_x:max_x]
    #instead of a square image get a non rectangular one using the corners, black out the rest
    mask = np.zeros(image.shape, dtype=np.uint8)
    roi_corners = np.array([button_corners_pixel], dtype=np.int32)
    # fill the ROI so it doesn't get wiped out when the mask is applied
    channel_count = image.shape[2]  # i.e. 3 or 4 depending on your image
    ignore_mask_color = (255,)*channel_count
    cv2.fillPoly(mask, roi_corners, ignore_mask_color)
    # apply the mask
    button_image = cv2.bitwise_and(image, mask)
    #crop the image with a margin
    crop_margin_x=int(image.shape[1]*0.02)
    crop_margin_y=int(image.shape[0]*0.02)
    button_image=button_image[min_y-crop_margin_y:max_y+crop_margin_y,min_x-crop_margin_x:max_x+crop_margin_x]
    
    
    
    if GUI and viz:
        viz_img=button_image
        #resize
        viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('Button image', viz_img)
        cv2.waitKey(0)
        #cv2.destroyAllWindows()

    #draw a rectangle on button on original image
    new_image=image.copy()
    #print("min-max:",(min_x,min_y),(max_x,max_y))
    #cv2.rectangle(new_image,(min_x,min_y),(max_x,max_y),(0,255,0),2)
    #draw a polygon on the corners
    button_corners_pixel=button_corners_pixel.astype(int)
    #print("button corners pixel: ",button_corners_pixel)
    cv2.polylines(new_image,[button_corners_pixel],True,(0,255,0),2)
    if GUI:
        viz_img=cv2.resize(new_image, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('Button image', viz_img)
        cv2.waitKey(0)
    return button_image,new_image

#given the image of a button detect the number
def detect_number(image,params):
    result=params["numDetector"].readtext(image)
    #print("detected: ",result)
    number=0
    try:
        #remove characters that are not numbers and convert to int
        number=int(''.join(filter(str.isdigit, result[0][1])))
        print("detection result: {}, original: {}".format(number,result[0][1]))
    except:
        print("number not detected")
        return number
        
    return number

#given a sum find the index of the buttons that sum up to it
#5 buttons have to be selected to achieve the sum
def find_combination_of_5(numbers, target_sum):
    for combination in combinations(numbers, 5):
        #print(combination)
        if sum(combination) == target_sum:
            return combination
    return None


#given an image and a detected tag, detect all the buttons, 
# find the number on each button and find which buttons have to be pressed
def buttonTask(image,tag,params):
    tag_x,tag_y,tag_z=tag.T
    print("tag is in:: ", tag.T)
    viz=True

    #put a circle on the tag in the image
    K=params["tagDetector"].camera_K
    D=params["tagDetector"].camera_D
    tag_pixels=find_pixel_coordinates(K,D,[tag.T])
    cv2.circle(image, (tag_pixels[0],tag_pixels[1]), 10, (0,0,255), -1)
    if GUI and viz:
        viz_img=image
        #resize
        
        viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('Tag', viz_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    '''
    Preprocess image
    '''
    time_process=time.time()
    preprocessed_image=preprocess_image(image,params,viz=False)
    time_process=time.time()-time_process

    #if all buttons have been found, skip this part
    buttons_to_found=False
    for number in params["numbers_found"]:
        if number==0:
            buttons_to_found=True
            break
    total_time_detect=time.time()
    time_image_subset_mean=0
    time_detect_mean=0
    buttons_offset=compute_button_offsets(tag,params,margin=0.005)
    if buttons_to_found:
        '''
        Detect buttons
        '''
        #given the tag coordinates, for each button find its coorsinates by applying the offset,
        #then get a subset of the image with only the button 
        
        for i in range(N_BUTTONS):
            #button_coordinates[i]=np.array([tag_x+buttons_offset[i][0],tag_y-buttons_offset[i][1],tag_z])
            #button_coordinates[i]=apply_offset(tag,buttons_offset[i][0],-buttons_offset[i][1])
            button_coordinates[i]=apply_offset2(tag,buttons_offset[i][0],buttons_offset[i][1])

            #TODO consider 3D?
            '''
            Find each button
            '''
            print("button is in: ",button_coordinates[i])
            time_image_subset=time.time()
            button_image,new_image=get_button_image(preprocessed_image,button_coordinates[i],tag,params,viz=False)
            time_image_subset=time.time()-time_image_subset
            time_image_subset_mean+=time_image_subset


            #preprocessed_image=image
            '''
            Detect number on each button
            '''
            time_detect=time.time()
            processing_level=0
            number=0
            while number==0 and processing_level<3:
                print("\tdetecting number on button {} with processing level {}".format(i,processing_level))
                new_button_image=process_button_image(button_image,processing_level,params)

                #viz
                if GUI:
                    viz_img=new_button_image
                    #resize
                    viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
                    cv2.imshow('Button image level {}'.format(processing_level), viz_img)
                    cv2.waitKey(2000)
                    #cv2.destroyAllWindows()

                number=detect_number(new_button_image,params)
                if number==0:
                    print("Could not detect number on button {}".format(i))
                else:
                    print(f"Button {i}: found {number}")
                    params["numbers_found"][i]=number
                    break
                processing_level+=1
                
            time_detect=time.time()-time_detect
            time_detect_mean+=time_detect
        total_time_detect=time.time()-total_time_detect
        time_image_subset=time_image_subset_mean/N_BUTTONS
        time_detect=time_detect_mean/N_BUTTONS
    else:
        #print("offsets: ",buttons_offset)
        for i in range(N_BUTTONS):
            #button_coordinates[i]=np.array([tag_x+buttons_offset[i][0],tag_y-buttons_offset[i][1],tag_z])
            button_coordinates[i]=apply_offset2(tag,buttons_offset[i][0],buttons_offset[i][1])
    button_numbers=params["numbers_found"]

    '''
    Find combination of buttons that sum up to the desired number
    '''
    time_combination=time.time()
    combination=find_combination_of_5(button_numbers,params["desired_sum"])
    time_combination=time.time()-time_combination
    
    '''
    Get the coordinates of the buttons that have to be pressed
    '''
    to_press=np.zeros((5,3))
    
    if combination is None:
        print("Error: could not find combination of buttons that sum up to {}".format(params["desired_sum"]))
    else:
        for i in range(len(combination)):
            number_index=button_numbers.index(combination[i])
            to_press[i]=button_coordinates[number_index]
            #print("number index: ",number_index)
            #print("button coordinates: ",button_coordinates[number_index])


    #TODO add main button
    main_offsets=compute_main_position(params,margin=0.005)
    main_button_coordinates=np.array([tag_x+main_offsets[0],tag_y-main_offsets[1],tag_z])
    to_press=to_press.tolist()
    to_press=[main_button_coordinates]+to_press

    #visualize the positions to press
    viz_pos_to_press(image,to_press,params)

    print("numbers detected: {}".format(button_numbers))
    print("numbers to press: {}".format(combination))
    print("Positions to press: {}".format(to_press))
    print("Times:")
    print("\tPreprocess image: {}".format(time_process))
    if buttons_to_found:
        print("\tTotal detection time: {}".format(total_time_detect))
        print("\t\tImage subset (single): {}".format(time_image_subset))
        print("\t\tDetect number(single): {}".format(time_detect))
    print("\tFind combination: {}".format(time_combination))
    if GUI:
        cv2.destroyAllWindows()

    return to_press


def request_callback(msg,params):
    to_press_idx=params["to_press_idx"]
    print("Received request for button {}".format(to_press_idx))
    time.sleep(2)
    #get pose stamped of the current position
    position=params["to_press"][to_press_idx]
    if to_press_idx>len(to_press):
        print("task finished,sending null pose")
        position=[0,0,0]

    #publish pose stamped
    pose_stamped=PoseStamped()
    pose_stamped.header.frame_id="base_link"
    pose_stamped.pose.position.x=position[0]
    pose_stamped.pose.position.y=position[1]
    pose_stamped.pose.position.z=position[2]
    q=params["button_q"]
    pose_stamped.pose.orientation.x=q[0]
    pose_stamped.pose.orientation.y=q[1]
    pose_stamped.pose.orientation.z=q[2]
    pose_stamped.pose.orientation.w=q[3]

    print("sending pose: ",pose_stamped.pose)
    params["button_pub"].publish(pose_stamped)
    params["to_press_idx"]+=1
    return





def callback(data,params):

    tagDetector=params["tagDetector"]
    #from ros image get a cv2 black and white image
    cv2_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="mono8")
    #apply dynamic thresholding
    #cv2_image = cv2.adaptiveThreshold(cv2_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,11,2)

    #view cv2 image
    if GUI:
        viz_img=cv2_image
        #resize
        viz_img=cv2.resize(viz_img, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('Tag', viz_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    tags=tagDetector.detect_from_cv_image(cv2_image)
    if len(tags)>0 and params["verbosity"]:
        #print(f"Found {len(tags)} tags in {time.time()-tag_time} seconds")
        pass

    print("found tags with ids: ", [tag.id for tag in tags])

    processing_time=time.time()
    for tag in tags:
        #if len(tags_to_consider)>0 and tag.id not in goal_tag:
        #    continue
        if tag.id!=params["tag_to_consider"]:
            print("can't find requested tag: ",params["tag_to_consider"])
            print(":/ trying again...")
            continue
        #tag.transform_tag_frame(params["base_frame"])
        params["tag_pub"].publish(tag.get_marker_msg())
        bridge=CvBridge()
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        to_press=buttonTask(image,tag,params)
        params["to_press"]=to_press
        q=tf.transformations.quaternion_from_euler(tag.R[0],tag.R[1],tag.R[2])
        params["button_q"]= q
        
    
        return 0
    return -1

    
    



def main():
    rospy.init_node('remote_tag_detection', anonymous=True)


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
    if not cv2_4_7:
        detectorParams = cv2.aruco.DetectorParameters_create()
        detectorParams.markerBorderBits = rospy.get_param("border_thickness",1)
        #thresholding parameters
        detectorParams.adaptiveThreshWinSizeMin=50
        detectorParams.adaptiveThreshWinSizeMax=200
        detectorParams.adaptiveThreshWinSizeStep=20
        #detectorParams.perspectiveRemovePixelPerCell=0
        #detectorParams.minMarkerPerimeterRate = 0.005
        #detectorParams.perspectiveRemoveIgnoredMarginPerCell=0.33  #spiegazione https://github.com/zsiki/Find-GCP
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        #detectorParams.maxErroneousBitsInBorderRate = 0.5
        #detectorParams.polygonalApproxAccuracyRate = 0.1



        tagDetector.set_detector_params(detectorParams)
        print("detector params set")
    else:
        detectorParams=cv2.aruco.DetectorParameters()
        detectorParams.markerBorderBits = rospy.get_param("border_thickness",1)
        #thresholding parameters
        detectorParams.adaptiveThreshWinSizeMin=50
        detectorParams.adaptiveThreshWinSizeMax=200
        detectorParams.adaptiveThreshWinSizeStep=20
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        tagDetector.set_detector_params(detectorParams)

    print("params set")
    #publisher of pose stamped message
    tag_pub=rospy.Publisher("tag_pose",Marker, queue_size=10)
    
    print("Loading number detector...")
    gpu=rospy.get_param("~gpu",False)
    if gpu:
        print("Using GPU")
    else:
        print("Using CPU")
    numDetector = easyocr.Reader(['en'], gpu=gpu)
    print("Number detector loaded\n")

    #subscriber
    request_sub=rospy.Subscriber("button_request", Float32, lambda msg: request_callback(msg, params))
    params= {
        "base_frame":  rospy.get_param("~target_frame","base_link"),
        "tag_pub": tag_pub,
        "tagDetector": tagDetector,
        "desired_sum": rospy.get_param("~desired_sum",0),
        "numDetector": numDetector,
        "numbers_found": np.zeros(N_BUTTONS),
        "verbosity": verbosity,
        "button_pub": rospy.Publisher("/button_to_press",PoseStamped, queue_size=10),
        "to_press_idx": 0,
        "button_q": None,
        "should_detect": True,
        "tag_to_consider": int(rospy.get_param("~tag_to_consider",0)),
    }

    #rospy.Subscriber(image_topic, Image, lambda msg: callback(msg, params))


    #when the user press SPACE: the first image received get processed
    found=False
    try:
        while(not found):
            command=input("Press enter to process the next image, or space for the secret move ;)")
            if command==" ":
                print("secret move activated")
                #TODO do secret move
                path=os.path.dirname(os.path.realpath(__file__))
                path=os.path.dirname(path)
                path=os.path.dirname(path)
                path=path+"/files/button_numbers.txt"
                #read numbers from file
                with open(path, 'r') as fp:
                    button_numbers = fp.read()
                button_numbers=[int(num) for num in button_numbers.split(",")]
                params["numbers_found"]=button_numbers
                found=True
            #if command is q exit
            elif command=="q":
                return
            print("processing image")
            use_ros=True
            retries=0
            max_retries=10
            if use_ros:
                while(retries<max_retries):
                    print("waiting for image on topic: ",image_topic)
                    #wait for a ros image on the topic
                    ros_img=rospy.wait_for_message(image_topic, Image, timeout=rospy.Duration(5))
                    res=callback(ros_img,params)
                    if res==-1:
                        print("could not find requested tag")
                        retries+=1
                
                    time.sleep(0.5)
                print("could not find requested tag after {} retries".format(max_retries))
            else:
                print("Getting image from file")
                #read cv image from folder
                path =os.path.dirname(os.path.realpath(__file__))
                path=os.path.dirname(path)
                path=os.path.dirname(path)
                path=path+"/figures/panel_test"
                img_folder= path
                print("getting image from folder: ",img_folder)
                img_name="image0.png"
                img_path=os.path.join(img_folder,img_name)
                image=cv2.imread(img_path)
                #convert to ros image
                bridge=CvBridge()
                ros_img=bridge.cv2_to_imgmsg(image, encoding="bgr8")
                callback(ros_img,params)
    except KeyboardInterrupt:
        #on ctrl c exit
        return

        
        

    rospy.spin()


if __name__=='__main__':
    main()
