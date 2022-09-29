#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import sys
import cv2

def send_image():
    bridge = CvBridge()

    cam = cv2.VideoCapture(2)

    image_pub = rospy.Publisher("image_pub/compressed", CompressedImage, queue_size=1000)
    print("Publisher aperto")

    #rospy.spin()
    while(True):
        result, image = cam.read()

        if not result:
            print("No image detected")

        #cv2.imwrite("prova.png", image)
        #print("immagine stampata")


        image_msg = bridge.cv2_to_compressed_imgmsg(image)

        image_pub.publish(image_msg)

def main(args):
    rospy.init_node("image_sender")
    print("creo nodo")
    """     
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down") 
    """
    print("parte send_image")
    send_image()

if __name__ == '__main__':
    main(sys.argv)


    