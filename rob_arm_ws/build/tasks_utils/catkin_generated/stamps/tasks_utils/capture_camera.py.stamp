#ros node that subscribe to the iamge topic and capture and save and image when the user presses space
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os


class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_count = 0
        path =os.path.dirname(os.path.realpath(__file__))
        path=os.path.dirname(path)
        path=path+"/not_ros/images_from_ros"
        self.image_dir=path
        print("image dir: ", self.image_dir)
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

    def save_image(self):
        data=rospy.wait_for_message("/camera/image_raw", Image)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite(self.image_dir + f"/image{self.image_count}.png", cv_image)
        self.image_count += 1
        print("Image saved, save count: ", self.image_count)


def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    while not rospy.is_shutdown():
        key=input("Press enter to save image")

        print("Saving image")
        image_saver.save_image()


if __name__ == '__main__':
    main()