"""

ROS node that using opencv open the camera and publish raw and compressed iamges 
"""

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml
class CameraSetup(object):
    def __init__(self,params):
        self.bridge = CvBridge()
        #get available device indexes
        index=0
        arr=[]
        while index<10:
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                arr.append(index)
            cap.release()
            index += 1
        print("Available devices: ",arr)
        if params["device"] not in arr:
            print("Device ,",params["device"],", not available")
            exit()
        self.cap = cv2.VideoCapture(params["device"])
        self.cap.set(3, params["width"])
        self.cap.set(4, params["height"])
        self.cap.set(5, params["fps"])
        self.cap.set(6, cv2.VideoWriter.fourcc('M','J','P','G'))
        #self.cap.set(10, params["quality"])
        self.rate = rospy.Rate(30)
        self.camera_info_path=params["camera_info_path"]
        self.camera_frame_id=params["camera_frame_id"]
        self.params=params
        
        self.image_pub = rospy.Publisher("image_raw",Image, queue_size=1)
        self.compressed_image_pub = rospy.Publisher("image_raw/compressed",CompressedImage, queue_size=1)
        self.camera_info_pub = rospy.Publisher("camera_info",CameraInfo, queue_size=1)

        self.camera_info=CameraInfo()
        self.extract_camera_info()
        self.run()

    def run(self):
        while True:
            try:

                ret, frame = self.cap.read()
                if ret:
                    try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                        self.compressed_image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
                        if self.camera_info_path!="":
                            self.camera_info.header.stamp = rospy.Time.now()
                            self.camera_info_pub.publish(self.camera_info)
                    except CvBridgeError as e:
                        print(e)
                    except Exception as e:
                        print(e)
                
            except KeyboardInterrupt:
                print("Shutting down")
                self.cap.release()
                cv2.destroyAllWindows()
                break

    def extract_camera_info(self):
        if self.camera_info_path=="":
            return
        with open(self.camera_info_path, 'r') as stream:
            try:
                #read yaml and encode in utf-8
                data = yaml.safe_load(stream)
                #data={k.encode('utf8'): v for k, v in data.items()}
                print(data)
            except yaml.YAMLError as exc:
                print(exc)

        self.camera_info.header.frame_id = self.camera_frame_id
        self.camera_info.height = self.params["height"]
        self.camera_info.width = self.params["width"]
        self.camera_info.distortion_model = data["distortion_model"]
        self.camera_info.D = data["distortion_coefficients"]["data"]
        self.camera_info.K = data["camera_matrix"]["data"]
        self.camera_info.R = data["rectification_matrix"]["data"]
        self.camera_info.P = data["projection_matrix"]["data"]

        print(self.camera_info)

        


if __name__ == '__main__':
    rospy.init_node('camera_setup')

    params={
        "width": rospy.get_param("~width", 640),
        "height": rospy.get_param("~height", 480),
        "fps": rospy.get_param("~fps", 30),
        "device": rospy.get_param("~device", 1),
        "compression": rospy.get_param("~compression", "MJPG"),
        "quality": rospy.get_param("~quality", 30),
        "camera_info_path": rospy.get_param("~camera_info_path", ""),
        "camera_frame_id": rospy.get_param("~camera_frame_id", "usb_cam"),
    }
    try:
        CameraSetup(params)
    except rospy.ROSInterruptException:
        pass

