#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv){
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("cam_pub", 1);
  int cam_device;
  //condition to verify if the camera device was passed as the first
  //terminal parameter "argv[1]" or as the "cam_device" parameter in the .launch.
  //id 0 device is passed as default
  if (argc < 2)
     nh.param("cam_device", cam_device, 0);
  else{
     // Convert the passed as command line parameter index
     //for the video device to an integer
     std::istringstream cam_deviceCmd(argv[1]);
     // Check if it is indeed a number
     if(!(cam_deviceCmd >> cam_device)) return 1;
  }
  //initialize a VideoCapture variable to get camera data
  cv::VideoCapture cap(cam_device);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  int image_show;
  nh.param("image_show", image_show, 0);

  while (nh.ok()) {
     cap >> frame;
     //show the image
     if (image_show){
        cv::imshow("Cam View", frame);
        cv::waitKey(30);
     }
     //Check if grabbed frame is actually full with some content,
     //then publish the image
     if(!frame.empty()) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
     }
  }
}