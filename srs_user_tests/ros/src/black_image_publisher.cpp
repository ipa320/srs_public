#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "black_image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("black_image", 1);
  
  //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(img, "bgr8");
  cv_bridge::CvImage cvb;
	cvb.encoding = "8UC3";
	//cvb.header = 
  
  ros::Rate loop_rate(1);
  
  ROS_INFO("Black image publisher started.");
  
  while (nh.ok()) {
  
    cvb.image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::randn(cvb.image,20,10);
  
    sensor_msgs::Image::Ptr msg = cvb.toImageMsg();
  
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


}
