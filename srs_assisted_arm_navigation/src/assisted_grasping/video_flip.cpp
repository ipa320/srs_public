#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
//#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <image_transport/image_transport.h>

using namespace std;

image_transport::Publisher m_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

  static bool callback_received = false;

  if (callback_received == false)
  {

    ROS_INFO("First callback received.");
    callback_received = true;

  }

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  try
  {

    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::flip(cv_ptr->image, cv_ptr->image, -1);

  try
  {

    m_pub.publish(cv_ptr->toImageMsg());

  }

  catch (image_transport::Exception& e)
  {

    ROS_ERROR("image_transport exception: %s", e.what());
    return;

  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "video_flip");

  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  m_pub = it.advertise("video_out", 1);
  image_transport::Subscriber sub_depth = it.subscribe("video_in", 1, imageCallback);

  ROS_INFO("Video flip node spinning...");

  ros::spin();

}
