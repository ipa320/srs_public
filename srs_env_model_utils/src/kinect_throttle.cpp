/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Modified to throttle all Kinect data by Michal Spanel, Robo@FIT, BUT
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>


namespace srs_env_model_utils
{

typedef sensor_msgs::PointCloud2 tPointCloud;
typedef sensor_msgs::Image tImage;
typedef sensor_msgs::CameraInfo tCameraInfo;
//typedef message_filters::sync_policies::ExactTime<tPointCloud, tImage, tCameraInfo, tImage, tPointCloud> tSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<tPointCloud, tImage, tCameraInfo, tImage, tPointCloud> tSyncPolicy;

//const int QUEUE_SIZE = 10;
const int QUEUE_SIZE = 1;

class KinectThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  KinectThrottle(): max_update_rate_(0), sync_(tSyncPolicy(10))
  {
  }

private:
  ros::Time last_update_;
  double max_update_rate_;
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("max_rate", max_update_rate_);

    rgb_cloud_pub_ = nh.advertise<tPointCloud>("rgb_cloud_out", QUEUE_SIZE);
    rgb_image_pub_ = nh.advertise<tImage>("rgb_image_out", QUEUE_SIZE);
    rgb_caminfo_pub_ = nh.advertise<tCameraInfo>("rgb_caminfo_out", QUEUE_SIZE);
    depth_image_pub_ = nh.advertise<tImage>("depth_image_out", QUEUE_SIZE);
    cloud_pub_ = nh.advertise<tPointCloud>("cloud_out", QUEUE_SIZE);

    rgb_cloud_sub_.subscribe(nh, "rgb_cloud_in", QUEUE_SIZE);
    rgb_image_sub_.subscribe(nh, "rgb_image_in", QUEUE_SIZE);
    rgb_caminfo_sub_.subscribe(nh, "rgb_caminfo_in", QUEUE_SIZE);
    depth_image_sub_.subscribe(nh, "depth_image_in", QUEUE_SIZE);
    cloud_sub_.subscribe(nh, "cloud_in", QUEUE_SIZE);

    sync_.connectInput(rgb_cloud_sub_, rgb_image_sub_, rgb_caminfo_sub_, depth_image_sub_, cloud_sub_);
    sync_.registerCallback(boost::bind(&srs_env_model_utils::KinectThrottle::callback, this, _1, _2, _3, _4, _5));
  }

  void callback(const tPointCloud::ConstPtr& rgb_cloud, 
                const tImage::ConstPtr& rgb_image, 
                const tCameraInfo::ConstPtr& rgb_caminfo,
                const tImage::ConstPtr& depth_image, 
                const tPointCloud::ConstPtr& cloud
                )
  {
    if (max_update_rate_ > 0.0)
    {
      NODELET_DEBUG("update set to %f", max_update_rate_);
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }
    else
      NODELET_DEBUG("update_rate unset continuing");

    last_update_ = ros::Time::now();

    rgb_cloud_pub_.publish(rgb_cloud);
    rgb_image_pub_.publish(rgb_image);
    rgb_caminfo_pub_.publish(rgb_caminfo);
    depth_image_pub_.publish(depth_image);
    cloud_pub_.publish(cloud);
  }

  ros::Publisher rgb_cloud_pub_, rgb_image_pub_, rgb_caminfo_pub_, depth_image_pub_, cloud_pub_;

  message_filters::Subscriber<tPointCloud> rgb_cloud_sub_;
  message_filters::Subscriber<tImage> rgb_image_sub_;
  message_filters::Subscriber<tCameraInfo> rgb_caminfo_sub_;
  message_filters::Subscriber<tImage> depth_image_sub_;
  message_filters::Subscriber<tPointCloud> cloud_sub_;

  message_filters::Synchronizer<tSyncPolicy> sync_;
};


PLUGINLIB_DECLARE_CLASS(srs_env_model_utils, KinectThrottle, srs_env_model_utils::KinectThrottle, nodelet::Nodelet);
}

