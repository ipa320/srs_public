/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 14.2.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#ifndef POINTCLOUDTOOLS_H_
#define POINTCLOUDTOOLS_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>
#include <float.h>
#include <srs_ui_but/ClosestPoint.h>

#define CAMERA_TOPIC "/cam3d/depth/points"
#define CAMERA_LINK "/head_cam3d_link"

namespace but_services
{

class PointCloudTools
{
public:
  /*
   * Constructor
   */
  PointCloudTools();
  /*
   * Destructor
   */
  virtual ~PointCloudTools()
  {
  }

  /*
   * This function calculates closest point from a robot link from latest point cloud
   * @param link is robot link from which we want to get the closest point
   */
  srs_ui_but::ClosestPoint getClosestPoint(std::string link);

private:
  void incomingCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

  // PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ> pcl_pointCloud;

  // PointCloud tim stamp
  ros::Time pointCloud_stamp;

  // Closest point data
  srs_ui_but::ClosestPoint closestPoint;

  // Transformer
  tf::Transformer transformer;

  // Transformations
  tf::StampedTransform sensorToLinkTf, linkToSensorTf;

  // Transform filter
  tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;

  // Transform listener
  tf::TransformListener *tfListener;

  // Node handler
  ros::NodeHandle threaded_nh_;

  // Point cloud subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif /* POINTCLOUDTOOLS_H_ */
