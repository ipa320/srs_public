/******************************************************************************
 * \file
 *
 * $Id: PointCloudTools.h 556 2012-04-11 16:10:40Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 14/02/2012
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
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

/**
 * @brief This class gathers actual point cloud data and provides method which returns
 * closest point from a specified link.
 *
 * @author Tomas Lokaj
 */
class PointCloudTools
{
public:
  /**
   * @brief Constructor
   */
  PointCloudTools();
  /**
   * @brief Destructor
   */
  virtual ~PointCloudTools()
  {
  }

  /**
   * @brief This function calculates closest point from a robot link from latest point cloud.
   * @param link is robot link from which we want to get the closest point
   */
  srs_ui_but::ClosestPoint getClosestPoint(std::string link);

private:
  /**
   * @brief Callback function for handling incoming point cloud data.
   * @param cloud is incoming point cloud
   */
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
