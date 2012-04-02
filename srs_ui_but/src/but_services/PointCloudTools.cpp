/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 18/02/2012
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

#include <but_services/PointCloudTools.h>

namespace but_services
{

PointCloudTools::PointCloudTools()
{
  tfListener = new tf::TransformListener();

  transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud, *tfListener, "/map", 1);

  point_cloud.subscribe(threaded_nh_, CAMERA_TOPIC, 1);

  transform_filter->registerCallback(boost::bind(&PointCloudTools::incomingCloudCallback, this, _1));
}

void PointCloudTools::incomingCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // Store time stamp
  pointCloud_stamp = cloud->header.stamp;

  // Transfotm PointCloud2 to PCL PointCloud
  pcl::fromROSMsg(*cloud, pcl_pointCloud);
}

srs_ui_but::ClosestPoint PointCloudTools::getClosestPoint(std::string link)
{
  closestPoint.time_stamp = pointCloud_stamp;
  closestPoint.status = false;

  try
  {
    tfListener->waitForTransform(link, CAMERA_LINK, pointCloud_stamp, ros::Duration(0.2));
    tfListener->lookupTransform(link, CAMERA_LINK, pointCloud_stamp, sensorToLinkTf);
    tfListener->waitForTransform(CAMERA_LINK, link, pointCloud_stamp, ros::Duration(0.2));
    tfListener->lookupTransform(CAMERA_LINK, link, pointCloud_stamp, linkToSensorTf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Transform ERROR");
    return closestPoint;
  }
  transformer.setTransform(linkToSensorTf);

  // Transform link to camera
  tf::Stamped<btVector3> p;
  p.setX(0);
  p.setY(0);
  p.setZ(0);
  p.frame_id_ = link;
  transformer.transformPoint(CAMERA_LINK, p, p);

  float d;
  closestPoint.distance = FLT_MAX;

  // Get closest point and distance
  BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_pointCloud.points)
{  d = pow((pt.x - p.getX()), 2) + pow((pt.y - p.getY()), 2) + pow((pt.z - p.getZ()), 2);

  if (d < closestPoint.distance)
  {
    closestPoint.distance = d;
    closestPoint.position.x = pt.x;
    closestPoint.position.y = pt.y;
    closestPoint.position.z = pt.z;
  }
}

// Transform closest point back to link
transformer.setTransform(sensorToLinkTf);
p.setX(closestPoint.position.x);
p.setY(closestPoint.position.y);
p.setZ(closestPoint.position.z);
p.frame_id_ = CAMERA_LINK;
transformer.transformPoint(link, p, p);

closestPoint.position.x = p.getX();
closestPoint.position.y = p.getY();
closestPoint.position.z = p.getZ();
closestPoint.distance = sqrt(closestPoint.distance);
closestPoint.status = true;

return closestPoint;
}
}
