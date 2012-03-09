/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 14.2.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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
