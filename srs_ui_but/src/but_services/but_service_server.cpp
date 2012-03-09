/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 18.2.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include <but_services/but_service_server.h>

namespace but_services
{

bool getClosestPoint(GetClosestPoint::Request &req, GetClosestPoint::Response &res)
{
  ROS_INFO("Getting closest point");

  res.closest_point_data = pcTools->getClosestPoint(req.link);

  if (!res.closest_point_data.status)
  {
    ROS_WARN("Cannot get closest point!");
    return false;
  }

  ROS_INFO("..... DONE");
  return true;
}

}

/*
 * Main function
 */
int main(int argc, char **argv)
{

  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "but_service_server");

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Point Cloud data handler
  pcTools = new but_services::PointCloudTools();

  // Create and advertise this service over ROS
  ros::ServiceServer getClosestPointService = n.advertiseService(BUT_GetClosestPoint_SRV, but_services::getClosestPoint);

  ROS_INFO("BUT Service Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}
