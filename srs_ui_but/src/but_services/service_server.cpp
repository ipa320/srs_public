/******************************************************************************
 * \file
 *
 * $Id: but_service_server.cpp 556 2012-04-11 16:10:40Z xlokaj03 $
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

#include <srs_ui_but/but_services/service_server.h>

namespace srs_ui_but
{

// Point cloud data handler
srs_ui_but::PointCloudTools * pcTools;

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
  srs_ui_but::pcTools = new srs_ui_but::PointCloudTools();

  // Create and advertise this service over ROS
  ros::ServiceServer getClosestPointService = n.advertiseService(srs_ui_but::GetClosestPoint_SRV, srs_ui_but::getClosestPoint);

  ROS_INFO("BUT Service Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}
