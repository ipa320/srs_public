/******************************************************************************
 * \file
 *
 * $Id:
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 16.4.2012
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

#include "but_context/but_context_server.h"

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "but_context_server");

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  but_context::contextServer = new but_context::ContextServer();

  // Create and advertise this services over ROS
  ros::ServiceServer setContextService = n.advertiseService(BUT_SetContext_SRV, but_context::setContext);
  ros::ServiceServer getContextService = n.advertiseService(BUT_GetContext_SRV, but_context::getContext);

  ROS_INFO("BUT Context Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}

