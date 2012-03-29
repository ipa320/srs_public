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

#ifndef BUT_SERVICE_SERVER_H_
#define BUT_SERVICE_SERVER_H_

#include <ros/ros.h>
#include <but_services/PointCloudTools.h>
#include "services_list.h"

using namespace srs_ui_but;

// Point cloud data handler
but_services::PointCloudTools * pcTools;

namespace but_services
{
/*
 * Gets closest point between link and point cloud.
 * @param req is request of type GetClosestPoint
 * * @param res is response of type GetClosestPoint
 */
bool getClosestPoint(GetClosestPoint::Request &req, GetClosestPoint::Response &res);
}

#endif /* BUT_SERVICE_SERVER_H_ */
