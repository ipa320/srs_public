/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 18.2.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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
