/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 10/12/2012
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

#pragma once
#ifndef COB_FOOTPRINT_MARKER_H
#define COB_FOOTPRINT_MARKER_H

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

namespace srs_ui_but
{

///
/// @class FootprintMarker
/// @brief visualizes polygon representing robot's footprint
///
class FootprintMarker
{
public:
    ///
    /// @brief  Constructor
    ///
    FootprintMarker();

    ///
    /// @brief  Destructor
    ///
    ~FootprintMarker();

protected:
    ///
    /// @brief  Creates the marker, the method is called from the constructor, etc.
    ///
	void polygonCallback(const geometry_msgs::PolygonStampedConstPtr& polygon);

protected:
    // Footprint marker
    visualization_msgs::Marker marker_;

    // a handle for this node
    ros::NodeHandle nh_;

    // Marker publisher
    ros::Publisher marker_pub_;

    // Robot base frame
//    std::string base_frame_;

    // Marker lifetime
    double lifetime_;

    // Marker z-position
    double z_pos_;

    // Footprint polygon message subscriber
    ros::Subscriber polygon_sub_;
};


}

#endif // COB_FOOTPRINT_MARKER_H

