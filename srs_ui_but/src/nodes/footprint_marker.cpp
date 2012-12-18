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

#include "footprint_marker.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


namespace srs_ui_but
{

const double DEFAULT_LIFETIME      = 5.0;
const double DEFAULT_Z_POSITION    = 0.11;

const std::string POLYGON_TOPIC    = "polygon_in";
const std::string MARKER_TOPIC     = "marker_out";


FootprintMarker::FootprintMarker()
{
    // a handle for this node, initialize node
    nh_ = ros::NodeHandle("~");

    // read parameters from parameter server
//    nh_.param("marker_frame", base_frame_, std::string("/base_link"));
    nh_.param("marker_lifetime", lifetime_, DEFAULT_LIFETIME);
    nh_.param("z_pos", z_pos_, DEFAULT_Z_POSITION);

    // Create the publisher
    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 10);

    // Subscribe to the polygon topic
    polygon_sub_ = nh.subscribe<geometry_msgs::PolygonStamped>(POLYGON_TOPIC, 10, &FootprintMarker::polygonCallback, this);

    // Create the marker template
    // Message template
    marker_.header.frame_id = "/base_link";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = "cob_footprint_marker";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::LINE_LIST;
//    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.lifetime = ros::Duration(lifetime_);
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;
    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = z_pos_;
    marker_.scale.x = 0.01;
    marker_.color.r = 0.0;
    marker_.color.g = 0.0;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;

    // Create the disc like geometry for the markers
    geometry_msgs::Point v;
    v.x = v.y = v.z = 0;
    marker_.points.push_back(v);
    marker_.points.push_back(v);

    ROS_INFO("Footprint marker publisher initialized.");
}


FootprintMarker::~FootprintMarker()
{
}

void FootprintMarker::polygonCallback(const geometry_msgs::PolygonStampedConstPtr& polygon)
{
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = polygon->header.frame_id;
    marker_.points.clear();

    // Line list...
    for( std::size_t i = 0; i < polygon->polygon.points.size(); ++i )
    {
        std::size_t i2 = (i + 1) % polygon->polygon.points.size();

        geometry_msgs::Point v1, v2;
        v1.x = polygon->polygon.points[i].x;
        v1.y = polygon->polygon.points[i].y;
        v1.z = polygon->polygon.points[i].z;
        v2.x = polygon->polygon.points[i2].x;
        v2.y = polygon->polygon.points[i2].y;
        v2.z = polygon->polygon.points[i2].z;

        marker_.points.push_back(v1);
        marker_.points.push_back(v2);
    }

    ROS_INFO_ONCE("First footprint polygon received, publishing the marker...");

	marker_pub_.publish(marker_);
}


}

