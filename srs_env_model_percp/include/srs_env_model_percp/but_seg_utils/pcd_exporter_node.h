/******************************************************************************
 * \file
 *
 * $Id: exporter.cpp 619 2012-04-16 13:47:28Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
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

/**
 * Description:
 * Module exports depth map images into files
 * Output files are marked as model_NUM.pcd
 *
 */

#pragma once
#ifndef BUT_SEG_UTILS_PCD_EXPORTER_H
#define BUT_SEG_UTILS_PCD_EXPORTER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/io/pcd_io.h>

#include <srs_env_model_percp/topics_list.h>

namespace srs_env_model_percp
{
	static const std::string NODE_NAME = PCDEXP_NODE_NAME;
	static const std::string INPUT_IMAGE_TOPIC = PCDEXP_INPUT_IMAGE_TOPIC;
	static const std::string INPUT_CAM_INFO_TOPIC = PCDEXP_INPUT_CAM_INFO_TOPIC;
	static const std::string OUTPUT_POINT_CLOUD_TOPIC = PCDEXP_OUTPUT_POINT_CLOUD_TOPIC;
	static const std::string OUTPUT_POINT_CLOUD_FRAMEID = PCDEXP_OUTPUT_POINT_CLOUD_FRAMEID;

	sensor_msgs::PointCloud2 cloud_msg;
	ros::Publisher pub;
	int modelNo = 0;

	void callback( const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& cam_info);
}

#endif //BUT_SEG_UTILS_PCD_EXPORTER_H
