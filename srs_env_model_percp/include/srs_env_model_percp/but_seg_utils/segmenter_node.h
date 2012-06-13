/******************************************************************************
 * \file
 *
 * $Id: segmenter.cpp 619 2012-04-16 13:47:28Z ihulik $
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
 * A module gets depth map data and segments them
 * For use please start with -h param
 *
 */

#pragma once
#ifndef BUT_SEG_UTILS_SEGMENTER_NODE_H
#define BUT_SEG_UTILS_SEGMENTER_NODE_H

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// this
#include <srs_env_model_percp/but_seg_utils/filtering.h>
#include <srs_env_model_percp/but_seg_utils/normals.h>
#include <srs_env_model_percp/topics_list.h>

namespace srs_env_model_percp
{
	#define REGIONS_DEPTH 		WatershedType::DepthDiff
	#define REGIONS_NORMAL 		WatershedType::NormalDiff
	#define REGIONS_COMBINED 	WatershedType::Combined
	#define REGIONS_PREDICTOR 	WatershedType::PredictorDiff
	#define REGIONS_TILE	 	16
	#define DEFAULT_REGIONS 	REGIONS_COMBINED
	#define DEFAULT_MAXDEPTH 	3000

	static const std::string NODE_NAME = SEG_NODE_NAME;
	static const std::string INPUT_IMAGE_TOPIC = SEG_INPUT_IMAGE_TOPIC;
	static const std::string INPUT_CAM_INFO_TOPIC = SEG_INPUT_CAM_INFO_TOPIC;
	static const std::string OUTPUT_REGION_INFO_TOPIC = SEG_OUTPUT_REGION_INFO_TOPIC;
	static const std::string OUTPUT_DEVIATION_IMAGE_TOPIC = SEG_OUTPUT_DEVIATION_IMAGE_TOPIC;

	ros::Publisher n_pub;
	ros::Publisher region_image;
	ros::Publisher deviation_image;
	ros::Publisher triangle_pub;

	int typeRegions = DEFAULT_REGIONS;
	unsigned short maxDepth = DEFAULT_MAXDEPTH;

	bool initParams( int argc, char** argv );
	void callback( const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& cam_info);
}

#endif // BUT_SEG_UTILS_SEGMENTER_NODE_H
