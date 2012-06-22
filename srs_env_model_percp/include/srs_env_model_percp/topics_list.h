/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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
#ifndef BUT_PLANE_DETECTOR_TOPICS_LIST_H
#define BUT_PLANE_DETECTOR_TOPICS_LIST_H

#include "services_list.h"

namespace srs_env_model_percp
{
    /**
      * but_plane_detector - published topics
      */
	static const std::string DET_NODE_NAME = "but_plane_detector";
	static const std::string DET_OUTPUT_POINT_CLOUD_TOPIC = PLANE_DETECTOR_PREFIX + std::string("/but_env_model/point_cloud");
	static const std::string DET_OUTPUT_PLANES_TOPIC = PLANE_DETECTOR_PREFIX + std::string("/but_env_model/plane_array");

    /**
      * but_plane_detector - parameters
      */
	static const std::string DET_INPUT_POINT_CLOUD_TOPIC = "/cam3d/depth/points";
	static const std::string DET_INPUT_IMAGE_TOPIC = "/cam3d/depth/image_raw";
	static const std::string DET_INPUT_CAM_INFO_TOPIC = "/cam3d/depth/camera_info";

    /**
      * but_plane_detector - env. model
      */
	static const std::string DET_SERVICE_INSERT_PLANES = "/but_env_model/insert_planes";
	static const std::string DET_SERVICE_INSERT_PLANE = "/but_env_model/insert_plane";

    /**
      * but_plane_detector - target frames
      * TODO: make this automatic
      */
	static const std::string DET_OUTPUT_PLANE_FRAMEID = "/map";
	static const std::string DET_OUTPUT_PLANE_ORIGINAL_FRAMEID = "/head_cam3d_link";


    /**
      * but_seg_utils - kinect to pcl converter topics
      */
	static const std::string KIN2PCL_NODE_NAME 					= "but_kin2pcl_node";
	static const std::string KIN2PCL_INPUT_IMAGE_TOPIC 			= "/cam3d/depth/image_raw";
	static const std::string KIN2PCL_INPUT_CAM_INFO_TOPIC 		= "/cam3d/depth/camera_info";
	static const std::string KIN2PCL_OUTPUT_POINT_CLOUD_TOPIC 	= PACKAGE_NAME_PREFIX + std::string("/point_cloud");
	static const std::string KIN2PCL_OUTPUT_POINT_CLOUD_FRAMEID = "/openni_depth_frame";

    /**
      * but_seg_utils - PCD exporter topics
      */
	static const std::string PCDEXP_NODE_NAME 				   = "but_pcd_exporter_node";
	static const std::string PCDEXP_INPUT_IMAGE_TOPIC 		   = "/cam3d/depth/image_raw";
	static const std::string PCDEXP_INPUT_CAM_INFO_TOPIC 	   = "/cam3d/depth/camera_info";
	static const std::string PCDEXP_OUTPUT_POINT_CLOUD_TOPIC   = PACKAGE_NAME_PREFIX + std::string("/point_cloud");
	static const std::string PCDEXP_OUTPUT_POINT_CLOUD_FRAMEID = "/openni_depth_frame";

    /**
      * but_seg_utils - depth map segmenter topics
      */
	static const std::string SEG_NODE_NAME 					  = "but_segmenter_node";
	static const std::string SEG_INPUT_IMAGE_TOPIC 			  = "/cam3d/depth/image_raw";
	static const std::string SEG_INPUT_CAM_INFO_TOPIC 		  = "/cam3d/depth/camera_info";
	static const std::string SEG_OUTPUT_REGION_INFO_TOPIC 	  = PACKAGE_NAME_PREFIX + std::string("/but_env_model/seg_region_image");
	static const std::string SEG_OUTPUT_DEVIATION_IMAGE_TOPIC = PACKAGE_NAME_PREFIX + std::string("/but_env_model/seg_deviation_image");


    /**
      * bb_estimator - default topics to subscribe from: variant #1
      */
	const std::string sv1_depthTopicDefault("/cam3d/depth/image_raw");
	const std::string sv1_camInfoTopicDefault("/cam3d/depth/camera_info");
	const std::string sv1_rgbTopicDefault("/cam3d/rgb/image_raw");

    /**
      * bb_estimator - default topics to subscribe from: variant #2
      */
	const std::string sv2_pointCloudTopicDefault("/cam3d/depth/points");
	const std::string sv2_camInfoTopicDefault("/cam3d/camera_info");
	const std::string sv2_rgbTopicDefault("/cam3d/rgb/image_raw");

    /**
      * bb_estimator - default scene (world) frame id
      */
	const std::string sceneFrameIdDefault("/map");
}

#endif //BUT_PLANE_DETECTOR_TOPICS_LIST_H

