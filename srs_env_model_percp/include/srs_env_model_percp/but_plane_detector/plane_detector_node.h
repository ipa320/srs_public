/******************************************************************************
 * \file
 *
 * $Id: detector.cpp 777 2012-05-11 11:23:17Z ihulik $
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
 * The testing framework for future plane detection using Hough Transform
 *
 * Subscribes cam_info and point cloud messages from COB
 * Tries to detect planes in image and sends them via interactive markers server to display
 *
 */
#pragma once
#ifndef BUT_PLANE_DETECTOR_NODE_H
#define BUT_PLANE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <float.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/quaternion.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <srs_env_model_percp/ClearPlanes.h>

#include <srs_env_model_percp/but_seg_utils/filtering.h>
#include <srs_env_model_percp/but_seg_utils/normals.h>
#include <srs_env_model_percp/but_plane_detector/scene_model.h>
#include <srs_env_model_percp/but_plane_detector/dyn_model_exporter.h>

#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>


namespace srs_env_model_percp
{
	static const std::string NODE_NAME = DET_NODE_NAME;
	static const std::string INPUT_POINT_CLOUD_TOPIC = DET_INPUT_POINT_CLOUD_TOPIC;
	static const std::string OUTPUT_POINT_CLOUD_TOPIC = DET_OUTPUT_POINT_CLOUD_TOPIC;
	static const std::string INPUT_IMAGE_TOPIC = DET_INPUT_IMAGE_TOPIC;
	static const std::string INPUT_CAM_INFO_TOPIC = DET_INPUT_CAM_INFO_TOPIC;

	static const std::string SERVICE_CLEAR_PLANES = DET_SERVICE_CLEAR_PLANES;

	//static const std::string OUTPUT_POINT_CLOUD_TOPIC = "/point_cloud";
	//static const std::string OUTPUT_POINT_CLOUD_FRAMEID = "/openni_depth_frame";


	std::string target_topic = "/map";
	int counter = 0;
	sensor_msgs::PointCloud2 cloud_msg;
	tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;
	tf::TransformListener *tfListener;
	ros::Publisher pub1;
	ros::Publisher pub2;
	DynModelExporter *exporter = NULL;

	sensor_msgs::CameraInfo cam_info_legacy;
	sensor_msgs::CameraInfoConstPtr cam_info_aux (&cam_info_legacy);
	SceneModel *model;

	/**
	 * Callback function manages sync of messages
	 */
	void callbackpcl(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& cam_info);
	bool clear(srs_env_model_percp::ClearPlanes::Request &req,srs_env_model_percp::ClearPlanes::Response &res);
}

int main( int argc, char** argv );

#endif //BUT_PLANE_DETECTOR_NODE_H
