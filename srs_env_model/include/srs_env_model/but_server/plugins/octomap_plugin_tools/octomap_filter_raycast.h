/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
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
#ifndef OCTOMAP_FILTER_RAYCAST_H_
#define OCTOMAP_FILTER_RAYCAST_H_

#include "octomap_filter_base.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

namespace srs_env_model
{

class COcFilterRaycast : public COcTreeFilterBase
{
public:
	//! Constructor
	COcFilterRaycast(const std::string & octree_frame_id, ERunMode mode = FILTER_ALLWAYS);

	//! Initialize. Must be called before first filtering
	virtual void init(ros::NodeHandle & node_handle);

	//! Configure filter before each frame. Set input cloud.
	void setCloud(const tPointCloud * cloud);

	//! Write some info about last filter run
	virtual void writeLastRunInfo();

protected:
	//! Filtering function implementation
	virtual void filterInternal( tButServerOcTree & tree );

	/// Camera info callback
	void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

	/// Compute sensor origin from the header info
	octomap::point3d getSensorOrigin(const std_msgs::Header& sensor_header);

	/// Is point in sensor cone?
	bool inSensorCone(const cv::Point2d& uv) const;

	/// Return true, if occupied cell is between origin and p
	bool isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p, double resolution, tButServerOcTree & tree) const;

	//! Compute boundig box
	void computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max);

protected:
	//! Initialized
	bool m_bFilterInitialized;

	//! Transform listener
	tf::TransformListener m_tfListener;

	/// Camera offsets
	int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

	/// Camera size
	cv::Size m_camera_size;

	/// Camera model
	image_geometry::PinholeCameraModel m_camera_model;

	/// Is camera model initialized?
	bool m_bCamModelInitialized;

	/// Camera info locking
	boost::mutex m_lockCamera;

	/// Camera info topic name
	std::string m_camera_info_topic;

	/// Camera info subscriber
	ros::Subscriber m_ciSubscriber;

	/// Point cloud
	const tPointCloud * m_cloudPtr;

	/// Number of removed leafs
	long m_numLeafsRemoved;

}; // class COcFilterRaycast

} // namespace srs_env_model

#endif /* OCTOMAP_FILTER_RAYCAST_H_ */
