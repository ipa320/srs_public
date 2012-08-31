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
#ifndef CompressedPointCloudPlugin_H_included
#define CompressedPointCloudPlugin_H_included

#include "point_cloud_plugin.h"

#include <srs_env_model_msgs/RVIZCameraPosition.h>
#include <srs_env_model/but_server/server_tools.h>
#include <srs_env_model/OctomapUpdates.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <image_geometry/pinhole_camera_model.h>


namespace srs_env_model
{
class CCompressedPointCloudPlugin : public CPointCloudPlugin
{
public:
	/// Constructor
	CCompressedPointCloudPlugin( const std::string & name );

	/// Destructor
	virtual ~CCompressedPointCloudPlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapParameters & mp);

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

	//! Connect/disconnect plugin to/from all topics
	virtual void pause( bool bPause, ros::NodeHandle & node_handle);

	//! Should plugin publish data?
	virtual bool shouldPublish();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	/// On camera position changed callback
	void onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

	// main loop when spinning our own thread
	// - process callbacks in our callback queue
	// - process pending goals
	void spinThread();

	//! Test if point is in camera cone
	bool inSensorCone(const cv::Point2d& uv) const;

protected:
	/// Should camera position and orientation be transformed?
	bool m_bTransformCamera;

	/// Camera frame id
	std::string m_cameraFrameId;

    // Camera position topic name
    std::string m_cameraInfoTopic;


	/// Subscriber - camera position
	// message_filters::Subscriber<srs_env_model_msgs::RVIZCameraPosition> *m_camPosSubscriber;
	ros::Subscriber m_camPosSubscriber;

	/// Publisher - packed info
	ros::Publisher m_ocUpdatePublisher;

	/// Packed info publisher name
	std::string m_ocUpdatePublisherName;

	//! Message filter (we only want point cloud 2 messages)
	tf::MessageFilter<srs_env_model_msgs::RVIZCameraPosition> *m_tfCamPosSub;

	/// Counters
	long m_countVisible, m_countAll;

	float min, max;

	//! Spin out own input callback thread
	bool m_bSpinThread;

	// these are needed when spinning up a dedicated thread
	boost::scoped_ptr<boost::thread> spin_thread_;
	ros::NodeHandle node_handle_;
	ros::CallbackQueue callback_queue_;
	volatile bool need_to_terminate_;

	// Mutex used to lock camera position parameters
	boost::recursive_mutex m_camPosMutex;

	/// Camera model
	image_geometry::PinholeCameraModel m_camera_model, m_camera_model_buffer;

	//! Transform listener
	tf::TransformListener m_tfListener;

	/// Transform from pointcloud to camera space
	tf::Transform m_to_sensor;

	/// Camera offsets
	int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

	/// Camera size
	cv::Size m_camera_size, m_camera_size_buffer;

	//! Camera info buffer
	sensor_msgs::CameraInfo m_camera_info_buffer;

	/// Is camera model initialized?
	bool m_bCamModelInitialized;

	//! Pubilishing counter
	long m_frame_counter;

	//! Every n-th frame should be complete
	long m_uncomplete_frames;

	//! Should be complete frame published?
	bool m_bPublishComplete;

	//! Create packed info message?
	bool m_bCreatePackedInfoMsg;

	//! Packed info message data
	srs_env_model::OctomapUpdatesPtr m_octomap_updates_msg;
};

/// Declare holder object - partial specialization of the default holder with predefined connection settings
template< class tpOctomapPlugin >
struct SCompressedPointCloudPluginHolder : public  CCrawlingPluginHolder< CCompressedPointCloudPlugin, tpOctomapPlugin >
{
protected:
	/// Define holder type
	typedef CCrawlingPluginHolder< CCompressedPointCloudPlugin, tpOctomapPlugin > tHolder;

public:
	/// Create holder
	SCompressedPointCloudPluginHolder( const std::string & name )
	: tHolder(  name,  tHolder::ON_START | tHolder::ON_OCCUPIED | tHolder::ON_STOP)
	{

	}

}; // struct SLimitedPointCloudPluginHolder

} // namespace srs_env_model

// CompressedPointCloudPlugin_H_included
#endif

