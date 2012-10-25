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
 * Date: 25/1/2012
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

#include <srs_env_model/but_server/registration/pcl_registration_module.h>
#include <srs_env_model/topics_list.h>

/**
 * Constructor
 */
srs_env_model::COcToPcl::COcToPcl()
: m_bTransformCamera( false )
, m_bSpinThread( true )
{

}

/**
 * Initialize module - called in server constructor
 */
void srs_env_model::COcToPcl::init(ros::NodeHandle & node_handle)
{
	ROS_DEBUG("Initializing CCompressedPointCloudPlugin");

	if ( m_bSpinThread )
	{
		// if we're spinning our own thread, we'll also need our own callback queue
		node_handle.setCallbackQueue( &callback_queue_ );

		need_to_terminate_ = false;
		spin_thread_.reset( new boost::thread(boost::bind(&COcToPcl::spinThread, this)) );
		node_handle_ = node_handle;
	}

	// Read parameters
	{
		// Where to get camera position information
		node_handle.param("camera_info_topic_name", m_cameraInfoTopic, CPC_CAMERA_INFO_PUBLISHER_NAME );
	}

	// Subscribe to position topic
	// Create subscriber
	m_camPosSubscriber = node_handle.subscribe<sensor_msgs::CameraInfo>( m_cameraInfoTopic, 10, &srs_env_model::COcToPcl::onCameraChangedCB, this );

	if (!m_camPosSubscriber)
	{
		ROS_ERROR("Not subscribed...");
		ROS_ERROR( "Cannot subscribe to the camera position publisher..." );
	}

	// stereo cam params for sensor cone:
	node_handle.param<int> ("compressed_pc_camera_stereo_offset_left", m_camera_stereo_offset_left, 0); // 128
	node_handle.param<int> ("compressed_pc_camera_stereo_offset_right", m_camera_stereo_offset_right, 0);
}

/**
 * On camera position changed callback
 */
void srs_env_model::COcToPcl::onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{

}

/**
 * Compute new frustum
 */
void srs_env_model::COcToPcl::updateFrustum()
{

}

/**
 * Main loop when spinning our own thread - process callbacks in our callback queue - process pending goals
 */
void srs_env_model::COcToPcl::spinThread()
{

}
