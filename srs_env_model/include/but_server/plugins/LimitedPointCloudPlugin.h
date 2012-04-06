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

#ifndef LimitedPointCloudPlugin_H_included
#define LimitedPointCloudPlugin_H_included

#include "PointCloudPlugin.h"
#include <srs_env_model_msgs/RVIZCameraPosition.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>


namespace srs
{
class CLimitedPointCloudPlugin : public CPointCloudPlugin
{
public:
	/// Constructor
	CLimitedPointCloudPlugin( const std::string & name );

	/// Destructor
	virtual ~CLimitedPointCloudPlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp);

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	/// On camera position changed callback
	void onCameraPositionChangedCB(const srs_env_model_msgs::RVIZCameraPosition::ConstPtr& position);

	// main loop when spinning our own thread
	// - process callbacks in our callback queue
	// - process pending goals
	void spinThread();

protected:
	/// Should camera position and orientation be transformed?
	bool m_bTransformCamera;

	/// Camera frame id
	std::string m_cameraFrameId;

	/// Camera normalized normal - part of the plane equation
	Eigen::Vector3f m_normal, m_normalBuf;

	/// Last part of the plane equation
	float m_d, m_dBuf;

	/// Transformation from camera to the octomap frame id - rotation
	Eigen::Matrix3f m_camToOcRot;

	/// Transformation from camera to the octomap frame id - translation
	Eigen::Vector3f m_camToOcTrans;

	/// Subscriber - camera position
	// message_filters::Subscriber<srs_env_model_msgs::RVIZCameraPosition> *m_camPosSubscriber;
	ros::Subscriber m_camPosSubscriber;

	//! Message filter (we only want point cloud 2 messages)
	tf::MessageFilter<srs_env_model_msgs::RVIZCameraPosition> *m_tfCamPosSub;

	/// Counters
	long m_countVisible, m_countHidden;

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
};

/// Declare holder object - partial specialization of the default holder with predefined connection settings
template< class tpOctomapPlugin >
struct SLimitedPointCloudPluginHolder : public  srs::CCrawlingPluginHolder< srs::CLimitedPointCloudPlugin, tpOctomapPlugin >
{
protected:
	/// Define holder type
	typedef srs::CCrawlingPluginHolder< srs::CLimitedPointCloudPlugin, tpOctomapPlugin > tHolder;

public:
	/// Create holder
	SLimitedPointCloudPluginHolder( const std::string & name )
	: tHolder(  name,  tHolder::ON_START | tHolder::ON_OCCUPIED | tHolder::ON_STOP)
	{

	}

}; // struct SLimitedPointCloudPluginHolder

} // namespace srs

// LimitedPointCloudPlugin_H_included
#endif
