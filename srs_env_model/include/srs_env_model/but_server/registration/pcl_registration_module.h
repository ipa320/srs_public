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

#pragma once
#ifndef pcl_registration_module_H_included
#define pcl_registration_module_H_included

#include <srs_env_model/but_server/server_tools.h>
#include <srs_env_model/OctomapUpdates.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <LinearMath/btMatrix3x3.h>

namespace srs_env_model
{
enum EPclRegistrationMode
{
	PCL_REGISTRATION_MODE_NONE = 0,
	PCL_REGISTRATION_MODE_ICP = 1,
	PCL_REGISTRATION_MODE_ICPNL = 2,
	PCL_REGISTRATION_MODE_SCA = 3
};

template <typename PointSource, typename PointTarget, typename Scalar = float>
class CPclRegistration
{
public:
	typedef typename pcl::Registration<PointSource, PointTarget> tRegistration;
	typedef typename pcl::Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
	typedef typename PointCloudSource::Ptr 	PointSourcePtr;
	typedef typename PointCloudSource::ConstPtr 	PointSourceConstPtr;

	typedef typename pcl::Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;
	typedef typename PointCloudTarget::Ptr 	PointTargetPtr;
	typedef typename PointCloudTarget::ConstPtr 	PointTargetConstPtr;

	// String mode names
	static const std::string m_mode_names[];
public:

	//! Constructor
	CPclRegistration() : m_mode(PCL_REGISTRATION_MODE_NONE), m_registrationPtr(0) { }

	//! Set used mode
	void setMode( EPclRegistrationMode mode );

	//! Get mode
	EPclRegistrationMode getMode() { return m_mode; }

	//! Is registration used
	bool isRegistering() { return m_mode != PCL_REGISTRATION_MODE_NONE; }

	//! Get output transform
	Eigen::Matrix4f getTransform()
	{
		if( m_registrationPtr != 0 )
			return m_registrationPtr->getFinalTransformation();

		return Eigen::Matrix4f();
	}

	//! Process data
	//! @param source Source point cloud - this should be aligned to the target cloud
	//! @param target Target point cloud - to this cloud should be source cloud aligned
	//! @param output Output point cloud
	bool process( PointSourcePtr & source, PointTargetPtr & target, PointSourcePtr & output );

	//! Get registration algorithm pointer
	template< class tpRegistration >
	tpRegistration * getRegistrationAlgorithmPtr()
	{
		return m_registrationPtr;
	}

	//! Initialize parameters from the parameter server
	//! @param node_handle Node handle
	void init( ros::NodeHandle & node_handle );

	//! Get registration mode as a string
	std::string getStrMode() { return m_mode_names[ m_mode ]; }

	//! Reinitialize registration parameters
	void resetParameters();

protected:
	//! Convert string to the mode
	//! @param name Mode name
	EPclRegistrationMode modeFromString( const std::string & name );

	//! Set common parameters
	void setRegistrationParameters();

	//! Set SCA parameters
	void setSCAParameters();

	bool inSensorCone(const cv::Point2d& uv);

protected:
	//! Used mode
	EPclRegistrationMode m_mode;

	//! ICP algorithm
	pcl::IterativeClosestPoint< PointSource, PointTarget > m_algIcp;

	//! Nonlinear ICP
	pcl::IterativeClosestPointNonLinear< PointSource, PointTarget > m_algIcpNl;

	//! Sample consensus alignment
	pcl::SampleConsensusInitialAlignment< PointSource, PointTarget, pcl::FPFHSignature33 > m_algSCA;

	//! Used registration
	tRegistration * m_registrationPtr;

	//---------------------------
	// Common parameters

	//! Maximum of iterations
	int m_maxIterations;

	//! RANSAC outlier rejection threshodl
	double m_RANSACOutlierRejectionThreshold;

	//! Maximal correspondence distance
	double m_maxCorrespondenceDistance;

	//! Transformation epsilon value
	double m_transformationEpsilon ;

	//--------------------------
	// SCA features

	//! Minimum distances between samples
	double m_scaMinSampleDistance;

	//! Number of samples to use during each iteration.
	int m_scaNumOfSamples;

	//! Number of neighbors to use when selecting a random feature correspondence
	int m_scaCorrespondenceRamdomness;

}; // CPclRegistration

/**
 * Get visible pointcloud from octomap module
 */
class COcToPcl
{
public:
	//! Constructor
	COcToPcl();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Get output pointcloud
	bool computeCloud( const SMapWithParameters & par );

	//! Get cloud
	tPointCloud & getCloud( ) { return m_cloud; }

	//! Set output cloud frame id
	void setCloudFrameId( const std::string & fid ){ m_pcFrameId = fid; }

protected:
	/// On camera position changed callback
	void onCameraChangedCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

	// main loop when spinning our own thread
	// - process callbacks in our callback queue
	// - process pending goals
	void spinThread();

	//! Test if point is in camera cone
	bool inSensorCone(const cv::Point2d& uv) const;

	//! Called when new scan was inserted and now all can be published
	void publishInternal(const ros::Time & timestamp);

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapWithParameters & mp);

protected:
	/// Should camera position and orientation be transformed?
	bool m_bTransformCamera;

	/// Camera frame id
	std::string m_cameraFrameId;

	//! Output frame id
	std::string m_pcFrameId;

	// Camera position topic name
	std::string m_cameraInfoTopic;

	/// Subscriber - camera position
	ros::Subscriber m_camPosSubscriber;

	// Mutex used to lock camera position parameters
	boost::recursive_mutex m_camPosMutex;

	//! Spin out own input callback thread
	bool m_bSpinThread;

	// these are needed when spinning up a dedicated thread
	boost::scoped_ptr<boost::thread> spin_thread_;
	ros::NodeHandle node_handle_;
	ros::CallbackQueue callback_queue_;
	volatile bool need_to_terminate_;

	/// Is camera model initialized?
	bool m_bCamModelInitialized;

	/// Camera offsets
	int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

	/// Output point cloud data
	tPointCloud m_cloud;

	//! Should i publish pointcloud
	bool m_bPublishCloud;

	/// Camera model
	image_geometry::PinholeCameraModel m_camera_model, m_camera_model_buffer;

	//! Camera info buffer
	sensor_msgs::CameraInfo m_camera_info_buffer;

	/// Camera size
	cv::Size m_camera_size, m_camera_size_buffer;

	//! Transform listener
	tf::TransformListener m_tfListener;

	//! Cloud publishers
	ros::Publisher m_pubConstrainedPC;

	/// Crawled octomap frame id
	std::string m_ocFrameId;

	/// Time stamp
	ros::Time m_DataTimeStamp, m_time_stamp;

	/// PC to sensor transformation
	tf::Transform m_to_sensorTf;

	//! Fraction of the field of view taken from the octomap (x-direction)
	double m_fracX;

	//! Fraction of the field of view taken from the octomap (y-direction)
	double m_fracY;

	//! View fraction computation matrix
	btMatrix3x3 m_fracMatrix;
};

} // namespace srs_env_model

#include "pcl_registration_module.hpp"

#endif //  pcl_registration_module_H_included

