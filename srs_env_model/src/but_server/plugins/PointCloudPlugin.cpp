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

#include <but_server/plugins/PointCloudPlugin.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#define POINTCLOUD_CENTERS_PUBLISHER_NAME std::string("butsrv_pointcloud_centers")
#define SUBSCRIBER_POINT_CLOUD_NAME std::string("/cam3d/depth/points")
#define DEFAULT_FRAME_ID std::string("/base_footprint")
#define BASE_FRAME_ID std::string("/base_footprint")

/// Constructor
srs::CPointCloudPlugin::CPointCloudPlugin(const std::string & name, bool subscribe)
: srs::CServerPluginBase(name)
, m_publishPointCloud(true)
, m_pcPublisherName(POINTCLOUD_CENTERS_PUBLISHER_NAME)
, m_pcSubscriberName(SUBSCRIBER_POINT_CLOUD_NAME)
, m_bSubscribe( subscribe )
, m_latchedTopics( false )
, m_pcFrameId(DEFAULT_FRAME_ID)
, m_bFilterPC(true)
, m_pointcloudMinZ(-std::numeric_limits<double>::max())
, m_pointcloudMaxZ(std::numeric_limits<double>::max())
{
	m_data = new tData;
	assert( m_data != 0 );
}

/// Destructor
srs::CPointCloudPlugin::~CPointCloudPlugin()
{
	if( m_pcSubscriber )
		delete m_pcSubscriber;

	// Delete tf
	if (m_tfPointCloudSub)
		delete m_tfPointCloudSub;
}

//! Initialize plugin - called in server constructor
void srs::CPointCloudPlugin::init(ros::NodeHandle & node_handle)
{
	// Read parameters

	// Point cloud publishing topic name
	node_handle.param("pointcloud_centers_publisher", m_pcPublisherName, POINTCLOUD_CENTERS_PUBLISHER_NAME );

	// Point cloud subscribing topic name
	node_handle.param("pointcloud_subscriber", m_pcSubscriberName, SUBSCRIBER_POINT_CLOUD_NAME);

	// Point cloud limits
	node_handle.param("pointcloud_min_z", m_pointcloudMinZ, m_pointcloudMinZ);
	node_handle.param("pointcloud_max_z", m_pointcloudMaxZ, m_pointcloudMaxZ);

	// Create publisher
	m_pcPublisher = node_handle.advertise<sensor_msgs::PointCloud2> (m_pcPublisherName, 100, m_latchedTopics);


	// If should subscribe, create message filter and connect to the topic
	if( m_bSubscribe )
	{
		PERROR("Subscribing to: " << m_pcSubscriberName );
		// Create subscriber
		m_pcSubscriber  = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle, m_pcSubscriberName, 5);

		if (!m_pcSubscriber)
		{
			ROS_ERROR("Not subscribed...");
		}

		// Create message filter
		m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>( *m_pcSubscriber, m_tfListener, m_pcFrameId, 5);
		m_tfPointCloudSub->registerCallback(boost::bind( &CPointCloudPlugin::insertCloudCallback, this, _1));

		//std::cerr << "SUBSCRIBER NAME: " << m_pcSubscriberName << ", FRAMEID: " << m_pcFrameId << std::endl;
	}

	// Clear old pointcloud data
	m_data->clear();

}

//! Called when new scan was inserted and now all can be published
void srs::CPointCloudPlugin::onPublish(const ros::Time & timestamp)
{
	// No subscriber or disabled
	if( ! shouldPublish() )
		return;

	// No data...
	if( m_data->size() == 0 )
		return;

	// Convert data
	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg(*m_data, cloud);

//	PERROR("publishing...")

	// Set message parameters and publish
	cloud.header.frame_id = m_pcFrameId;
	cloud.header.stamp = timestamp;
	m_pcPublisher.publish(cloud);
}

//! Set used octomap frame id and timestamp
void srs::CPointCloudPlugin::onFrameStart( const SMapParameters & par )
{
	m_data->clear();
	m_ocFrameId = par.frameId;
	m_DataTimeStamp = m_time_stamp = par.currentTime;
}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void srs::CPointCloudPlugin::handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp)
{
//	std::cerr << "PCP: handle occupied" << std::endl;
	m_data->push_back(pcl::PointXYZ( it.getX(), it.getY(), it.getZ() ));
}

void srs::CPointCloudPlugin::handlePostNodeTraversal(const SMapParameters & mp)
{

//	PERROR("PostNode");

	// If different frame id
	if( m_ocFrameId != m_pcFrameId )
	{
		tf::StampedTransform ocToPcTf;

		// Get transform
		try {
			// Transformation - to, from, time, waiting time
			m_tfListener.waitForTransform(m_pcFrameId, m_ocFrameId,
					mp.currentTime, ros::Duration(0.2));

			m_tfListener.lookupTransform(m_pcFrameId, m_ocFrameId,
			        mp.currentTime, ocToPcTf);

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR( "Transform error.");
			return;
		}

		Eigen::Matrix4f ocToPcTM;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(ocToPcTf, ocToPcTM);	// Sensor TF to defined base TF


		// transform point cloud from sensor frame to the preset frame
		pcl::transformPointCloud(*m_data, *m_data, ocToPcTM);
	}

	// Invalidate data
	invalidate();
}


///////////////////////////////////////////////////////////////////////////////

/**
 Cloud insertion callback
 */
void srs::CPointCloudPlugin::insertCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	ros::WallTime startTime = ros::WallTime::now();

	m_data->clear();

	// Convert pc
	pcl::fromROSMsg(*cloud, *m_data);

	// If different frame id
	if( cloud->header.frame_id != m_pcFrameId )
	{
		// Some transforms
		tf::StampedTransform sensorToPcTf;

		// Get transforms
		try {
			// Transformation - from, to, time, waiting time
			m_tfListener.waitForTransform(m_pcFrameId, cloud->header.frame_id,
					cloud->header.stamp, ros::Duration(0.2));

			m_tfListener.lookupTransform(m_pcFrameId, cloud->header.frame_id,
					cloud->header.stamp, sensorToPcTf);

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR("Transform error");
			return;
		}

		Eigen::Matrix4f sensorToPcTM;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(sensorToPcTf, sensorToPcTM);	// Sensor TF to defined base TF


		// transform pointcloud from sensor frame to the preset frame
		pcl::transformPointCloud(*m_data, *m_data, sensorToPcTM);
	}

	// Filter input pointcloud
	if( m_bFilterPC )		// TODO: Optimize this by removing redundant transforms
	{
		// Get transforms to and from base id
		tf::StampedTransform pcToBaseTf;
		try {
			// Transformation - from, to, time, waiting time
			m_tfListener.waitForTransform(BASE_FRAME_ID, m_pcFrameId,
					cloud->header.stamp, ros::Duration(0.2));

			m_tfListener.lookupTransform(BASE_FRAME_ID, m_pcFrameId,
					cloud->header.stamp, pcToBaseTf);

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
			PERROR( "Transform error.");
			return;
		}

		Eigen::Matrix4f pcToBaseTM, baseToPcTM;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(pcToBaseTf, pcToBaseTM);	// Sensor TF to defined base TF
		baseToPcTM = pcToBaseTM;
		baseToPcTM.inverse();


		// transform pointcloud from pc frame to the base frame
		pcl::transformPointCloud(*m_data, *m_data, pcToBaseTM);

		// filter height and range, also removes NANs:
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
		pass.setInputCloud(m_data->makeShared());
		pass.filter(*m_data);

		// transform pointcloud back to pc frame from the base frame
		pcl::transformPointCloud(*m_data, *m_data, baseToPcTM);
	}

	// Modify header
	m_data->header = cloud->header;
    m_data->header.frame_id = m_pcFrameId;


	invalidate();

}

//! Should plugin publish data?
bool srs::CPointCloudPlugin::shouldPublish()
{
	return( m_publishPointCloud && m_pcPublisher.getNumSubscribers() > 0 );
}

