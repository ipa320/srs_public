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

#include <srs_env_model/but_server/plugins/cmap_plugin.h>
#include <srs_env_model/topics_list.h>

#include <pcl_ros/transforms.h>

srs_env_model::CCMapPlugin::CCMapPlugin(const std::string & name)
: srs_env_model::CServerPluginBase(name)
, m_cmapPublisherName(COLLISION_MAP_PUBLISHER_NAME)
, m_collisionMapLimitRadius(2.0)
, m_collisionMapVersion(0)
, m_cmapFrameId(COLLISION_MAP_FRAME_ID)
, m_publishCollisionMap( true )
, m_latchedTopics(false)
, m_bConvertPoint( false )
, m_mapTime(0)
{
	// Create collision map and the buffer
	m_data = new arm_navigation_msgs::CollisionMap();
	m_dataBuffer = new arm_navigation_msgs::CollisionMap();
}

//! Initialize plugin - called in server constructor
void srs_env_model::CCMapPlugin::init(ros::NodeHandle & node_handle)
{
	m_collisionMapVersion = 0;
	m_dataBuffer->boxes.clear();
	m_data->boxes.clear();

	// Read parameters

	// Get collision map radius
	node_handle.param("collision_map_radius", m_collisionMapLimitRadius, COLLISION_MAP_RADIUS_LIMIT );

	// Get collision map topic name
	node_handle.param("collision_map_publisher", m_cmapPublisherName, COLLISION_MAP_PUBLISHER_NAME );

	// Get FID to which will be points transformed when publishing collision map
	node_handle.param("collisionmap_frame_id", m_cmapFrameId, COLLISION_MAP_FRAME_ID ); //

	// Create and publish service - get collision map
	m_serviceGetCollisionMap = node_handle.advertiseService( GetCollisionMap_SRV, 	&CCMapPlugin::getCollisionMapSrvCallback, this);

	// Create and publish service - is new collision map
	m_serviceIsNewCMap = node_handle.advertiseService( IsNewCMap_SRV, &CCMapPlugin::isNewCmapSrvCallback, this );

	m_serviceLockCMap = node_handle.advertiseService( LockCMap_SRV, &CCMapPlugin::lockCmapSrvCallback, this );

	// Connect publishing services
	pause( false, node_handle );
}

//! Called when new scan was inserted and now all can be published
void srs_env_model::CCMapPlugin::onPublish(const ros::Time & timestamp)
{
	if( ! shouldPublish() )
		return;

	// Should map be published?
	bool publishCollisionMap = m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0);

	// Test collision maps and swap them, if needed
	if( ! sameCMaps( m_data, m_dataBuffer ) )
	{
		// CMaps differs, increase version index and swap them
		++m_collisionMapVersion;
		m_mapTime = timestamp;
		swap( m_data, m_dataBuffer );

		// Call invalidation
		invalidate();
	}

	// Publish collision map
	if (publishCollisionMap) {
//		std::cerr << "Publishing cmap. Frame id: " << m_cmapFrameId << ", Size: " << m_data->boxes.size() << std::endl;
		m_data->header.frame_id = m_cmapFrameId;
		m_data->header.stamp = m_time_stamp;
		m_cmapPublisher.publish(*m_data);
	}
}

//! Set used octomap frame id and timestamp
void srs_env_model::CCMapPlugin::onFrameStart( const SMapParameters & par )
{
	if( m_bLocked )
		return;

	// store parameters
	tOctomapCrawler::onFrameStart( par );

	// Reset collision map buffer
	m_dataBuffer->boxes.clear();

	std::string robotBaseFrameId("/base_footprint");

	// Get octomap to collision map transform matrix
	tf::StampedTransform omapToCmapTf,	// Octomap to collision map
						 baseToOmapTf;  // Robot baselink to octomap
	try
	{
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_cmapFrameId, par.frameId, par.currentTime, ros::Duration(5));

		m_tfListener.lookupTransform(m_cmapFrameId, par.frameId, par.currentTime, omapToCmapTf);

		m_tfListener.waitForTransform(par.frameId, robotBaseFrameId, par.currentTime, ros::Duration(5));

		m_tfListener.lookupTransform(par.frameId, robotBaseFrameId, par.currentTime, baseToOmapTf);
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR( "Transform error.");
		return;
	}

	// World TF to the collision map TF
	pcl_ros::transformAsMatrix(omapToCmapTf, m_worldToCMapTM );

	// Disassemble world to cmap translation and rotation
	m_worldToCMapRot  = m_worldToCMapTM.block<3, 3> (0, 0);
	m_worldToCMapTrans = m_worldToCMapTM.block<3, 1> (0, 3);

	m_bConvertPoint = m_cmapFrameId != par.frameId;

	// Compute robot position in the collision map coordinate system
	geometry_msgs::TransformStamped msg;
	tf::transformStampedTFToMsg(baseToOmapTf, msg);
	m_robotBasePosition.setX( msg.transform.translation.x );
	m_robotBasePosition.setY( msg.transform.translation.y );
	m_robotBasePosition.setZ( msg.transform.translation.z );

}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void srs_env_model::CCMapPlugin::handleOccupiedNode(srs_env_model::tButServerOcTree::iterator& it, const SMapParameters & mp)
{
	if( m_bLocked )
			return;

	// Should we publish something?
	if (! m_publishCollisionMap)
		return;

	// Is this point near enough to the robot?
	if( ! isNearRobot( btVector3( it.getX(), it.getY(), it.getZ() ), it.getSize() ) )
		return;

	Eigen::Vector3f point( it.getX(), it.getY(), it.getZ() );

	if( m_bConvertPoint )
	{
	    // Transform point from the world to the CMap TF
	    point = m_worldToCMapRot * point + m_worldToCMapTrans;
	}

	// Add point to the collision map
	arm_navigation_msgs::OrientedBoundingBox box;
	double size = it.getSize();
	box.extents.x = box.extents.y = box.extents.z = size;
	box.axis.x = box.axis.y = 0.0;
	box.axis.z = 1.0;
	box.angle = 0.0;
	box.center.x = point[0];
	box.center.y = point[1];
	box.center.z = point[2];


	m_dataBuffer->boxes.push_back(box);
}

/**
 * @brief Compare two collision maps
 *
 * @param map1 First map to compare
 * @param map2 Second map to compare
 * @return true if maps are the same
 */
bool srs_env_model::CCMapPlugin::sameCMaps( arm_navigation_msgs::CollisionMap * map1, arm_navigation_msgs::CollisionMap * map2 )
{
	// Wrong input
	if( map1 == 0 || map2 == 0 )
	{
//		std::cerr << "Some null. map1: " << map1 << ", map2: " << map2 << std::endl;
		return false;
	}

	// Two maps cannot be the same, if differs in the size
	if( map1->boxes.size() != map2->boxes.size() )
	{
//		std::cerr << "Maps size different. map1: " << map1->boxes.size() << ", map2: " << map2->boxes.size() << std::endl;
		return false;
	}

	// Compare maps box by box
	arm_navigation_msgs::CollisionMap::_boxes_type::iterator icm1, icm2;
	arm_navigation_msgs::CollisionMap::_boxes_type::iterator end1( map1->boxes.end());

	long num( 0 );
	for( icm1 = map1->boxes.begin(), icm2 = map2->boxes.begin(); icm1 != end1; ++icm1, ++icm2 )
	{
		if( isGreat( icm1->center.x - icm2->center.x ) ||
				isGreat( icm1->center.y - icm2->center.y ) ||
				isGreat( icm1->center.z - icm2->center.z ) ||
				isGreat( icm1->extents.x - icm2->extents.x ) )
		{
//			std::cerr << "Point number " << num << " different." << std::endl;
			return false;
		}
		++num;
	}

	return true;
}



/**
 * @brief Test collision object if it is in the collision distance from the robot
 *
 * @param point Position of the object center - in the world coordinates
 * @param extent Bounding sphere of the collision object
 * @return
 */
bool srs_env_model::CCMapPlugin::isNearRobot( const btVector3 & point, double extent )
{
	btScalar s( point.distance( m_robotBasePosition ) );

	return s - extent < m_collisionMapLimitRadius;
}

/**
 * @brief Get collision map service call
 *
 * @param req request - caller's map version
 * @param res response - current map and current version
 */
bool srs_env_model::CCMapPlugin::getCollisionMapSrvCallback( srs_env_model::GetCollisionMap::Request & req, srs_env_model::GetCollisionMap::Response & res )
{

	PERROR( "Get collision map service called" );

	// Response map version should be current version number
	res.current_version = m_collisionMapVersion;

	// Get callers map version and compare to the current version number
	if( req.my_version != m_collisionMapVersion )
	{
		res.map = *m_data;
	}else{
		// No new data needed
		res.map = m_dataEmpty;
	}

	return true;
}

/**
 * @brief Returns true, if should be data published and are some subscribers.
 */
bool srs_env_model::CCMapPlugin::shouldPublish(  )
{
	return(m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0));
}

/**
 * @brief Get true if given timestamp is older then current map time
 * @param req request - caller's map timestamp
 * @param res response - true, if new map and current timestamp
 */
bool srs_env_model::CCMapPlugin::isNewCmapSrvCallback( srs_env_model::IsNewCollisionMap::Request & req, srs_env_model::IsNewCollisionMap::Response & res )
{
	PERROR( "Is new cmap service called ");

	res.is_newer = req.my_time < m_mapTime;
	res.current_time = m_mapTime;

	return true;
}

/**
 * @brief Lock collision map - disable its updates from new point cloud data
 * @param req request - bool - lock/unlock
 * @param res response -
 */
bool srs_env_model::CCMapPlugin::lockCmapSrvCallback( srs_env_model::LockCollisionMap::Request & req, srs_env_model::LockCollisionMap::Response & res )
{
	boost::mutex::scoped_lock lock( m_lockData );

	bool locked( req.lock != 0 );

	m_bLocked = locked;

	if( locked )
		std::cerr << "Locking called. Lock set." << std::endl;
	else
		std::cerr << "Locking called. Lock removed." << std::endl;

	return true;
}

/**
 *  Disconnect plugin from all topics
 */
void srs_env_model::CCMapPlugin::pause( bool bPause, ros::NodeHandle & node_handle )
{
	if( bPause )
		m_cmapPublisher.shutdown();
	else
		m_cmapPublisher = node_handle.advertise<arm_navigation_msgs::CollisionMap> ( m_cmapPublisherName, 100, m_latchedTopics);
}



