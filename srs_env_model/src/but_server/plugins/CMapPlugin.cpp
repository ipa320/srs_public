/**
 * $Id$
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: 06.02.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */


#include <but_server/plugins/CMapPlugin.h>
#include <pcl_ros/transforms.h>

#define COLLISION_MAP_RADIUS_LIMIT double(2.0)
#define COLLISION_MAP_PUBLISHER_NAME std::string("but_srv_collision_map")
#define COLLISIONMAP_FRAME_ID std::string("/base_footprint")
#define GETCOLLISIONMAP_SERVICE_NAME "but_srv_getcollisionmap"


srs::CCMapPlugin::CCMapPlugin(const std::string & name)
: srs::CServerPluginBase(name)
, m_cmapPublisherName("COLLISION_MAP_PUBLISHER_NAME")
, m_collisionMapLimitRadius(2.0)
, m_collisionMapVersion(0)
, m_cmapFrameId(COLLISIONMAP_FRAME_ID)
, m_publishCollisionMap( true )
, m_latchedTopics(false)
{
	// Create collision map and the buffer
	m_data = new arm_navigation_msgs::CollisionMap();
	m_dataBuffer = new arm_navigation_msgs::CollisionMap();
}

//! Initialize plugin - called in server constructor
void srs::CCMapPlugin::init(ros::NodeHandle & node_handle)
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
	node_handle.param("collisionmap_frame_id", m_cmapFrameId, COLLISIONMAP_FRAME_ID ); //

	// Connect publisher
	m_cmapPublisher = node_handle.advertise<arm_navigation_msgs::CollisionMap> (	m_cmapPublisherName, 100, m_latchedTopics);

	// Create and publish service
	m_serviceGetCollisionMap = node_handle.advertiseService( GETCOLLISIONMAP_SERVICE_NAME, 	&CCMapPlugin::getCollisionMapSrvCallback, this);
}

//! Called when new scan was inserted and now all can be published
void srs::CCMapPlugin::onPublish(const ros::Time & timestamp)
{
	// Should map be published?
	bool publishCollisionMap = m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0);

	// Test collision maps and swap them, if needed
	if( ! sameCMaps( m_data, m_dataBuffer ) )
	{
		// CMaps differs, increase version index and swap them
		++m_collisionMapVersion;
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
void srs::CCMapPlugin::onFrameStart( const SMapParameters & par )
{
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
		m_tfListener.waitForTransform(m_cmapFrameId, par.frameId, par.currentTime, ros::Duration(2.0)); // orig. 0.2

		m_tfListener.lookupTransform(m_cmapFrameId, par.frameId, par.currentTime, omapToCmapTf);

		m_tfListener.waitForTransform(par.frameId, robotBaseFrameId, par.currentTime, ros::Duration(2.0));

		m_tfListener.lookupTransform(par.frameId, robotBaseFrameId, par.currentTime, baseToOmapTf);
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		return;
	}

	// World TF to the collision map TF
	pcl_ros::transformAsMatrix(omapToCmapTf, m_worldToCMapTM );

	// Disassemble world to cmap translation and rotation
	m_worldToCMapRot  = m_worldToCMapTM.block<3, 3> (0, 0);
	m_worldToCMapTrans = m_worldToCMapTM.block<3, 1> (0, 3);

	// Compute robot position in the collision map coordinate system
	geometry_msgs::TransformStamped msg;
	tf::transformStampedTFToMsg(baseToOmapTf, msg);
	m_robotBasePosition.setX( msg.transform.translation.x );
	m_robotBasePosition.setY( msg.transform.translation.y );
	m_robotBasePosition.setZ( msg.transform.translation.z );

}

/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
void srs::CCMapPlugin::handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp)
{
	// Should we publish something?
	if (! m_publishCollisionMap)
		return;

	// Is this point near enough to the robot?
	if( ! isNearRobot( btVector3( it.getX(), it.getY(), it.getZ() ), it.getSize() ) )
		return;

	// Transform point from the world to the CMap TF
	Eigen::Vector3f point( it.getX(), it.getY(), it.getZ() );
	point = m_worldToCMapRot * point + m_worldToCMapTrans;

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
bool srs::CCMapPlugin::sameCMaps( arm_navigation_msgs::CollisionMap * map1, arm_navigation_msgs::CollisionMap * map2 )
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
bool srs::CCMapPlugin::isNearRobot( const btVector3 & point, double extent )
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
bool srs::CCMapPlugin::getCollisionMapSrvCallback( srs_env_model::GetCollisionMap::Request & req, srs_env_model::GetCollisionMap::Response & res )
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
bool srs::CCMapPlugin::shouldPublish(  )
{
	return(m_publishCollisionMap && (m_latchedTopics || m_cmapPublisher.getNumSubscribers() > 0));
}
