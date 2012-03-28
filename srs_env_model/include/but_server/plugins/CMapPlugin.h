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
 
 #ifndef CMapPubPlugin_H_included
 #define CMapPubPlugin_H_included
 
#include <but_server/ServerTools.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <srs_env_model/GetCollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace srs
{

class CCMapPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< arm_navigation_msgs::CollisionMap >
{
protected:
	typedef COctomapCrawlerBase<tButServerOcTree::NodeType> tOctomapCrawler;

public:
	//! Constructor
	CCMapPlugin(const std::string & name);

	//! Enable or disable publishing
	void enable( bool enabled ){ m_publishCollisionMap = enabled; }

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp);

	/// Is something to publish and some subscriber to publish to?
	virtual bool shouldPublish(  );


protected:
	//! Compare two collision maps and test if they are the same
	bool sameCMaps( arm_navigation_msgs::CollisionMap * map1, arm_navigation_msgs::CollisionMap * map2 );

	//! Test collision point if it is in the collision distance from the robot
	bool isNearRobot( const btVector3 & point, double extent );

	/**
	 * @brief Get collision map service call
	 *
	 * @param req request - caller's map version
	 * @param res response - current map and current version
	 */
	bool getCollisionMapSrvCallback( srs_env_model::GetCollisionMap::Request & req, srs_env_model::GetCollisionMap::Response & res );


protected:
	//! Collision map publisher name
	std::string m_cmapPublisherName;

	//! Publisher
	ros::Publisher m_cmapPublisher;

	//! Collision map distance limit
	double m_collisionMapLimitRadius;

	//! Collision map version counter
	long int m_collisionMapVersion;

	//! Robot position in the octomap coordinate system
	tf::Stamped<btVector3> m_robotBasePosition;

	//! Collision map frame id
	std::string m_cmapFrameId;

	/// Collision map message buffer - used to resolve if collision map has changed.
	arm_navigation_msgs::CollisionMap * m_dataBuffer;

	/// Empty collision map - used when callers map id is the same as the current map id
	arm_navigation_msgs::CollisionMap m_dataEmpty;

	//! Is publishing enabled?
	bool m_publishCollisionMap;

	//! Get current collision map service
	ros::ServiceServer m_serviceGetCollisionMap;

	//! Transform listener
	tf::TransformListener m_tfListener;

	/// World to collision map transform matrix
	Eigen::Matrix4f m_worldToCMapTM;

	/// World to cmap translation and rotation
	Eigen::Matrix3f m_worldToCMapRot;
	Eigen::Vector3f m_worldToCMapTrans;

	//
	bool m_latchedTopics;

}; // class CCollisionMapPublisher
 
 }
 
 // CCMapPubPlugin_H_included
 #endif
 
 
