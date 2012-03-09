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

#ifndef CollisionObjectPlugin_H_included
#define CollisionObjectPlugin_H_included

#include <but_server/ServerTools.h>
#include <message_filters/subscriber.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <tf/message_filter.h>

namespace srs
{

class CCollisionObjectPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< arm_navigation_msgs::CollisionObject >
{
public:
	/// Constructor
	CCollisionObjectPlugin(const std::string & name);

	/// Destructor
	virtual ~CCollisionObjectPlugin();

	//! Enable or disable publishing
	void enable( bool enabled ){ m_publishCollisionObject = enabled; }

	//! Should plugin publish data?
	bool shouldPublish();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp);

	/// Called when all nodes was visited.
	virtual void handlePostNodeTraversal(const ros::Time& rostime);


protected:
	//! Is publishing enabled?
	bool m_publishCollisionObject;

	//! Collision object publisher name
	std::string m_coPublisherName;

    /// Collision object publisher
    ros::Publisher m_coPublisher;

    //! Transform listener
    tf::TransformListener m_tfListener;

    //
    bool m_latchedTopics;

    //! Used frame id (input data will be transformed to it)
    std::string m_coFrameId;

    /// Crawled octomap frame id
    std::string m_ocFrameId;

    /// Transformation from octomap to the collision object frame id - rotation
    Eigen::Matrix3f m_ocToCoRot;

    /// Transformation from octomap to the collision object frame id - translation
    Eigen::Vector3f m_ocToCoTrans;

}; // class CPointCloudPlugin


}



 // namespace srs


// CollisionObjectPlugin_H_included
#endif

