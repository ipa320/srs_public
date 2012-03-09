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

#ifndef Map2DPlugin_H_included
#define Map2DPlugin_H_included

#include <but_server/ServerTools.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace srs
{

class CMap2DPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< nav_msgs::OccupancyGrid >
{
public:
	/// Constructor
	CMap2DPlugin(const std::string & name);

	/// Destructor
	virtual ~CMap2DPlugin();

	//! Enable or disable publishing
	void enable( bool enabled ){ m_publishMap2D = enabled; }

	//! Should plugin publish data?
	bool shouldPublish();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	//! Handle free node (does nothing here)
	virtual void handleFreeNode(const tButServerOcTree::iterator & it, const SMapParameters & mp );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp);

	/// Called when all nodes was visited.
	virtual void handlePostNodeTraversal(const ros::Time& rostime);


protected:
	//! Is publishing enabled?
	bool m_publishMap2D;

	//! Collision object publisher name
	std::string m_map2DPublisherName;

    /// Collision object publisher
    ros::Publisher m_map2DPublisher;

    //! Transform listener
    tf::TransformListener m_tfListener;

    //
    bool m_latchedTopics;

    //! Used frame id (input data will be transformed to it)
    std::string m_map2DFrameId;

    /// Crawled octomap frame id
    std::string m_ocFrameId;

    /// Transformation from octomap to the collision object frame id - rotation
    Eigen::Matrix3f m_ocToMap2DRot;

    /// Transformation from octomap to the collision object frame id - translation
    Eigen::Vector3f m_ocToMap2DTrans;

    /// Padded key minimum
    octomap::OcTreeKey m_paddedMinKey;

    /// Map limits
    double m_minSizeX;
    double m_minSizeY;

}; // class CMap2DPlugin


} // namespace srs



 // namespace srs


// Map2DPlugin_H_included
#endif

