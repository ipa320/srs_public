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

#ifndef PointCloudPlugin_H_included
#define PointCloudPlugin_H_included

#include <but_server/ServerTools.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>

namespace srs
{

class CPointCloudPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< tPointCloud >
{
public:
	/// Constructor
	CPointCloudPlugin(const std::string & name, bool subscribe = true );

	/// Destructor
	virtual ~CPointCloudPlugin();

	//! Enable or disable publishing
	void enable( bool enabled ){ m_publishPointCloud = enabled; }

	//! Should plugin publish data?
	bool shouldPublish();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Initialize plugin - called in server constructor, enable or disable subscription.
	virtual void init(ros::NodeHandle & node_handle, bool subscribe){ m_bSubscribe = subscribe; init(node_handle); }

	//! Called when new scan was inserted and now all can be published
	virtual void onPublish(const ros::Time & timestamp);

	//! Set used octomap frame id and timestamp
	virtual void onFrameStart( const SMapParameters & par );

	/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
	virtual void handleOccupiedNode(const srs::tButServerOcTree::iterator& it, const SMapParameters & mp);

	/// Called when all nodes was visited.
	virtual void handlePostNodeTraversal(const ros::Time& rostime);

protected:


	/**
	 * @brief Insert point cloud callback
	 *
	 * @param cloud Input point cloud
	 */
	void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);


protected:
	//! Is publishing enabled?
	bool m_publishPointCloud;

	//! Point cloud publisher name
	std::string m_pcPublisherName;

	//! Point cloud subscriber name
	std::string m_pcSubscriberName;

    /// Subscriber - point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pcSubscriber;

    //! Message filter (we only want point cloud 2 messages)
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;

    /// Point cloud publisher
    ros::Publisher m_pcPublisher;

    //! Should this plugin subscribe to some publishing topic?
    bool m_bSubscribe;


    //! Transform listener
    tf::TransformListener m_tfListener;

    //
    bool m_latchedTopics;

    //! Used frame id (point cloud will be transformed to it)
    std::string m_pcFrameId;

    /// Crawled octomap frame id
    std::string m_ocFrameId;

    //! Do pointcloud filtering?
    bool m_bFilterPC;

    //! Minimal Z value
    double m_pointcloudMinZ;

    //! Maximal Z value
    double m_pointcloudMaxZ;

}; // class CPointCloudPlugin


} // namespace srs


// PointCloudPubPlugin_H_included
#endif

