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

#ifndef OCTOMAPPLUGIN_H_INCLUDED
#define OCTOMAPPLUGIN_H_INCLUDED

#include <but_server/ServerTools.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

namespace srs
{
class COctoMapPlugin : public CServerPluginBase, public CDataHolderBase< tButServerOcMap >
{
public:
	// Crawling signals

	/// On crawling start
	typedef boost::signal< void (const SMapParameters &) > tSigOnStart;

	/// On node
	typedef boost::signal< void (const tButServerOcTree::iterator &, const SMapParameters & ) > tSigOnNode;

	/// On free node
	typedef boost::signal< void (const tButServerOcTree::iterator &, const SMapParameters & ) > tSigOnFreeNode;

	/// On occupied node
	typedef boost::signal< void (const tButServerOcTree::iterator &, const SMapParameters & ) > tSigOnOccupiedNode;

	/// Post node traversal
	typedef boost::signal< void (const ros::Time &) > tSigOnPost;

public:
	/// Constructor
	COctoMapPlugin(const std::string & name);

	/// Constructor - load data from the file
	COctoMapPlugin( const std::string & name, const std::string & filename );

	/// Insert pointcloud
	void insertCloud( const tPointCloud& cloud);

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

	//! Reset octomap
	void reset();

	//! Get current octomap size
	unsigned getSize() { return m_data->octree.size(); }

	//! Get current tree depth
	unsigned getTreeDepth() { return m_mapParameters.treeDepth; }

	/// Get octomap resolution
	double getResolution(){ return m_mapParameters.resolution; }

	/// Crawl octomap
	void crawl( const ros::Time & currentTime );

	tSigOnStart & getSigOnStart() { return m_sigOnStart; }

	tSigOnNode & getSigOnNode() { return m_sigOnNode; }

	tSigOnFreeNode & getSigOnFreeNode() { return m_sigOnFreeNode; }

	tSigOnOccupiedNode & getSigOnOccupiedNode() { return m_sigOnOccupiedNode; }

	tSigOnPost & getSigOnPost(){ return m_sigOnPost; }

	/// Should something be published?
	virtual bool shouldPublish();

protected:

	/// Set octomap default parameters
	void setDefaults();

	/**
	 * @brief Insert scan to the octomap
	 */
	void insertScan(const tf::Point& sensorOriginTf, const tPointCloud& ground, const tPointCloud& nonground);

	/// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
	void filterGroundPlane(const tPointCloud& pc, tPointCloud& ground, tPointCloud& nonground) const;

	/// On octomap crawling start
	void onCrawlStart(const ros::Time & currentTime);

	/// Handle node
	void handleNode(const tButServerOcTree::iterator & it, const SMapParameters & mp);

	/// Handle free node
	void handleFreeNode(const tButServerOcTree::iterator & it, const SMapParameters & mp);

	/// Handle occupied node
	void handleOccupiedNode(const tButServerOcTree::iterator & it, const SMapParameters & mp);

	/// Called when all nodes was visited.
	virtual void handlePostNodeTraversal(const ros::Time& rostime);

	/// Fill map parameters
	void fillMapParameters(const ros::Time & time);

	/// Reset octomap service callback
	bool resetOctomapCB(std_srvs::Empty::Request& request,	std_srvs::Empty::Response& response);

protected:
	///



    /// Should ground plane be filtered?
    bool m_filterGroundPlane;
    double m_groundFilterDistance;
    double m_groundFilterAngle;
    double m_groundFilterPlaneDistance;

    /// Temporary storage for ray casting
    octomap::KeyRay m_keyRay;

    /// On traversal start
    tSigOnStart m_sigOnStart;

    /// On node signal
    tSigOnNode m_sigOnNode;

    /// On free node signal
    tSigOnFreeNode m_sigOnFreeNode;

    /// On occupied node signal
    tSigOnOccupiedNode m_sigOnOccupiedNode;

    /// On traversal end signal
    tSigOnPost m_sigOnPost;

    /// Octomap parameters
    SMapParameters m_mapParameters;

    //! Transform listener
    tf::TransformListener m_tfListener;

    /// Reset octomap service
    ros::ServiceServer m_serviceResetOctomap;

}; // class COctoMapPlugin;


}



 // namespace srs

// OCTOMAPPLUGIN_H_INCLUDED
#endif

