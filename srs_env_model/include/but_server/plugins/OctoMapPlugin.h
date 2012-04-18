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

#ifndef OCTOMAPPLUGIN_H_INCLUDED
#define OCTOMAPPLUGIN_H_INCLUDED

#include <but_server/ServerTools.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

//========================
// Filtering

#include <image_geometry/pinhole_camera_model.h>

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
	typedef boost::signal< void (const SMapParameters &) > tSigOnPost;

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

	/// Publishing callback
	virtual void onPublish(const ros::Time & timestamp);

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
	virtual void handlePostNodeTraversal(const SMapParameters & mp);

	/// Fill map parameters
	void fillMapParameters(const ros::Time & time);

	/// Reset octomap service callback
	bool resetOctomapCB(std_srvs::Empty::Request& request,	std_srvs::Empty::Response& response);

	/// Camera info callback
	void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

	// ------------------------------------------------------------------------
	// Obstacle cleaning

	/// Remove outdated nodes
	void degradeOutdatedRaycasting(const std_msgs::Header& sensor_header, const octomap::point3d& sensor_origin);

	/// Remove speckles
	void degradeSingleSpeckles();

	/// Compute bounding box from the sensor position and cone
	void computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max);

	/// Is point in sensor cone?
	bool inSensorCone(const cv::Point2d& uv) const;

	/// Return true, if occupied cell is between origin and p
	bool isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p) const;

	/// Get used sensor origin
	octomap::point3d getSensorOrigin(const std_msgs::Header& sensor_header);

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

    /// Should octomap be published
    bool m_bPublishOctomap;

    //! Octomap publisher name
    std::string m_ocPublisherName;

    /// Octomap publisher
    ros::Publisher m_ocPublisher;

    bool m_latchedTopics;

    /// Remove specle nodes now
    bool m_removeSpecles;

    int filecounter;

    //=========================================================================
    // Filtering

    /// Should be outdated nodes be removed?
    bool m_bRemoveOutdated;

    /// Camera model
    image_geometry::PinholeCameraModel m_camera_model;

    /// Is camera model initialized?
    bool m_bCamModelInitialized;

    /// Camera size
    cv::Size m_camera_size;

    /// Camera info topic name
    std::string m_camera_info_topic;

    /// Camera info subscriber
    ros::Subscriber*  m_ciSubscriber;

    /// Should be markers visualized
    bool m_bVisualizeMarkers;

    /// Markers visualizing publisher
    ros::Publisher m_markerPublisher;

    /// Markers topic name
    std::string m_markers_topic_name;

    /// Camera offsets
    int m_camera_stereo_offset_left, m_camera_stereo_offset_right;

}; // class COctoMapPlugin;


}



 // namespace srs

// OCTOMAPPLUGIN_H_INCLUDED
#endif

