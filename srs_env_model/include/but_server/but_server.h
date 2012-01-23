/**
 * $Id: but_server.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Modified by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 *
 */

/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SERVER_H
#define SERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <arm_navigation_msgs/CollisionMap.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap_ros/ClearBBXRegion.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeKey.h>

#include "octonode.h"

#include <but_gui/Plane.h>
#include <srs_env_model/RemoveObject.h>
#include <srs_env_model/AddPlanes.h>
#include <srs_env_model/AddPlane.h>
#include <srs_env_model/PlaneDesc.h>

typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;


/**
  BUT dynamic scene server class.
  */
class CButServer{

public:
    //! Type of the used pointcloud
    typedef pcl::PointCloud<pcl::PointXYZ> tPCLPointCloud;

    //! Constructor - load file
    CButServer(const std::string& filename= "");

    //! Destructor
    virtual ~CButServer();


    /// Reset octomap service
    bool resetOctomapCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

protected:

    //! Publish stored map
    void publishMap(const ros::Time& rostime = ros::Time::now()) const;

    //! Publish all
    void publishAll(const ros::Time& rostime = ros::Time::now());


    //! Clear octomap data
    void resetOctomap();


    /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
    void filterGroundPlane(const tPCLPointCloud& pc, tPCLPointCloud& ground, tPCLPointCloud& nonground) const;

    /**
     * @brief update occupancy map with a scan labeled as ground and nonground.
     * The scans should be in the global map frame.
     *
     * @param sensorOrigin origin of the measurements for raycasting
     * @param ground scan endpoints on the ground plane (only clear space)
     * @param nonground all other endpoints (clear up to occupied endpoint)
     */
    void insertScan(const tf::Point& sensorOrigin, const tPCLPointCloud& ground, const tPCLPointCloud& nonground);

    /**
     * @brief Insert point cloud callback
     *
     * @param cloud Input point cloud
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    /**
     * @brief New camera info published callback
     *
     * @param ci Camera info
     */
    void cameraInfoCallback( const sensor_msgs::CameraInfo::ConstPtr& ci){};

    /**
     * @brief Insert or modify plane array
     *
     * @param pa Array of planes
     */
    bool insertPlaneCallback( srs_env_model::AddPlanes::Request & req, srs_env_model::AddPlanes::Response & res );

    /**
     * @brief Insert/modify/remove plane
     *
     * @param plane Plane
     */
    void operatePlane( const srs_env_model::PlaneDesc & plane );

    /**
     * @brief Service helper - add plane
     *
     * @param plane Added plane
     */
    void addPlaneSrvCall( const srs_env_model::PlaneDesc & plane, const std::string & name );

    /**
     * @brief Service helper - remove plane
     *
     * @param plane Added plane
     */
    void removePlaneSrvCall( const srs_env_model::PlaneDesc & plane, const std::string & name );

    /**
     *  @brief Get unique string (used as interactive marker name)
     */
    std::string getUniqueName();

    /**
 * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
 * @param key
 * @return
 */
    bool isSpeckleNode(const octomap::OcTreeKey& key) const;

    /// hook that is called after traversing all nodes
    void handlePreNodeTraversal(const ros::Time& rostime);

    /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
    void handleNode(const octomap::OcTreeROS::OcTreeType::iterator& it) {};

    /// hook that is called when traversing occupied nodes of the updated Octree (updates 2D map projection here)
    void handleOccupiedNode(const octomap::OcTreeROS::OcTreeType::iterator& it);

    /// hook that is called when traversing free nodes of the updated Octree (updates 2D map projection here)
    void handleFreeNode(const octomap::OcTreeROS::OcTreeType::iterator& it);

    /// hook that is called after traversing all nodes
    void handlePostNodeTraversal(const ros::Time& rostime);

protected:
    std_msgs::ColorRGBA heightMapColor(double h) const;

    /// Node handle
    ros::NodeHandle m_nh;

    /// Publishers
    ros::Publisher m_markerPub, m_binaryMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_collisionMapPub;

    /// Subscriber - point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> * m_pointCloudSub;

    /// Subscriber - camera information
    //message_filters::Subscriber<sensor_msgs::CameraInfo> * m_camInfoSub;

    //! Message filter (we only want point cloud 2 messages)
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;

    //! Message filter - camera info
    //tf::MessageFilter<sensor_msgs::CameraInfo>* m_tfCameraInfoSub;

    ros::ServiceServer m_octomapService, m_clearBBXService, m_resetService;
    tf::TransformListener m_tfListener;

    /// Stored octomap
    octomap::OcTreeROS *m_octoMap;

    octomap::KeyRay m_keyRay;  // temp storage for ray casting

    double m_maxRange;
    bool m_useHeightMap;
    std_msgs::ColorRGBA m_color;
    double m_colorFactor;

    bool m_latchedTopics;

    double m_res;
    unsigned m_treeDepth;
    double m_probHit;
    double m_probMiss;
    double m_thresMin;
    double m_thresMax;

    double m_pointcloudMinZ;
    double m_pointcloudMaxZ;
    double m_occupancyMinZ;
    double m_occupancyMaxZ;
    double m_minSizeX;
    double m_minSizeY;

    /// Unique name counter
    long int m_uniqueNameCounter;

    /// Should speckles be filtered?
    bool m_filterSpeckles;

    /// Should ground plane be filtered?
    bool m_filterGroundPlane;
    double m_groundFilterDistance;
    double m_groundFilterAngle;
    double m_groundFilterPlaneDistance;

    /// downprojected 2D map:
    nav_msgs::OccupancyGrid m_gridmap;

    octomap::OcTreeKey m_paddedMinKey;

    /// Should 2D map be published?
    bool m_publish2DMap;

    /// Should collision map be published
    bool m_publishCollisionMap;

    /// Collision map message
    arm_navigation_msgs::CollisionMap m_cmap;

    /// Services
    ros::ServiceServer m_serviceResetOctomap, // Reset octomap service
                       m_serviceInsertPlanes; // Insert some planes

    /// Interactive markers server pointer
    but_gui::InteractiveMarkerServerPtr m_imServer;

    /// Remove object from the interactive markers server pointer
    ros::ServiceClient m_removeInteractiveMarkerService;

    /// Add plane interactive marker service
    ros::ServiceClient m_addInteractivePlaneService;

    /// Connection settings
    std::string m_ocupiedCellsPublisher,
                m_octomapBinaryPublisher,
                m_pointcloudCentersPublisher,
                m_collisionObjectPublisher,
                m_mapPublisher,
                m_collisionMapPublisher,
                m_pointcloudSubscriber,
                m_cameraInfoSubscriber,
                m_worldFrameId,
                m_baseFrameId;


    // DETECTED ENTITIES
    /// Plane
    typedef std::pair< std::string, srs_env_model::PlaneDesc > tNamedPlane;
    typedef std::map< int, tNamedPlane > tPlanesMap;
    tPlanesMap m_dataPlanes;

    // Planes frame id
    std::string m_planesFrameId;


};




#endif // SERVER_H
