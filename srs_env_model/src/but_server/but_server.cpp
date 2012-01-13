/**
 * $Id: but_server.cpp 145 2012-01-13 09:48:58Z ihulik $
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

#include <but_server/but_server.h>
#include <sstream>


// Define publishers names
#define MARKERS_PUBLISHER_NAME std::string("butsrv_ocupied_cells_markers")
#define OCTOMAP_BINARY_PUBLISHER_NAME std::string("butsrv_octomap_binary")
#define POINTCLOUD_CENTERS_PUBLISHER_NAME std::string("butsrv_pointcloud_centers")
#define COLLISION_OBJECT_PUBLISHER_NAME std::string("butsrv_collision_object")
#define MAP_PUBLISHER_NAME std::string("but_map")
#define COLLISION_MAP_PUBLISHER_NAME std::string("but_srv_collision_map")
#define SUBSCRIBER_POINT_CLOUD std::string("cam3d/depth/points")
#define SUBSCRIBER_CAMERA_INFO std::string("cam3d/depth/camera_info")
#define WORLD_FRAME_ID std::string("/map")
#define BASE_FRAME_ID std::string("base_footprint")

/**
  Constructor
  */
CButServer::CButServer(const std::string& filename)
  : m_nh(),
    m_pointCloudSub(NULL),
    m_tfPointCloudSub(NULL),
    m_octoMap(NULL),        // Octomap data
    m_maxRange(-1.0),
    m_useHeightMap(true),
    m_colorFactor(0.8),
    m_latchedTopics(false),
    m_res(0.05),
    m_treeDepth(0),
    m_probHit(0.7), m_probMiss(0.4),
    m_thresMin(0.12), m_thresMax(0.97),
    m_pointcloudMinZ(-std::numeric_limits<double>::max()),
    m_pointcloudMaxZ(std::numeric_limits<double>::max()),
    m_occupancyMinZ(-std::numeric_limits<double>::max()),
    m_occupancyMaxZ(std::numeric_limits<double>::max()),
    m_minSizeX(0.0), m_minSizeY(0.0),
    m_uniqueNameCounter(0),
    m_filterSpeckles(false), m_filterGroundPlane(false),
    m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07)
{
    // Get node handla
    ros::NodeHandle private_nh("~");

    // Set parameters
    private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
    private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
    private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
    private_nh.param("color_factor", m_colorFactor, m_colorFactor);

    private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
    private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
    private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
    private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
    private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
    private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

    private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
    private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
    // distance of points from plane for RANSAC
    private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
    // angular derivation of found plane:
    private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
    // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
    private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

    private_nh.param("max_range", m_maxRange, m_maxRange);

    private_nh.param("resolution", m_res, m_res);
    private_nh.param("sensor_model/hit", m_probHit, m_probHit);
    private_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
    private_nh.param("sensor_model/min", m_thresMin, m_thresMin);
    private_nh.param("sensor_model/max", m_thresMax, m_thresMax);


    // Topic names
    private_nh.param("ocupied_cells_publisher", m_ocupiedCellsPublisher, MARKERS_PUBLISHER_NAME );
    private_nh.param("octomap_binary_publisher", m_octomapBinaryPublisher, OCTOMAP_BINARY_PUBLISHER_NAME );
    private_nh.param("pointcloud_centers_publisher", m_pointcloudCentersPublisher, POINTCLOUD_CENTERS_PUBLISHER_NAME );
    private_nh.param("collision_object_publisher", m_collisionObjectPublisher, COLLISION_OBJECT_PUBLISHER_NAME );
    private_nh.param("map_publisher", m_mapPublisher, MAP_PUBLISHER_NAME );
    private_nh.param("collision_map_publisher", m_collisionMapPublisher, COLLISION_MAP_PUBLISHER_NAME );
    private_nh.param("pointcloud_subscriber", m_pointcloudSubscriber, SUBSCRIBER_POINT_CLOUD);
    private_nh.param("camera_info_subscriber", m_cameraInfoSubscriber, SUBSCRIBER_CAMERA_INFO );
    private_nh.param("world_frame_id", m_worldFrameId, WORLD_FRAME_ID );
    private_nh.param("base_frame_id", m_baseFrameId, BASE_FRAME_ID );


    // Reset octomap
    resetOctomap();

    // Initialize interactive markers server
    m_imServer.reset( new InteractiveMarkerServer( "BUT_IM_Server", "", false ) );

    double r, g, b, a;

    private_nh.param("color/r", r, 0.0);
    private_nh.param("color/g", g, 0.0);
    private_nh.param("color/b", b, 1.0);
    private_nh.param("color/a", a, 1.0);
    m_color.r = r;
    m_color.g = g;
    m_color.b = b;
    m_color.a = a;

    // Is map static (loaded from file)?
    bool staticMap(filename != "");

    m_latchedTopics = staticMap;
    private_nh.param("latch", m_latchedTopics, m_latchedTopics);

    // Create publishers
    m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>(m_ocupiedCellsPublisher, 100, m_latchedTopics);
    m_binaryMapPub = m_nh.advertise<octomap_ros::OctomapBinary>(m_octomapBinaryPublisher, 100, m_latchedTopics);
    m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>(m_pointcloudCentersPublisher, 100, m_latchedTopics);
    m_collisionObjectPub = m_nh.advertise<arm_navigation_msgs::CollisionObject>(m_collisionObjectPublisher, 100, m_latchedTopics);
    m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_mapPublisher, 100, m_latchedTopics);
    m_collisionMapPub = m_nh.advertise<arm_navigation_msgs::CollisionMap>( m_collisionMapPublisher, 100, m_latchedTopics);

    // a filename to load is set => distribute a static map latched
    if (staticMap){

        // Try to load data
        if (m_octoMap->octree.readBinary(filename))
        {
            ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octoMap->octree.size());

            // get tree depth
            m_treeDepth = m_octoMap->octree.getTreeDepth();

            // get resolution
            m_res = m_octoMap->octree.getResolution();

            m_gridmap.info.resolution = m_res;

            // Publish data
            publishAll();

        } else {

            // Something is wrong - cannot load data...
            ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
            exit(-1);
        }

    } else {
        // Dynamic map
        // otherwise: do scan integration

        // Create pointcloud2 subscriber
        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, m_pointcloudSubscriber, 5);

        if( !m_pointCloudSub )
        {
            ROS_ERROR("Not subscribed...");
        }

        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
        m_tfPointCloudSub->registerCallback(boost::bind(&CButServer::insertCloudCallback, this, _1));


        ROS_INFO("Octomap should be dynamic..." );
        // TODO - add some service message filters etc.
    }

    // Subscribe to the camera info topic
   // m_camInfoSub = new message_filters::Subscriber<sensor_msgs::CameraInfo> ( m_nh, m_cameraInfoSubscriber, 200 );
   // m_tfCameraInfoSub = new tf::MessageFilter<sensor_msgs::CameraInfo> (*m_camInfoSub, m_tfListener, m_worldFrameId, 100);
   // m_tfCameraInfoSub->registerCallback(boost::bind(&CButServer::cameraInfoCallback, this, _1));


    // Advertise services
    m_serviceResetOctomap = m_nh.advertiseService("reset_octomap", &CButServer::resetOctomapCB, this);
    m_serviceInsertPlanes = m_nh.advertiseService("insert_plane", &CButServer::insertPlaneCallback, this );

    // Connect to the services
    m_removeInteractiveMarkerService = m_nh.serviceClient<srs_env_model::RemoveObject> ("remove_object");
    m_addInteractivePlaneService = m_nh.serviceClient<srs_env_model::AddPlane> ("add_plane");


    // Interactive marker server test
    {
        // Creating Plane object with name "plane1"
        but_gui::Plane *plane = new but_gui::Plane(m_imServer, "plane1", "" );

        // Color
        std_msgs::ColorRGBA c;
        c.r = 1.0; c.g = c.b = 0.0; c.a = 1.0;
        plane->setColor(c);

        // Positioning
        geometry_msgs::Pose p;
        p.position.x = p.position.y = p.position.z = 0.0;
        plane->setPose( p );

        // Scaling
        but_gui::Scale s;
        s.x = s.y = s.z = 10.0;
        plane->setScale( s );

        // Creating plane with specified attributes
        plane->create();

        // Inserting object into server
        plane->insert();
    }

}  // Constructor

/**
  Destructor
  */
CButServer::~CButServer(){

    // Delete point clouds
    if (m_pointCloudSub)
        delete m_pointCloudSub;

    // Delete tf
    if (m_tfPointCloudSub)
      delete m_tfPointCloudSub;

    // Delete octomap
    if (m_octoMap)
      delete m_octoMap;

}

/**
 * @brief update occupancy map with a scan labeled as ground and nonground.
 * The scans should be in the global map frame.
 *
 * @param sensorOrigin origin of the measurements for raycasting
 * @param ground scan endpoints on the ground plane (only clear space)
 * @param nonground all other endpoints (clear up to occupied endpoint)
 */
void CButServer::insertScan(const tf::Point& sensorOriginTf, const tPCLPointCloud& ground, const tPCLPointCloud& nonground)
{
    // Write some debug info
//    ROS_INFO("Inserting scan. Points: %u ", ground.size() + nonground.size() );


    octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;
    // insert ground points only as free:
    for (tPCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it){
        octomap::point3d point(it->x, it->y, it->z);
        // maxrange check
        if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
            point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
        }

        // only clear space (ground points)
        if (m_octoMap->octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }
    }



    // all other points: free on ray, occupied on endpoint:
    for (tPCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
        octomap::point3d point(it->x, it->y, it->z);
        // maxrange check
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
//*
            // free cells
            if (m_octoMap->octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }
//*/
            // occupied endpoint
            octomap::OcTreeKey key;
            if (m_octoMap->octree.genKey(point, key)){
                occupied_cells.insert(key);
            }
        } else {// ray longer than maxrange:;
//*
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octoMap->octree.computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

//*/
        }
    }


    // mark free cells only if not seen occupied in this cloud
    for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octoMap->octree.updateNode(*it, false, true);
        }
    }


    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
        m_octoMap->octree.updateNode(*it, true, true);
    }
    // TODO: eval lazy+updateInner vs. proper insertion
    m_octoMap->octree.updateInnerOccupancy();
    m_octoMap->octree.prune();

}


/**
  Publish all data
  */
void CButServer::publishAll(const ros::Time& rostime){


    // Store start time
    ros::WallTime startTime = ros::WallTime::now();

    // If no data, do nothing
    if (m_octoMap->octree.size() <= 1)
    {
        ROS_WARN("Nothing to publish, octree is empty");
        return;
    }

//    ROS_INFO("Publishing data. Tree size: %zu", m_octoMap->octree.size() );

    // What should be published - get if there are any subscribers...
    bool publishCollisionObject = (m_latchedTopics || m_collisionObjectPub.getNumSubscribers() > 0);
    bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
    bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
    bool publishOctoMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
    m_publishCollisionMap = (m_latchedTopics || m_collisionMapPub.getNumSubscribers() > 0 );

    // init collision object:
    arm_navigation_msgs::CollisionObject collisionObject;
    collisionObject.header.frame_id = m_worldFrameId;
    collisionObject.header.stamp = rostime;
    collisionObject.id = "map";

    arm_navigation_msgs::Shape shape;
    shape.type = arm_navigation_msgs::Shape::BOX;
    shape.dimensions.resize(3);

    geometry_msgs::Pose pose;
    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // init markers:
    visualization_msgs::MarkerArray occupiedNodesVis;

    // each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(m_treeDepth+1);

    // init pointcloud:
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    // call pre-traversal hook:
    handlePreNodeTraversal(rostime);

    // Clear collision map
    if( m_publishCollisionMap )
    {
        m_cmap.boxes.clear();
    }

    // Iterate through octree
    for (octomap::OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(), end = m_octoMap->octree.end(); it != end; ++it)
    {

        // call general hook:
        handleNode(it);

        // Node is occupied?
        if (m_octoMap->octree.isNodeOccupied(*it))
        {
            // Get node z coordinate
            double z = it.getZ();

            // Is this node in the tested interval?
            if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
            {
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();
                octomap::OcTreeKey nKey = it.getKey();

                // Ignore speckles in the map:
                if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(nKey))
                {
                    ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
                    continue;
                } // else: current octree node is no speckle, send it out

                // Add node to the pc
                handleOccupiedNode(it);


                // create collision object:
                if (publishCollisionObject)
                {
                    shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = size;
                    collisionObject.shapes.push_back(shape);
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.position.z = z;
                    collisionObject.poses.push_back(pose);
                }



                //PUBLISH - as a marker
                if (publishMarkerArray)
                {
                    unsigned idx = it.getDepth();
                    assert(idx < occupiedNodesVis.markers.size());

                    geometry_msgs::Point cubeCenter;
                    cubeCenter.x = x;
                    cubeCenter.y = y;
                    cubeCenter.z = z;

                    // Add marker to the array
                    occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

                    // If using z coordinate as a heightmap value, calculate color of the marker
                    if (m_useHeightMap)
                    {
                        double minX, minY, minZ, maxX, maxY, maxZ;
                        m_octoMap->octree.getMetricMin(minX, minY, minZ);
                        m_octoMap->octree.getMetricMax(maxX, maxY, maxZ);

                        double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                        occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
                    }
                }


                // PUBLISH - as a pointcloud
                if (publishPointCloud)
                    pclCloud.push_back(pcl::PointXYZ(x, y, z));
            }


        } else{ // node not occupied => mark as free in 2D map if unknown so far

            handleFreeNode(it);

        } // Node is occupied?
    } // Iterate through octree

    // call post-traversal hook:
    handlePostNodeTraversal(rostime);

    // finish MarkerArray:
    if (publishMarkerArray)
    {
        for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
        {
            double size = m_octoMap->octree.getNodeSize(i);

            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].scale.x = size;
            occupiedNodesVis.markers[i].scale.y = size;
            occupiedNodesVis.markers[i].scale.z = size;
            occupiedNodesVis.markers[i].color = m_color;


            if (occupiedNodesVis.markers[i].points.size() > 0)
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
                occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        // Publish marker array
        m_markerPub.publish(occupiedNodesVis);
    }

    // finish pointcloud:
    if (publishPointCloud)
    {
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg (pclCloud, cloud);
        cloud.header.frame_id = m_worldFrameId;
        cloud.header.stamp = rostime;
        m_pointCloudPub.publish(cloud);
    }

    // Publish collision object
    if (publishCollisionObject)
        m_collisionObjectPub.publish(collisionObject);

    // Publish collision map
    if (m_publishCollisionMap )
    {
        m_cmap.header.frame_id = m_worldFrameId;
        m_cmap.header.stamp = rostime;
        m_collisionMapPub.publish( m_cmap );
    }


    // Finalize octomap publishing
    if (publishOctoMap)
        publishMap(rostime);


    // Compute and show elapsed time
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Map publishing in CButServer took %f sec", total_elapsed);

}

/**
  Reset octomap data member
*/
void CButServer::resetOctomap() {

    // TODO: replace with m_octoMap->clear() in version 1.4

    // Clear octomap
    if(m_octoMap)
        delete m_octoMap;

    // Create new octomap
    m_octoMap = new octomap::OcTreeROS(m_res);
    m_octoMap->octree.setProbHit(m_probHit);
    m_octoMap->octree.setProbMiss(m_probMiss);
    m_octoMap->octree.setClampingThresMin(m_thresMin);
    m_octoMap->octree.setClampingThresMax(m_thresMax);
    m_treeDepth = m_octoMap->octree.getTreeDepth();
    m_gridmap.info.resolution = m_res;
}

/**
  Publish octomap
 */
void CButServer::publishMap(const ros::Time& rostime) const{

    octomap_ros::OctomapBinary map;
    map.header.frame_id = m_worldFrameId;
    map.header.stamp = rostime;

    octomap::octomapMapToMsgData(m_octoMap->octree, map.data);

    m_binaryMapPub.publish(map);
}

/**
  Prepare publishing of data
  */
void CButServer::handlePreNodeTraversal(const ros::Time& rostime)
{
    m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

    if (m_publish2DMap)
    {
        // init projected 2D map:
        m_gridmap.header.frame_id = m_worldFrameId;
        m_gridmap.header.stamp = rostime;

        // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
        double minX, minY, minZ, maxX, maxY, maxZ;
        m_octoMap->octree.getMetricMin(minX, minY, minZ);
        m_octoMap->octree.getMetricMax(maxX, maxY, maxZ);

        octomap::point3d minPt(minX, minY, minZ);
        octomap::point3d maxPt(maxX, maxY, maxZ);
        octomap::OcTreeKey minKey, maxKey, curKey;

        // Try to create key
        if (!m_octoMap->octree.genKey(minPt, minKey))
        {
            ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
            return;
        }

        if (!m_octoMap->octree.genKey(maxPt, maxKey))
        {
            ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
            return;
        }

        ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

        // add padding if requested (= new min/maxPts in x&y):
        double halfPaddedX = 0.5*m_minSizeX;
        double halfPaddedY = 0.5*m_minSizeY;
        minX = std::min(minX, -halfPaddedX);
        maxX = std::max(maxX, halfPaddedX);
        minY = std::min(minY, -halfPaddedY);
        maxY = std::max(maxY, halfPaddedY);
        minPt = octomap::point3d(minX, minY, minZ);
        maxPt = octomap::point3d(maxX, maxY, maxZ);

        octomap::OcTreeKey paddedMaxKey;

        if (!m_octoMap->octree.genKey(minPt, m_paddedMinKey))
        {
            ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
            return;
        }

        if (!m_octoMap->octree.genKey(maxPt, paddedMaxKey))
        {
            ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
            return;
        }

        ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
        assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

        m_gridmap.info.width = paddedMaxKey[0] - m_paddedMinKey[0] +1;
        m_gridmap.info.height = paddedMaxKey[1] - m_paddedMinKey[1] +1;
        int mapOriginX = minKey[0] - m_paddedMinKey[0];
        int mapOriginY = minKey[1] - m_paddedMinKey[1];
        assert(mapOriginX >= 0 && mapOriginY >= 0);

        // might not exactly be min / max of octree:
        octomap::point3d origin;
        m_octoMap->octree.genCoords(m_paddedMinKey, m_treeDepth, origin);
        m_gridmap.info.origin.position.x = origin.x() - m_res*0.5;
        m_gridmap.info.origin.position.y = origin.y() - m_res*0.5;

        // Allocate space to hold the data (init to unknown)
        m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);
    }

}

/**
  Do publishing...
  */
void CButServer::handlePostNodeTraversal(const ros::Time& rostime){

    if (m_publish2DMap)
        m_mapPub.publish(m_gridmap);

}

/**
  Node is ocupied - publish it
  */
void CButServer::handleOccupiedNode(const octomap::OcTreeROS::OcTreeType::iterator& it){

    // update 2D map (occupied always overrides):
    if (m_publish2DMap){

        if (it.getDepth() == m_treeDepth){

            octomap::OcTreeKey nKey = it.getKey(); // TODO: remove intermedate obj (1.4)
            int i = nKey[0] - m_paddedMinKey[0];
            int j = nKey[1] - m_paddedMinKey[1];
            m_gridmap.data[m_gridmap.info.width*j + i] = 100;

        } else{

            int intSize = 1 << (m_treeDepth - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx=0; dx < intSize; dx++){
                int i = minKey[0]+dx - m_paddedMinKey[0];
                for(int dy=0; dy < intSize; dy++){
                    int j = minKey[1]+dy - m_paddedMinKey[1];
                    m_gridmap.data[m_gridmap.info.width*j + i] = 100;
                }
            }
        }
    }

    if( m_publishCollisionMap )
    {
//        mapping_msgs::OrientedBoundingBox box;
        arm_navigation_msgs::OrientedBoundingBox box;
        double size = it.getSize();
        box.extents.x = box.extents.y = box.extents.z = size;
        box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
        box.angle = 0.0;
        box.center.x = it.getX();
        box.center.y = it.getY();
        box.center.z = it.getZ();
        m_cmap.boxes.push_back(box);
    }

}

/**
  Node is not ocupied
  */
void CButServer::handleFreeNode(const octomap::OcTreeROS::OcTreeType::iterator& it){

    if (m_publish2DMap){
        if (it.getDepth() == m_treeDepth){
            octomap::OcTreeKey nKey = it.getKey(); //TODO: remove intermedate obj (1.4)
            int i = nKey[0] - m_paddedMinKey[0];
            int j = nKey[1] - m_paddedMinKey[1];
            if (m_gridmap.data[m_gridmap.info.width*j + i] == -1){
                m_gridmap.data[m_gridmap.info.width*j + i] = 0;
            }
        } else{
            int intSize = 1 << (m_treeDepth - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx=0; dx < intSize; dx++){
                int i = minKey[0]+dx - m_paddedMinKey[0];
                for(int dy=0; dy < intSize; dy++){
                    int j = minKey[1]+dy - m_paddedMinKey[1];
                    if (m_gridmap.data[m_gridmap.info.width*j + i] == -1){
                        m_gridmap.data[m_gridmap.info.width*j + i] = 0;
                    }
                }
            }
        }
    }
}


/**
  Is this node speckle?
  */
bool CButServer::isSpeckleNode(const octomap::OcTreeKey&nKey) const {
    octomap::OcTreeKey key;
    bool neighborFound = false;
    for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
        for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
            for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
                if (key != nKey){
                    octomap::OcTreeNode* node = m_octoMap->octree.search(key);
                    if (node && m_octoMap->octree.isNodeOccupied(node)){
                        // we have a neighbor => break!
                        neighborFound = true;
                    }
                }
            }
        }
    }

    return neighborFound;
}

/**
  Generate color from the height
  */
std_msgs::ColorRGBA CButServer::heightMapColor(double h) const {

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
        color.r = v; color.g = n; color.b = m;
        break;
    case 1:
        color.r = n; color.g = v; color.b = m;
        break;
    case 2:
        color.r = m; color.g = v; color.b = n;
        break;
    case 3:
        color.r = m; color.g = n; color.b = v;
        break;
    case 4:
        color.r = n; color.g = m; color.b = v;
        break;
    case 5:
        color.r = v; color.g = m; color.b = n;
        break;
    default:
        color.r = 1; color.g = 0.5; color.b = 0.5;
        break;
    }

    return color;
}

/**
  Cloud insertion callback
  */
void CButServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{

    ros::WallTime startTime = ros::WallTime::now();


    //
    // ground filtering in base frame
    //
    tPCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf, sensorToBaseTf, baseToWorldTf;
    try {
        m_tfListener.waitForTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));

        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
        m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
        m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
        std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
        return;
    }
    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);

    // filter height and range, also removes NANs:
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);

    tPCLPointCloud pc_ground; // segmented ground plane
    tPCLPointCloud pc_nonground; // everything else

    if (m_filterGroundPlane){
        filterGroundPlane(pc, pc_ground, pc_nonground);
    } else {
        pc_nonground = pc;
        pc_ground.header = pc.header;
        pc_nonground.header = pc.header;
    }

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);


    insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

    // Publish new data
    publishAll(cloud->header.stamp);
}

/**
    Filter ground/nonground
  */
void CButServer::filterGroundPlane(const tPCLPointCloud& pc, tPCLPointCloud& ground, tPCLPointCloud& nonground) const{
    ground.header = pc.header;
    nonground.header = pc.header;

    if (pc.size() < 50){
        ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
        nonground = pc;
    } else {
        // plane detection for ground plane removal:
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        // Create the segmentation object and set up:
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(200);
        seg.setDistanceThreshold (m_groundFilterDistance);
        seg.setAxis(Eigen::Vector3f(0,0,1));
        seg.setEpsAngle(m_groundFilterAngle);


        tPCLPointCloud cloud_filtered(pc);
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        bool groundPlaneFound = false;

        while(cloud_filtered.size() > 10 && !groundPlaneFound){
            seg.setInputCloud(cloud_filtered.makeShared());
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0){
                ROS_WARN("No plane found in cloud.");

                break;
            }

            extract.setInputCloud(cloud_filtered.makeShared());
            extract.setIndices(inliers);

            if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
                ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                          coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
                extract.setNegative (false);
                extract.filter (ground);

                // remove ground points from full pointcloud:
                // workaround for PCL bug:
                if(inliers->indices.size() != cloud_filtered.size()){
                    extract.setNegative(true);
                    tPCLPointCloud cloud_out;
                    extract.filter(cloud_out);
                    nonground += cloud_out;
                    cloud_filtered = cloud_out;
                }

                groundPlaneFound = true;
            } else{
                ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                          coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
                pcl::PointCloud<pcl::PointXYZ> cloud_out;
                extract.setNegative (false);
                extract.filter(cloud_out);
                nonground +=cloud_out;
                // debug
                //            pcl::PCDWriter writer;
                //            writer.write<pcl::PointXYZ>("nonground_plane.pcd",cloud_out, false);

                // remove current plane from scan for next iteration:
                // workaround for PCL bug:
                if(inliers->indices.size() != cloud_filtered.size()){
                    extract.setNegative(true);
                    cloud_out.points.clear();
                    extract.filter(cloud_out);
                    cloud_filtered = cloud_out;
                } else{
                    cloud_filtered.points.clear();
                }
            }

        }
        // TODO: also do this if overall starting pointcloud too small?
        if (!groundPlaneFound){ // no plane found or remaining points too small
            ROS_WARN("No ground plane found in scan");

            // do a rough fitlering on height to prevent spurious obstacles
            pcl::PassThrough<pcl::PointXYZ> second_pass;
            second_pass.setFilterFieldName("z");
            second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
            second_pass.setInputCloud(pc.makeShared());
            second_pass.filter(ground);

            second_pass.setFilterLimitsNegative (true);
            second_pass.filter(nonground);
        }

        // debug:
        //        pcl::PCDWriter writer;
        //        if (pc_ground.size() > 0)
        //          writer.write<pcl::PointXYZ>("ground.pcd",pc_ground, false);
        //        if (pc_nonground.size() > 0)
        //          writer.write<pcl::PointXYZ>("nonground.pcd",pc_nonground, false);

    }


}

/**
 * @brief Reset octomap - service callback
 *
 */
bool CButServer::resetOctomapCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    std::cerr << "Reset octomap service called..." << std::endl;
    resetOctomap();
    return true;
}

/**
 * @brief Insert or modify plane array
 *
 * @param pa Array of planes
 */
bool CButServer::insertPlaneCallback( srs_env_model::AddPlanes::Request & req, srs_env_model::AddPlanes::Response & res )
{
    std::cerr << "Inset plane called" << std::endl;
    // Get plane array
    srs_env_model::PlaneArray & planea( req.plane_array );
    m_planesFrameId = planea.header.frame_id;
    std::vector< srs_env_model::PlaneDesc > & planes( planea.planes );
    std::vector< srs_env_model::PlaneDesc >::iterator i;

    for( i = planes.begin(); i != planes.end(); ++i )
    {
        operatePlane( *i );
    }

    return true;
}

/**
 * @brief Insert/modify/remove plane
 *
 * @param plane Plane
 */
void CButServer::operatePlane( const srs_env_model::PlaneDesc & plane )
{


    std::string name;

    switch( plane.flags )
    {
    case srs_env_model::PlaneDesc::INSERT:
        name = getUniqueName();
        m_dataPlanes[plane.id] = tNamedPlane( name, plane );

        addPlaneSrvCall( plane, name );
        break;

    case srs_env_model::PlaneDesc::MODIFY:
        m_dataPlanes[plane.id].second = plane;
        name = m_dataPlanes[plane.id].first;

        // call remove plane, add plane
        removePlaneSrvCall( plane, name );
        addPlaneSrvCall( plane, name );
        break;

    case srs_env_model::PlaneDesc::REMOVE:
        name = m_dataPlanes[plane.id].first;

        m_dataPlanes.erase( plane.id );
        // call remove plane
        removePlaneSrvCall( plane, name );
        break;


    default:
        break;
    }

    std::cerr << "Current planes count: " << m_dataPlanes.size() << std::endl;
}


/**
 * @brief Service helper - add plane
 *
 * @param plane Added plane
 */
void CButServer::addPlaneSrvCall( const srs_env_model::PlaneDesc & plane, const std::string & name )
{
    // Create service
    srs_env_model::AddPlane addPlaneSrv;

    // Modify service
    addPlaneSrv.request.name = name;
    addPlaneSrv.request.frame_id = m_planesFrameId;
    addPlaneSrv.request.pose = plane.pose;
    addPlaneSrv.request.scale = plane.scale;
    addPlaneSrv.request.color.r = 1.0;
    addPlaneSrv.request.color.g = 0.0;
    addPlaneSrv.request.color.b = 0.0;
    addPlaneSrv.request.color.a = 0.8;

    m_addInteractivePlaneService.call( addPlaneSrv );

    std::cerr << "Adding plane: " << name << ", frame: " << m_planesFrameId << ", pose: " << plane.pose << "scale: " << plane.scale << std::endl;
}

/**
 * @brief Service helper - remove plane
 *
 * @param plane Added plane
 */
void CButServer::removePlaneSrvCall( const srs_env_model::PlaneDesc & plane, const std::string & name )
{
    // Create service
    srs_env_model::RemoveObject removeObjectSrv;

    // Modify service
    removeObjectSrv.request.name = name;

    m_removeInteractiveMarkerService.call( removeObjectSrv );

    std::cerr << "Removing plane: " << name;
}

/**
 *  @brief Get unique string (used as interactive marker name)
 */
std::string CButServer::getUniqueName()
{
    std::stringstream ss;
    ss << "imn" << ++m_uniqueNameCounter;
    return ss.str();
}

