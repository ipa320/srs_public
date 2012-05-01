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

#include <but_server/plugins/OctoMapPlugin.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

// Filtering
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Interactive marker
#include <srs_interaction_primitives/AddUnknownObject.h>

#define OCTOMAP_FRAME_ID std::string("/map")
#define OCTOMAP_PUBLISHER_NAME std::string("butsrv_binary_octomap")
#define CAMERA_INFO_TOPIC_NAME std::string("/cam3d/camera_info")
#define MARKERS_TOPIC_NAME std::string("visualization_marker")
#define REMOVE_CUBE_SERVICE_NAME std::string("remove_cube")

void srs::COctoMapPlugin::setDefaults()
{
	// Set octomap parameters
	m_mapParameters.resolution = 0.1;
	m_mapParameters.treeDepth = 0;
	m_mapParameters.probHit = 0.7;         	// Probability of node, if node is occupied: 0.7
	m_mapParameters.probMiss = 0.4;        	// Probability of node, if node is free: 0.4
	m_mapParameters.thresMin = 0.12;		// Clamping minimum threshold: 0.1192;
	m_mapParameters.thresMax = 0.97; 		// Clamping maximum threshold: 0.971;
	m_mapParameters.thresOccupancy = 0.5; 	// Occupied node threshold: 0.5
	m_mapParameters.maxRange = -1.0;

	// Set ground filtering parameters
	m_filterGroundPlane = false;
	m_groundFilterDistance = 0.04;
	m_groundFilterAngle = 0.15;
	m_groundFilterPlaneDistance = 0.07;
	m_removeSpecles = false;

	m_mapParameters.frameId = "/map";

	m_bPublishOctomap = true;

	// Filtering
	m_bRemoveOutdated = true;
	m_bCamModelInitialized = false;
	m_camera_info_topic = CAMERA_INFO_TOPIC_NAME;
	m_bVisualizeMarkers = true;
	m_markers_topic_name = MARKERS_TOPIC_NAME;

	// CTestingPolymesh::tQuaternion quat(Eigen::AngleAxisf(0.33*M_PI, Eigen::Vector3f::UnitZ()) ) ;

	m_removeTester = 0; //new CTestingPolymesh(CTestingPolymesh::tPoint( 1.0, 1.0, 0.5 ), quat, CTestingPolymesh::tPoint( 1.0, 1.5, 2.0 ));

	m_testerLife = 10;

}

srs::COctoMapPlugin::COctoMapPlugin(const std::string & name)
: srs::CServerPluginBase(name)
, filecounter( 0 )
{
	//
	setDefaults();

	// Create octomap
	m_data = new tButServerOcMap(m_mapParameters.resolution);
	assert( m_data != 0 );

	// Set octomap parameters
	m_data->octree.setProbHit(m_mapParameters.probHit);
	m_data->octree.setProbMiss(m_mapParameters.probMiss);
	m_data->octree.setClampingThresMin(m_mapParameters.thresMin);
	m_data->octree.setClampingThresMax(m_mapParameters.thresMax);
	m_data->octree.setOccupancyThres( m_mapParameters.thresOccupancy );
	m_mapParameters.treeDepth = m_data->octree.getTreeDepth();
	m_mapParameters.map = m_data;
}


srs::COctoMapPlugin::COctoMapPlugin( const std::string & name, const std::string & filename )
: srs::CServerPluginBase(name)
{
	setDefaults();

	// Create octomap
	m_data = new tButServerOcMap(m_mapParameters.resolution);
	assert( m_data != 0 );

	// Set octomap parameters
	m_data->octree.setProbHit(m_mapParameters.probHit);
	m_data->octree.setProbMiss(m_mapParameters.probMiss);
	m_data->octree.setClampingThresMin(m_mapParameters.thresMin);
	m_data->octree.setClampingThresMax(m_mapParameters.thresMax);
	m_mapParameters.treeDepth = m_data->octree.getTreeDepth();

	// is filename valid?
	if( filename.length() > 0 )
	{
		// Try to load data
		if (m_data->octree.readBinary(filename)) {
			ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_data->octree.size());

			// get tree depth
			m_mapParameters.treeDepth = m_data->octree.getTreeDepth();

			// get resolution
			m_mapParameters.resolution = m_data->octree.getResolution();

			// We have new data
			invalidate();

		} else {

			// Something is wrong - cannot load data...
			ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
			PERROR( "Transform error.");
			exit(-1);
		}
	}
}

/**
 * Destructor
 */
srs::COctoMapPlugin::~COctoMapPlugin()
{
	// Remove tester
	if( m_removeTester != 0 )
		delete m_removeTester;
}

//! Initialize plugin - called in server constructor
void srs::COctoMapPlugin::init(ros::NodeHandle & node_handle)
{
	PERROR( "Initializing OctoMapPlugin" );

	reset();

	// Load parameters from the parameter server
	node_handle.param("resolution", m_mapParameters.resolution, m_mapParameters.resolution);
	node_handle.param("sensor_model/hit", m_mapParameters.probHit, m_mapParameters.probHit);
	node_handle.param("sensor_model/miss", m_mapParameters.probMiss, m_mapParameters.probMiss);
	node_handle.param("sensor_model/min", m_mapParameters.thresMin, m_mapParameters.thresMin);
	node_handle.param("sensor_model/max", m_mapParameters.thresMax, m_mapParameters.thresMax);
	node_handle.param("max_range", m_mapParameters.maxRange, m_mapParameters.maxRange);

	// Filtering presets
	{
		node_handle.param("camera_info_topic", m_camera_info_topic, m_camera_info_topic);
		node_handle.param("visualize_markers", m_bVisualizeMarkers, m_bVisualizeMarkers );
		node_handle.param("markers_topic", m_markers_topic_name, m_markers_topic_name );
		// stereo cam params for sensor cone:
		node_handle.param<int>("camera_stereo_offset_left", m_camera_stereo_offset_left, 128);
		node_handle.param<int>("camera_stereo_offset_right", m_camera_stereo_offset_right, 0);
	}

	// Set octomap parameters...
	{
	    m_data->octree.setResolution(m_mapParameters.resolution);
	    m_data->octree.setProbHit(m_mapParameters.probHit);
	    m_data->octree.setProbMiss(m_mapParameters.probMiss);
	    m_data->octree.setClampingThresMin(m_mapParameters.thresMin);
	    m_data->octree.setClampingThresMax(m_mapParameters.thresMax);
	}

	// Should ground plane be filtered?
	node_handle.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);

	// distance of points from plane for RANSAC
	node_handle.param("ground_filter/distance", m_groundFilterDistance,
			m_groundFilterDistance);
	// angular derivation of found plane:
	node_handle.param("ground_filter/angle", m_groundFilterAngle,
			m_groundFilterAngle);
	// distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
	node_handle.param("ground_filter/plane_distance",
			m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

	// Octomap publishing topic
	node_handle.param("octomap_publishing_topic", m_ocPublisherName, OCTOMAP_PUBLISHER_NAME );

	// Advertise services
	m_serviceResetOctomap = node_handle.advertiseService("reset_octomap",
			&srs::COctoMapPlugin::resetOctomapCB, this);

	m_serviceRemoveCube =   node_handle.advertiseService( REMOVE_CUBE_SERVICE_NAME,
			&srs::COctoMapPlugin::removeCubeCB, this );

	// Create publisher
	m_ocPublisher = node_handle.advertise<octomap_ros::OctomapBinary>(m_ocPublisherName, 100, m_latchedTopics);

	// Add camera info subscriber
	m_ciSubscriber = new ros::Subscriber;
	*m_ciSubscriber = node_handle.subscribe( m_camera_info_topic, 10, &srs::COctoMapPlugin::cameraInfoCB, this );

	// If should publish, create markers publisher
	m_markerPublisher = node_handle.advertise<visualization_msgs::Marker>(m_markers_topic_name, 10);

	PERROR( "OctoMapPlugin initialized..." );
}


void srs::COctoMapPlugin::insertCloud(const tPointCloud & cloud)
{
	if( !useFrame() )
		return;

	// Lock data
	boost::mutex::scoped_lock lock( m_lockData );

	ros::WallTime startTime = ros::WallTime::now();

	tPointCloud pc_ground; // segmented ground plane
	tPointCloud pc_nonground; // everything else

	if (m_filterGroundPlane) {
		filterGroundPlane(cloud, pc_ground, pc_nonground);

	} else {
		pc_nonground = cloud;
		pc_ground.clear();
		pc_ground.header = cloud.header;
		pc_nonground.header = cloud.header;
	}

	tf::StampedTransform cloudToMapTf;

	// Get transforms
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_mapParameters.frameId, cloud.header.frame_id,
				cloud.header.stamp, ros::Duration(0.2));

		m_tfListener.lookupTransform(m_mapParameters.frameId, cloud.header.frame_id,
				cloud.header.stamp, cloudToMapTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		PERROR( "Transform error.");
		return;
	}


	// transform clouds to world frame for insertion
	if( m_mapParameters.frameId != cloud.header.frame_id )
	{
		Eigen::Matrix4f c2mTM;

		pcl_ros::transformAsMatrix(cloudToMapTf, c2mTM );
		pcl::transformPointCloud(pc_ground, pc_ground, c2mTM);
		pcl::transformPointCloud(pc_nonground, pc_nonground, c2mTM);

	}

	pc_ground.header = cloud.header;
	pc_ground.header.frame_id = m_mapParameters.frameId;

	pc_nonground.header = cloud.header;
	pc_nonground.header.frame_id = m_mapParameters.frameId;

	insertScan(cloudToMapTf.getOrigin(), pc_ground, pc_nonground);

	if( m_removeSpecles )
	{
		degradeSingleSpeckles();
	}

	if( m_bRemoveOutdated )
	{
		octomap::point3d sensor_origin = getSensorOrigin(cloud.header);
		octomap::pose6d  sensor_pose(sensor_origin.x(), sensor_origin.y(), sensor_origin.z(), 0, 0, 0);


		degradeOutdatedRaycasting(cloud.header, sensor_origin );
	}

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Point cloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(),
			pc_nonground.size(), total_elapsed);

	if( m_removeTester != 0 )
	{
		long removed = doObjectTesting( m_removeTester );

//		PERROR( "Removed leafs: " << removed);

		if( removed > 0 )
			m_data->octree.prune();

		--m_testerLifeCounter;

		if( m_testerLifeCounter <= 0 )
		{
			delete m_removeTester;
			m_removeTester = 0;
		}
	}
	// Publish new data
	invalidate();
}


/**
 * Insert pointcloud scan TODO: Modify to add ground
 */
void srs::COctoMapPlugin::insertScan(const tf::Point & sensorOriginTf, const tPointCloud & ground, const tPointCloud & nonground)
{
	octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

	double maxRange(m_mapParameters.maxRange);
//	octomap::Pointcloud pcNonground;
//	octomap::pointcloudPCLToOctomap( nonground, pcNonground );
	m_data->octree.insertColoredScan(nonground, sensorOrigin, maxRange, true);
}



void srs::COctoMapPlugin::filterGroundPlane(const tPointCloud & pc, tPointCloud & ground, tPointCloud & nonground) const
{
	ground.header = pc.header;
	nonground.header = pc.header;

	if (pc.size() < 50)
	{
		ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
		nonground = pc;
	}
	else
	{
		// plane detection for ground plane removal:
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		// Create the segmentation object and set up:
		pcl::SACSegmentation<tPclPoint> seg;
		seg.setOptimizeCoefficients(true);
		// TODO: maybe a filtering based on the surface normals might be more robust / accurate?
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(200);
		seg.setDistanceThreshold(m_groundFilterDistance);
		seg.setAxis(Eigen::Vector3f(0, 0, 1));
		seg.setEpsAngle(m_groundFilterAngle);

		tPointCloud cloud_filtered(pc);
		// Create the filtering object
		pcl::ExtractIndices<tPclPoint> extract;
		bool groundPlaneFound = false;

		while (cloud_filtered.size() > 10 && !groundPlaneFound) {
			seg.setInputCloud(cloud_filtered.makeShared());
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0) {
				ROS_WARN("No plane found in cloud.");

				break;
			}

			extract.setInputCloud(cloud_filtered.makeShared());
			extract.setIndices(inliers);

			if (std::abs(coefficients->values.at(3))
			< m_groundFilterPlaneDistance) {
				ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
						cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1),
						coefficients->values.at(2), coefficients->values.at(3));
				extract.setNegative(false);
				extract.filter(ground);

				// remove ground points from full pointcloud:
				// workaround for PCL bug:
				if (inliers->indices.size() != cloud_filtered.size()) {
					extract.setNegative(true);
					tPointCloud cloud_out;
					extract.filter(cloud_out);
					nonground += cloud_out;
					cloud_filtered = cloud_out;
				}

				groundPlaneFound = true;
			} else {
				ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
						cloud_filtered.size(), coefficients->values.at(0), coefficients->values.at(1),
						coefficients->values.at(2), coefficients->values.at(3));
				tPointCloud cloud_out;
				extract.setNegative(false);
				extract.filter(cloud_out);
				nonground += cloud_out;
				// debug
				//            pcl::PCDWriter writer;
				//            writer.write<tPclPoint>("nonground_plane.pcd",cloud_out, false);

				// remove current plane from scan for next iteration:
				// workaround for PCL bug:
				if (inliers->indices.size() != cloud_filtered.size()) {
					extract.setNegative(true);
					cloud_out.points.clear();
					extract.filter(cloud_out);
					cloud_filtered = cloud_out;
				} else {
					cloud_filtered.points.clear();
				}
			}

		}
		// TODO: also do this if overall starting pointcloud too small?
		if (!groundPlaneFound) { // no plane found or remaining points too small
			ROS_WARN("No ground plane found in scan");

			// do a rough filtering on height to prevent spurious obstacles
			pcl::PassThrough<tPclPoint> second_pass;
			second_pass.setFilterFieldName("z");
			second_pass.setFilterLimits(-m_groundFilterPlaneDistance,
					m_groundFilterPlaneDistance);
			second_pass.setInputCloud(pc.makeShared());
			second_pass.filter(ground);

			second_pass.setFilterLimitsNegative(true);
			second_pass.filter(nonground);
		}

		// debug:
		//        pcl::PCDWriter writer;
		//        if (pc_ground.size() > 0)
		//          writer.write<tPclPoint>("ground.pcd",pc_ground, false);
		//        if (pc_nonground.size() > 0)
		//          writer.write<tPclPoint>("nonground.pcd",pc_nonground, false);

	}
}

void srs::COctoMapPlugin::reset()
{
	// Lock data
	boost::mutex::scoped_lock lock( m_lockData );

	m_data->octree.clear();
}

///////////////////////////////////////////////////////////////////////////////
//  OCTOMAP CRAWLING
///////////////////////////////////////////////////////////////////////////////

/// Crawl octomap
void srs::COctoMapPlugin::crawl( const ros::Time & currentTime )
{
	// Lock data
	boost::mutex::scoped_lock lock( m_lockData );

	// Fill needed structures
	onCrawlStart(currentTime);

	// Crawl through nodes
	for (srs::tButServerOcTree::leaf_iterator it = m_data->octree.begin_leafs(), end = m_data->octree.end_leafs(); it != end; ++it)
		{

			// call general hook:
			handleNode(it, m_mapParameters);

			// Node is occupied?
			if (m_data->octree.isNodeOccupied(*it))
			{
				handleOccupiedNode(it, m_mapParameters);
			} else { // node not occupied => mark as free in 2D map if unknown so far

				handleFreeNode(it, m_mapParameters);
			} // Node is occupied?
		} // Iterate through octree

	handlePostNodeTraversal(m_mapParameters);
/*
	std::stringstream ss;
	ss << "/home/wik/output/octomap" << filecounter << ".bt";

	PERROR( "Writing: " << ss.str() );
	m_data->octree.writeBinary( ss.str() );
	++filecounter;
*/
}

/// On octomap crawling start
void srs::COctoMapPlugin::onCrawlStart(const ros::Time & currentTime)
{

	fillMapParameters(currentTime);

	// Call signal
	m_sigOnStart( m_mapParameters );
}

/// Handle node
void srs::COctoMapPlugin::handleNode(tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	m_sigOnNode( it, mp );
}

/// Handle free node
void srs::COctoMapPlugin::handleFreeNode(tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	m_sigOnFreeNode( it, mp );
}

/// Handle occupied node
void srs::COctoMapPlugin::handleOccupiedNode(tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	m_sigOnOccupiedNode( it, mp );
}

void srs::COctoMapPlugin::handlePostNodeTraversal(const SMapParameters & mp)
{
	m_sigOnPost( mp );
}

//! Should plugin publish data?
bool srs::COctoMapPlugin::shouldPublish()
{
	return( m_bPublishOctomap && m_ocPublisher.getNumSubscribers() > 0 );
}

void srs::COctoMapPlugin::onPublish(const ros::Time & timestamp)
{
	// Lock data
	boost::mutex::scoped_lock lock( m_lockData );
	octomap_ros::OctomapBinary map;
	map.header.frame_id = m_mapParameters.frameId;
	map.header.stamp = timestamp;

	octomap::octomapMapToMsgData(m_data->octree, map.data);

	m_ocPublisher.publish(map);
}

/// Fill map parameters
void srs::COctoMapPlugin::fillMapParameters(const ros::Time & time)
{

	m_mapParameters.currentTime = time;
	m_mapParameters.mapSize = m_data->octree.size();
	m_mapParameters.treeDepth = m_data->octree.getTreeDepth();

}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Reset octomap - service callback
 *
 */
bool srs::COctoMapPlugin::resetOctomapCB(std_srvs::Empty::Request& request,	std_srvs::Empty::Response& response)
{
	std::cerr << "Reset octomap service called..." << std::endl;
	reset();
	return true;
}

// ============================================================================
// Filtering

void srs::COctoMapPlugin::cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
	PERROR( std::endl << std::endl << "CAMERA INFO CALLBACK" << std::endl << std::endl)
	// Get camera info
	ROS_DEBUG("OctMapPlugin: Set camera info: %d x %d\n", cam_info->height, cam_info->width);
	m_camera_model.fromCameraInfo(*cam_info);
	m_camera_size = m_camera_model.fullResolution();

	// Set flag
	m_bCamModelInitialized = true;

	// Disconnect subscriber
	delete m_ciSubscriber;
}

/**
 * Remove outdated nodes
 */
void srs::COctoMapPlugin::degradeOutdatedRaycasting( const std_msgs::Header& sensor_header, const octomap::point3d& sensor_origin )
{
	if (!m_bCamModelInitialized)
	{
		ROS_INFO ("ERROR: camera model not initialized.");
		return;
	}

	// Get tree
	tButServerOcTree & tree ( m_data->octree );

	tf::StampedTransform trans;
	m_tfListener.lookupTransform (sensor_header.frame_id, m_mapParameters.frameId, sensor_header.stamp, trans);
	tf::Transform to_sensor = trans;

	// compute bbx from sensor cone
	octomap::point3d min;
	octomap::point3d max;
	computeBBX(sensor_header, min, max);

	unsigned query_time = time(NULL);
	unsigned max_update_time = 1;
	for(tButServerOcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min,max),
			end=tree.end_leafs_bbx(); it!= end; ++it)
	{
		if (tree.isNodeOccupied(*it) &&
				((query_time - it->getTimestamp()) > max_update_time))
		{
			tf::Point pos(it.getX(), it.getY(), it.getZ());
			tf::Point posRel = to_sensor(pos);
			cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d(posRel.x(), posRel.y(), posRel.z()));

			// ignore point if not in sensor cone
			if (!inSensorCone(uv))
				continue;

			// ignore point if it is occluded in the map
			if (isOccludedMap(sensor_origin, it.getCoordinate()))
				continue;

			// otherwise: degrade node
			tree.integrateMissNoTime(&*it);

		}
	}
}

/**
 * Remove speckles
 */
void srs::COctoMapPlugin::degradeSingleSpeckles()
{
	tButServerOcTree & tree( m_data->octree );

	for(tButServerOcTree::leaf_iterator it = m_data->octree.begin_leafs(),
			end=m_data->octree.end_leafs(); it!= end; ++it)
	{
		// Test if node is occupied
		if (m_data->octree.isNodeOccupied(*it))
		{
			octomap::OcTreeKey nKey = it.getKey();
			octomap::OcTreeKey key;
			bool neighborFound = false;

			// Find neighbours
			for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
				for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
					for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
						if (key != nKey){
							tButServerOcTree::NodeType* node = tree.search(key);
							if (node && tree.isNodeOccupied(node)){
								// we have a neighbor => break!
								neighborFound = true;
							}
						}
					}
				}
			}

			// done with search, see if found and degrade otherwise:
			if (!neighborFound){
				ROS_DEBUG("Degrading single speckle at (%f,%f,%f)", it.getX(), it.getY(), it.getZ());

				// Remove it...
				m_data->octree.integrateMissNoTime(&*it);
			}

		}
	}
}

/**
 * Compute bounding box from the sensor position and cone
 */
void srs::COctoMapPlugin::computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max) {

  std::string sensor_frame = sensor_header.frame_id;

  //  transform sensor FOV
  geometry_msgs::PointStamped stamped_in;
  geometry_msgs::PointStamped stamped_out;
  stamped_in.header = sensor_header;
  stamped_in.header.frame_id = sensor_frame;

  // get max 3d points from camera at 0.5m and 5m.
  geometry_msgs::Point p[8];

  // define min/max 2d points
  cv::Point2d uv [4];
  uv[0].x = m_camera_stereo_offset_left;
  uv[0].y = 0;
  uv[1].x = m_camera_size.width + m_camera_stereo_offset_right;
  uv[1].y = 0;
  uv[2].x = m_camera_size.width + m_camera_stereo_offset_right;
  uv[2].y = m_camera_size.height;
  uv[3].x = m_camera_stereo_offset_left;
  uv[3].y = m_camera_size.height;

  // transform to 3d space
  cv::Point3d xyz [4];
  for (int i=0;i<4;i++) {
	xyz[i] = m_camera_model.projectPixelTo3dRay(uv[i]);
    cv::Point3d xyz_05 = xyz[i] * 0.5;
    xyz[i] *= 5.; // 5meters
    p[i].x = xyz[i].x;
    p[i].y = xyz[i].y;
    p[i].z = xyz[i].z;
    p[i+4].x = xyz_05.x;
    p[i+4].y = xyz_05.y;
    p[i+4].z = xyz_05.z;
  }

  // transform to world coodinates and find axis-aligned bbx
  bbx_min.x() = bbx_min.y() = bbx_min.z() = 1e6;
  bbx_max.x() = bbx_max.y() = bbx_max.z() = -1e6;
  for (int i=0; i<8; i++) {
    stamped_in.point = p[i];
    m_tfListener.transformPoint(m_mapParameters.frameId, stamped_in, stamped_out);
    p[i].x = stamped_out.point.x;
    p[i].y = stamped_out.point.y;
    p[i].z = stamped_out.point.z;
    if (p[i].x < bbx_min.x()) bbx_min.x() = p[i].x;
    if (p[i].y < bbx_min.y()) bbx_min.y() = p[i].y;
    if (p[i].z < bbx_min.z()) bbx_min.z() = p[i].z;
    if (p[i].x > bbx_max.x()) bbx_max.x() = p[i].x;
    if (p[i].y > bbx_max.y()) bbx_max.y() = p[i].y;
    if (p[i].z > bbx_max.z()) bbx_max.z() = p[i].z;
  }

  // Should be markers visualized
  if( !m_bVisualizeMarkers )
	  return;

  // // visualize axis-aligned querying bbx
  visualization_msgs::Marker bbx;
  bbx.header.frame_id = m_mapParameters.frameId;
  bbx.header.stamp = ros::Time::now();
  bbx.ns = "OCM_plugin";
  bbx.id = 1;
  bbx.action = visualization_msgs::Marker::ADD;
  bbx.type = visualization_msgs::Marker::CUBE;
  bbx.pose.orientation.w = 1.0;
  bbx.pose.position.x = (bbx_min.x() + bbx_max.x()) / 2.;
  bbx.pose.position.y = (bbx_min.y() + bbx_max.y()) / 2.;
  bbx.pose.position.z = (bbx_min.z() + bbx_max.z()) / 2.;
  bbx.scale.x = bbx_max.x()-bbx_min.x();
  bbx.scale.y = bbx_max.y()-bbx_min.y();
  bbx.scale.z = bbx_max.z()-bbx_min.z();
  bbx.color.g = 1;
  bbx.color.a = 0.3;
  m_markerPublisher.publish(bbx);


  // visualize sensor cone
  visualization_msgs::Marker bbx_points;
  bbx_points.header.frame_id = m_mapParameters.frameId;
  bbx_points.header.stamp = ros::Time::now();
  bbx_points.ns = "OCM_plugin";
  bbx_points.id = 2;
  bbx_points.action = visualization_msgs::Marker::ADD;
  bbx_points.type = visualization_msgs::Marker::LINE_STRIP;
  bbx_points.pose.orientation.w = 1.0;
  bbx_points.scale.x = 0.02;
  bbx_points.scale.y = 0.02;
  bbx_points.color.g = 1;
  bbx_points.color.a = 0.3;
  bbx_points.points.push_back(p[0]);
  bbx_points.points.push_back(p[1]);
  bbx_points.points.push_back(p[2]);
  bbx_points.points.push_back(p[3]);
  bbx_points.points.push_back(p[0]);
  bbx_points.points.push_back(p[4]);
  bbx_points.points.push_back(p[5]);
  bbx_points.points.push_back(p[6]);
  bbx_points.points.push_back(p[7]);
  bbx_points.points.push_back(p[4]);
  bbx_points.points.push_back(p[7]);
  bbx_points.points.push_back(p[3]);
  bbx_points.points.push_back(p[2]);
  bbx_points.points.push_back(p[6]);
  bbx_points.points.push_back(p[5]);
  bbx_points.points.push_back(p[1]);
  m_markerPublisher.publish(bbx_points);
}

bool srs::COctoMapPlugin::inSensorCone(const cv::Point2d& uv) const
{
	// Check if projected 2D coordinate in pixel range.
	// This check is a little more restrictive than it should be by using
	// 1 pixel less to account for rounding / discretization errors.
	// Otherwise points on the corner are accounted to be in the sensor cone.
	return ( (uv.x > m_camera_stereo_offset_left+1)
			&& (uv.x < m_camera_size.width + m_camera_stereo_offset_right - 2)
			&& (uv.y > 1)
			&& (uv.y < m_camera_size.height-2) );
}

/**
 * Return true, if occupied cell is between origin and p
 */
bool srs::COctoMapPlugin::isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p) const {

  octomap::point3d direction (p-sensor_origin);
  octomap::point3d obstacle;
  double range = direction.norm() - m_mapParameters.resolution;

  if (m_data->octree.castRay(sensor_origin, direction, obstacle, true, range)) {
    // fprintf(stderr, "<%.2f , %.2f , %.2f> -> <%.2f , %.2f , %.2f> // obs at: <%.2f , %.2f , %.2f>, range: %.2f\n",
    //         sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
    //         p.x(), p.y(), p.z(),
    //         obstacle.x(), obstacle.y(), obstacle.z(), (obstacle-p).norm());
    return true;
  }
  return false;
}

octomap::point3d srs::COctoMapPlugin::getSensorOrigin(const std_msgs::Header& sensor_header)
{
	geometry_msgs::PointStamped stamped_in;
	geometry_msgs::PointStamped stamped_out;
	stamped_in.header = sensor_header;

	std::string fixed_frame_( m_mapParameters.frameId );

	// HACK: laser origin
	if (sensor_header.frame_id == "base_footprint") {
		stamped_in.header.frame_id = "laser_tilt_link";
	}

	geometry_msgs::Point p;
	p.x=p.y=p.z=0;
	try {
		m_tfListener.transformPoint(fixed_frame_, stamped_in, stamped_out);
	} catch(tf::TransformException& ex) {
		ros::Time t;
		std::string err_string;
		ROS_INFO_STREAM("Transforming sensor origin using latest common time because there's a tf problem");
		if (m_tfListener.getLatestCommonTime(fixed_frame_, stamped_in.header.frame_id, stamped_in.header.stamp, &err_string) == tf::NO_ERROR) {
			try {
				m_tfListener.transformPoint(fixed_frame_, stamped_in, stamped_out);
			} catch(...) {
				ROS_WARN_STREAM("Still can't transform sensor origin between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
			}
		} else {
			ROS_WARN_STREAM("No common time between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
		}
	}
	octomap::point3d retval (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

	return retval;
}

/**
 * Do octomap testing by object
 */
long int srs::COctoMapPlugin::doObjectTesting( srs::CTestingObjectBase * object )
{
	if( object == 0 )
	{
		PERROR( "Wrong testing object - NULL. ");
		return 0;
	}

	// Create removed nodes counter
	long int counter( 0 );

	// For all leaves
	for (srs::tButServerOcTree::leaf_iterator it = m_data->octree.begin_leafs(), end = m_data->octree.end_leafs(); it != end; ++it)
	{
		// Node is occupied?
		if (m_data->octree.isNodeOccupied(*it))
		{
			// Node is in testing object
			if( object->isIn( it.getX(), it.getY(), it.getZ() ) )
			{
				// "Remove" node
				m_data->octree.integrateMissNoTime(&*it);
				++counter;
			}

		}
	}

	return counter;
}
/**
 * Remove cube as a service - callback
 */
#define G2EPOINT( gp ) (Eigen::Vector3f( gp.x, gp.y, gp.z ))
#define G2EQUAT( gp ) (Eigen::Quaternionf( gp.x, gp.y, gp.z, gp.w ))
bool srs::COctoMapPlugin::removeCubeCB( srs_env_model::RemoveCube::Request & req, srs_env_model::RemoveCube::Response & res )
{

	PERROR( "Remove cube from octomap: " << req.pose << " --- " << req.size );

	// Debug - show cube position
	// addCubeGizmo( req.pose, req.size );

	if( m_removeTester != 0 )
		delete m_removeTester;

	// Test frame id
	if( req.frame_id != m_mapParameters.frameId )
	{
		// Transform pose
		geometry_msgs::PoseStamped ps, psout;
		ps.header.frame_id = req.frame_id;
		ps.header.stamp = m_mapParameters.currentTime;
		ps.pose = req.pose;

		m_tfListener.transformPose( m_mapParameters.frameId, ps, psout );
		req.pose = psout.pose;

		// Transform size
		geometry_msgs::PointStamped vs, vsout;
		vs.header.frame_id = req.frame_id;
		vs.header.stamp = m_mapParameters.currentTime;
		vs.point = req.size;

		m_tfListener.transformPoint( m_mapParameters.frameId, vs, vsout );
		req.size = vsout.point;

		PERROR( "Transformed cube from octomap: " << req.pose << " --- " << req.size );
	}

	// Create new tester
	m_removeTester = new srs::CTestingPolymesh( G2EPOINT( req.pose.position ), G2EQUAT( req.pose.orientation ), G2EPOINT( req.size ) );

	// Set it to life
	m_testerLifeCounter = m_testerLife;

	return true;
}

/**
 * For debugging purpouses - add cubical interactive marker to the scene
 */
void srs::COctoMapPlugin::addCubeGizmo( const geometry_msgs::Pose & pose, const geometry_msgs::Point & size )
{
	srs_interaction_primitives::AddUnknownObject gizmo;
	gizmo.request.pose = pose;
	gizmo.request.scale.x = size.x; gizmo.request.scale.y = size.y; gizmo.request.scale.z = size.z;
	gizmo.request.frame_id = m_mapParameters.frameId;
	gizmo.request.name = "OctomapGizmo";

	ros::service::call("but_interaction_primitives/add_unknown_object", gizmo );
}
