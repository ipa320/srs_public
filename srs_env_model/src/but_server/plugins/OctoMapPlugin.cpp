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

#define OCTOMAP_FRAME_ID std::string("/map")
#define OCTOMAP_PUBLISHER_NAME std::string("butsrv_binary_octomap")

void srs::COctoMapPlugin::setDefaults()
{
	// Set octomap parameters
	m_mapParameters.resolution = 0.05;
	m_mapParameters.treeDepth = 0;
	m_mapParameters.probHit = 0.7;         	// Probability of node, if node is occupied: 0.7
	m_mapParameters.probMiss = 0.4;        	// Probability of node, if node is free: 0.4
	m_mapParameters.thresMin = 0.45;		// Clamping minimum threshold: 0.1192;
	m_mapParameters.thresMax = 0.55; 		// Clamping maximum threshold: 0.971;
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

	// TODO: Remove this line!!!
	// m_mapParameters.maxRange = 2.0;

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

	// Create publisher
	m_ocPublisher = node_handle.advertise<octomap_ros::OctomapBinary>(m_ocPublisherName, 100, m_latchedTopics);

	PERROR( "OctoMapPlugin initialized..." );
}


void srs::COctoMapPlugin::insertCloud(const tPointCloud & cloud)
{
	if( !useFrame() )
		return;

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

	insertScan(cloudToMapTf.getOrigin(), pc_ground, pc_nonground);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Point cloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(),
			pc_nonground.size(), total_elapsed);

	// Publish new data
	invalidate();
}



void srs::COctoMapPlugin::insertScan(const tf::Point & sensorOriginTf, const tPointCloud & ground, const tPointCloud & nonground)
{
	// Write some debug info
	//    ROS_INFO("Inserting scan. Points: %u ", ground.size() + nonground.size() );


	octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

	// instead of direct scan insertion, compute update to filter ground:
	octomap::KeySet free_cells, occupied_cells;

	double maxRange(m_mapParameters.maxRange);
/*
	octomap::Pointcloud pcNonground;
	octomap::pointcloudPCLToOctomap( nonground, pcNonground );
	m_data->octree.insertScan( pcNonground, sensorOrigin, maxRange, true );

//	PERROR( "Scan inserted. Size: " << nonground.size() << ", " << pcNonground.size() << ", " << m_data->octree.getNumLeafNodes() );

/*/
//	std::cerr << "OCM:  Insert ground. MR: " << maxRange << ", SO: " << sensorOrigin << std::endl;

	// insert ground points only as free:
	for (tPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it)
	{
		octomap::point3d point(it->x, it->y, it->z);

		// maxrange check
		if ((maxRange > 0.0) && ((point - sensorOrigin).norm() > maxRange))
		{
			point = sensorOrigin + (point - sensorOrigin).normalized()
					* maxRange;
		}

		// only clear space (ground points)
		if (m_data->octree.computeRayKeys(sensorOrigin, point, m_keyRay))
		{
			free_cells.insert(m_keyRay.begin(), m_keyRay.end());
		}
	}


	// all other points: free on ray, occupied on endpoint:
	int miss(0), hit(0);

	for (tPointCloud::const_iterator it( nonground.begin() ), end( nonground.end() ); it != end; ++it)
	{

		octomap::point3d point(it->x, it->y, it->z);

		// maxrange check
		if ((maxRange < 0.0) || ((point - sensorOrigin).norm() <= maxRange))
		{
			//*
			// free cells
			if (m_data->octree.computeRayKeys(sensorOrigin, point, m_keyRay))
			{
				free_cells.insert(m_keyRay.begin(), m_keyRay.end());
			}
			///
			// occupied endpoint
			octomap::OcTreeKey key;
			if (m_data->octree.genKey(point, key))
			{
				occupied_cells.insert(key);
			}

		}
		else
		{// ray longer than maxrange:;

			octomap::point3d new_end = sensorOrigin	+ (point - sensorOrigin).normalized() * maxRange;

			if (m_data->octree.computeRayKeys(sensorOrigin, new_end,	m_keyRay))
			{
				free_cells.insert(m_keyRay.begin(), m_keyRay.end());
			}

		}
	}

//	PERROR( "Rays hit: " << hit << ", miss: " << miss );

	long fcounter(0), fchanged(0), ocounter(0), nfound(0);

	// mark free cells only if not seen occupied in this cloud
	for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
	{
		if (occupied_cells.find(*it) == occupied_cells.end())
		{
			double o1, o2;
			tButServerOcNode * node( m_data->octree.search( *it ) );
			if( node != 0 )
				o1 = node->getOccupancy();
			else
				++nfound;
			m_data->octree.updateNode(*it, false, false);

			if( node != 0 )
				o2 = node->getOccupancy();

			if( node != 0 && o1 != o2 )
			{
//				PERROR( "Node changed: " << o1 << " -> " << o2 );
				++fchanged;
			}
			++fcounter;
		}
	}

	// now mark all occupied cells:
	for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
	{
		m_data->octree.updateNode(*it, true, false);
		++ocounter;
	}

//	PERROR( "Free cells: " << fcounter << ", occupied:" << ocounter << ", MaxRange: " << maxRange << ", free changed: " << fchanged << ", not found: " << nfound );
//	PERROR( "OC stats. LN: " << m_data->octree.getNumLeafNodes() );

	// TODO: eval lazy+updateInner vs. proper insertion
	m_data->octree.updateInnerOccupancy();
//	m_data->octree.prune();
	//*/


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
	m_data->octree.clear();
}

///////////////////////////////////////////////////////////////////////////////
//  OCTOMAP CRAWLING
///////////////////////////////////////////////////////////////////////////////

/// Crawl octomap
void srs::COctoMapPlugin::crawl( const ros::Time & currentTime )
{
	// Fill needed structures
	onCrawlStart(currentTime);

	// Crawl through node
	for (srs::tButServerOcTree::leaf_iterator it = m_data->octree.begin_leafs(), end = m_data->octree.end_leafs(); it != end; ++it)
		{

			// call general hook:
			handleNode(it, m_mapParameters);

			// Node is occupied?
			if (m_data->octree.isNodeOccupied(*it))
			{
			    if( m_removeSpecles )
			    {
			        if( !isSpeckleNode(it) )
			        {
			            handleOccupiedNode(it, m_mapParameters);
			        }
			    }else
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
void srs::COctoMapPlugin::handleNode(const tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	m_sigOnNode( it, mp );
}

/// Handle free node
void srs::COctoMapPlugin::handleFreeNode(const tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	m_sigOnFreeNode( it, mp );
}

/// Handle occupied node
void srs::COctoMapPlugin::handleOccupiedNode(const tButServerOcTree::iterator & it, const SMapParameters & mp)
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

/**
 * Find if this node is specle
 * @param it - node iterator
 * @return true, if this node is specle
 */
bool srs::COctoMapPlugin::isSpeckleNode(const tButServerOcTree::iterator & it) const
{
    const octomap::OcTreeKey nKey( it.getKey() );
    octomap::OcTreeKey key;

    bool neighborFound = false;
    for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
        for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
            for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
                if (key != nKey){
                    tButServerOcNode* node = m_data->octree.search(key);
                    if (node && m_data->octree.isNodeOccupied(node)){
                        // we have a neighbor => break!
                        neighborFound = true;
                    }
                }
            }
        }
    }

    return neighborFound;
}
