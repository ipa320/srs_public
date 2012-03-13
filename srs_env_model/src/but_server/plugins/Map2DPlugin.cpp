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

#include <but_server/plugins/Map2DPlugin.h>
#include <pcl_ros/transforms.h>


#define MAP2D_PUBLISHER_NAME std::string("butsrv_map2d_object")
#define MAP2D_FRAME_ID std::string("/map")


srs::CMap2DPlugin::CMap2DPlugin(const std::string & name)
: srs::CServerPluginBase(name)
, m_publishMap2D(true)
, m_map2DPublisherName(MAP2D_PUBLISHER_NAME)
, m_latchedTopics(false)
, m_map2DFrameId(MAP2D_FRAME_ID)
, m_minSizeX(0.0)
, m_minSizeY(0.0)
{
	m_data = new tData;
	assert( m_data != 0 );
}



srs::CMap2DPlugin::~CMap2DPlugin()
{
}



bool srs::CMap2DPlugin::shouldPublish()
{
	return( m_publishMap2D && m_map2DPublisher.getNumSubscribers() > 0 );
}



void srs::CMap2DPlugin::init(ros::NodeHandle & node_handle)
{
	node_handle.param("collision_object_publisher", m_map2DPublisherName, MAP2D_PUBLISHER_NAME );
	node_handle.param("collision_object_frame_id", m_map2DFrameId, MAP2D_FRAME_ID );
	node_handle.param("min_x_size", m_minSizeX, m_minSizeX);
	node_handle.param("min_y_size", m_minSizeY, m_minSizeY);

	// Create publisher
	m_map2DPublisher = node_handle.advertise<nav_msgs::OccupancyGrid> (m_map2DPublisherName, 100, m_latchedTopics);
}



void srs::CMap2DPlugin::onPublish(const ros::Time & timestamp)
{
	m_map2DPublisher.publish(*m_data);
}



void srs::CMap2DPlugin::onFrameStart(const SMapParameters & par)
{
	m_data->header.frame_id = m_map2DFrameId;
	m_data->header.stamp = par.currentTime;
	m_data->info.resolution = par.resolution;

	m_ocFrameId = par.frameId;
	ros::Time timestamp( par.currentTime );

	tf::StampedTransform ocToMap2DTf;

	// Get transform
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_map2DFrameId, m_ocFrameId,
				timestamp, ros::Duration(2.0)); // orig. 0.2

		m_tfListener.lookupTransform(m_map2DFrameId, m_ocFrameId,
				timestamp, ocToMap2DTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		return;
	}


	Eigen::Matrix4f ocToMap2DTM;

	// Get transformation matrix
	pcl_ros::transformAsMatrix(ocToMap2DTf, ocToMap2DTM);

	const tButServerOcMap &map(*par.map);

	// Disassemble translation and rotation
	m_ocToMap2DRot  = ocToMap2DTM.block<3, 3> (0, 0);
	m_ocToMap2DTrans = ocToMap2DTM.block<3, 1> (0, 3);

	double minX, minY, minZ, maxX, maxY, maxZ;
	map.octree.getMetricMin(minX, minY, minZ);
	map.octree.getMetricMax(maxX, maxY, maxZ);

	octomap::point3d minPt(minX, minY, minZ);
	octomap::point3d maxPt(maxX, maxY, maxZ);
	octomap::OcTreeKey minKey, maxKey, curKey;

	// Try to create key
	if (!map.octree.genKey(minPt, minKey)) {
		ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
		return;
	}

	if (!map.octree.genKey(maxPt, maxKey)) {
		ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
		return;
	}

	ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

	// add padding if requested (= new min/maxPts in x&y):
	double halfPaddedX = 0.5 * m_minSizeX;
	double halfPaddedY = 0.5 * m_minSizeY;
	minX = std::min(minX, -halfPaddedX);
	maxX = std::max(maxX, halfPaddedX);
	minY = std::min(minY, -halfPaddedY);
	maxY = std::max(maxY, halfPaddedY);
	minPt = octomap::point3d(minX, minY, minZ);
	maxPt = octomap::point3d(maxX, maxY, maxZ);

	octomap::OcTreeKey paddedMaxKey;

	if (!map.octree.genKey(minPt, m_paddedMinKey)) {
		ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
		return;
	}

	if (!map.octree.genKey(maxPt, paddedMaxKey)) {
		ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
		return;
	}

	ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1],
			m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
	assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

	m_data->info.width = paddedMaxKey[0] - m_paddedMinKey[0] + 1;
	m_data->info.height = paddedMaxKey[1] - m_paddedMinKey[1] + 1;
	int mapOriginX = minKey[0] - m_paddedMinKey[0];
	int mapOriginY = minKey[1] - m_paddedMinKey[1];
	assert(mapOriginX >= 0 && mapOriginY >= 0);

	// might not exactly be min / max of octree:
	octomap::point3d origin;
	map.octree.genCoords(m_paddedMinKey, par.treeDepth, origin);
	m_data->info.origin.position.x = origin.x() - par.resolution* 0.5;
	m_data->info.origin.position.y = origin.y() - par.resolution * 0.5;

	// Allocate space to hold the data (init to unknown)
	m_data->data.resize(m_data->info.width * m_data->info.height, -1);
}



void srs::CMap2DPlugin::handleOccupiedNode(const srs::tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	if (it.getDepth() == mp.treeDepth)
	{

		octomap::OcTreeKey nKey = it.getKey(); // TODO: remove intermedate obj (1.4)
		int i = nKey[0] - m_paddedMinKey[0];
		int j = nKey[1] - m_paddedMinKey[1];
		m_data->data[m_data->info.width * j + i] = 100;

	} else {

		int intSize = 1 << (mp.treeDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = minKey[0] + dx - m_paddedMinKey[0];
			for (int dy = 0; dy < intSize; dy++) {
				int j = minKey[1] + dy - m_paddedMinKey[1];
				m_data->data[m_data->info.width * j + i] = 100;
			}
		}
	}
}


void srs::CMap2DPlugin::handleFreeNode(const srs::tButServerOcTree::iterator & it, const SMapParameters & mp )
{
	if (it.getDepth() == mp.treeDepth) {
		octomap::OcTreeKey nKey = it.getKey(); //TODO: remove intermedate obj (1.4)
		int i = nKey[0] - m_paddedMinKey[0];
		int j = nKey[1] - m_paddedMinKey[1];
		if (m_data->data[m_data->info.width * j + i] == -1) {
			m_data->data[m_data->info.width * j + i] = 0;
		}
	} else {
		int intSize = 1 << (mp.treeDepth - it.getDepth());
		octomap::OcTreeKey minKey = it.getIndexKey();
		for (int dx = 0; dx < intSize; dx++) {
			int i = minKey[0] + dx - m_paddedMinKey[0];
			for (int dy = 0; dy < intSize; dy++) {
				int j = minKey[1] + dy - m_paddedMinKey[1];
				if (m_data->data[m_data->info.width * j + i] == -1) {
					m_data->data[m_data->info.width * j + i] = 0;
				}
			}
		}
	}
}


void srs::CMap2DPlugin::handlePostNodeTraversal(const ros::Time & rostime)
{
	invalidate();
}
