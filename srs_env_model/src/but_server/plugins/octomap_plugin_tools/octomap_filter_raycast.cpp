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

#include <srs_env_model/but_server/plugins/octomap_plugin_tools/octomap_filter_raycast.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>

#include <srs_env_model/topics_list.h>

/**
 * Constructor
 */
srs_env_model::COcFilterRaycast::COcFilterRaycast(const std::string & octree_frame_id, ERunMode mode /*= FILTER_ALLWAYS*/)
	: COcTreeFilterBase( octree_frame_id, mode )
	, m_bFilterInitialized(false)
	, m_camera_stereo_offset_left(128)
	, m_camera_stereo_offset_right(0)
	, m_bCamModelInitialized(false)
	, m_camera_info_topic(CAMERA_INFO_TOPIC_NAME)
	, m_cloudPtr(0)
	, m_numLeafsRemoved(0)
{

}

/**
 * Initialize. Must be called before first filtering
 */
void srs_env_model::COcFilterRaycast::init(ros::NodeHandle & node_handle)
{

	// stereo cam params for sensor cone:
	node_handle.param<int> ("camera_stereo_offset_left",
			m_camera_stereo_offset_left, 128);
	node_handle.param<int> ("camera_stereo_offset_right",
			m_camera_stereo_offset_right, 0);

	// Used camera info topic
	node_handle.param("camera_info_topic", m_camera_info_topic,
					m_camera_info_topic);

	// Add camera info subscriber
	m_ciSubscriber = node_handle.subscribe(m_camera_info_topic, 1,
			&COcFilterRaycast::cameraInfoCB, this);
}

/**
 * Set input cloud. Must be called before filter call
 */
void srs_env_model::COcFilterRaycast::setCloud(const tPointCloud * cloud)
{
	assert( cloud != 0 );
	m_cloudPtr = cloud;
}

//! Write some info about last filter run
void srs_env_model::COcFilterRaycast::writeLastRunInfo()
{
	std::cerr << "COcFilterRaycast: Number of leafs removed: " << m_numLeafsRemoved << std::endl;
}

/**
 * Camera ifo callback - initialize camera model
 */
void srs_env_model::COcFilterRaycast::cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info) {

	boost::mutex::scoped_lock lock( m_lockCamera );

	// Get camera model from camera info
	m_camera_model.fromCameraInfo(*cam_info);
	m_camera_size = m_camera_model.fullResolution();

	// Set flag
	m_bCamModelInitialized = true;
}

/**
 * Filtering function implementation
 */
void srs_env_model::COcFilterRaycast::filterInternal( tButServerOcTree & tree )
{
	assert( m_cloudPtr != 0 );

	m_numLeafsRemoved = 0;

	// Get sensor origin
	octomap::point3d sensor_origin = getSensorOrigin(m_cloudPtr->header);
	octomap::pose6d sensor_pose(sensor_origin.x(), sensor_origin.y(), sensor_origin.z(), 0, 0, 0);

	// Is camera model initialized?
	if (!m_bCamModelInitialized)
	{
		ROS_ERROR("ERROR: camera model not initialized.");
		return;
	}

	tf::StampedTransform trans;
	try
	{
		m_tfListener.lookupTransform(m_cloudPtr->header.frame_id, m_treeFrameId, m_cloudPtr->header.stamp, trans);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR_STREAM("ERROR: Cannot find transform: " <<  m_cloudPtr->header.frame_id << ", " << m_treeFrameId);
		return;
	}

	tf::Transform to_sensor = trans;

	// compute bbx from sensor cone
	octomap::point3d min;
	octomap::point3d max;
	computeBBX(m_cloudPtr->header, min, max);

	double resolution(tree.getResolution());

	boost::mutex::scoped_lock lock(m_lockCamera);

	unsigned query_time = time(NULL);
	unsigned max_update_time = 1;
	for (tButServerOcTree::leaf_bbx_iterator it =
			tree.begin_leafs_bbx(min, max), end = tree.end_leafs_bbx(); it
			!= end; ++it) {
		if (tree.isNodeOccupied(*it) && ((query_time - it->getTimestamp())
				> max_update_time)) {
			tf::Point pos(it.getX(), it.getY(), it.getZ());
			tf::Point posRel = to_sensor(pos);
			cv::Point2d uv = m_camera_model.project3dToPixel(cv::Point3d(
					posRel.x(), posRel.y(), posRel.z()));

			// ignore point if not in sensor cone
			if (!inSensorCone(uv))
				continue;

			// ignore point if it is occluded in the map
			if (isOccludedMap(sensor_origin, it.getCoordinate(), resolution, tree))
				continue;

			// otherwise: degrade node
			tree.integrateMissNoTime(&*it);
			++m_numLeafsRemoved;
		}
	}
}

/**
 * Compute sensor origin from the header
 */
octomap::point3d srs_env_model::COcFilterRaycast::getSensorOrigin(const std_msgs::Header& sensor_header)
{
	geometry_msgs::PointStamped stamped_in;
	geometry_msgs::PointStamped stamped_out;
	stamped_in.header = sensor_header;

	std::string fixed_frame_(m_treeFrameId);

	// HACK: laser origin
	if (sensor_header.frame_id == "base_footprint") {
		stamped_in.header.frame_id = "laser_tilt_link";
	}

	geometry_msgs::Point p;
	p.x = p.y = p.z = 0;
	try
	{
		m_tfListener.transformPoint(fixed_frame_, stamped_in, stamped_out);
	}
	catch (tf::TransformException& ex) {
		ros::Time t;
		std::string err_string;
		ROS_INFO_STREAM("Transforming sensor origin using latest common time because there's a tf problem");
		if (m_tfListener.getLatestCommonTime(fixed_frame_,
				stamped_in.header.frame_id, stamped_in.header.stamp,
				&err_string) == tf::NO_ERROR) {
			try {
				m_tfListener.transformPoint(fixed_frame_, stamped_in,
						stamped_out);
			} catch (...) {
				ROS_WARN_STREAM("Still can't transform sensor origin between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
			}
		} else {
			ROS_WARN_STREAM("No common time between " << fixed_frame_ << " and " << stamped_in.header.frame_id);
		}
	}

	octomap::point3d retval(stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

	return retval;
}
/**
 * Is point in sensor cone?
 */
bool srs_env_model::COcFilterRaycast::inSensorCone(const cv::Point2d& uv) const {
	// Check if projected 2D coordinate in pixel range.
	// This check is a little more restrictive than it should be by using
	// 1 pixel less to account for rounding / discretization errors.
	// Otherwise points on the corner are accounted to be in the sensor cone.
	return ((uv.x > m_camera_stereo_offset_left + 1) && (uv.x
			< m_camera_size.width + m_camera_stereo_offset_right - 2) && (uv.y
			> 1) && (uv.y < m_camera_size.height - 2));
}

/**
 * Return true, if occupied cell is between origin and p
 */
bool srs_env_model::COcFilterRaycast::isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p, double resolution, tButServerOcTree & tree) const
{
	octomap::point3d direction(p - sensor_origin);
	octomap::point3d obstacle;
	double range = direction.norm() - resolution;

	if (tree.castRay(sensor_origin, direction, obstacle, true, range))
	{
		// fprintf(stderr, "<%.2f , %.2f , %.2f> -> <%.2f , %.2f , %.2f> // obs at: <%.2f , %.2f , %.2f>, range: %.2f\n",
		//         sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
		//         p.x(), p.y(), p.z(),
		//         obstacle.x(), obstacle.y(), obstacle.z(), (obstacle-p).norm());
		return true;
	}
	return false;
}


/**
 * Compute boundig box
 */
void srs_env_model::COcFilterRaycast::computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max)
{
	std::string sensor_frame = sensor_header.frame_id;

		//  transform sensor FOV
		geometry_msgs::PointStamped stamped_in;
		geometry_msgs::PointStamped stamped_out;
		stamped_in.header = sensor_header;
		stamped_in.header.frame_id = sensor_frame;

		// get max 3d points from camera at 0.5m and 5m.
		geometry_msgs::Point p[8];

		// define min/max 2d points
		cv::Point2d uv[4];
		uv[0].x = m_camera_stereo_offset_left;
		uv[0].y = 0;
		uv[1].x = m_camera_size.width + m_camera_stereo_offset_right;
		uv[1].y = 0;
		uv[2].x = m_camera_size.width + m_camera_stereo_offset_right;
		uv[2].y = m_camera_size.height;
		uv[3].x = m_camera_stereo_offset_left;
		uv[3].y = m_camera_size.height;

		// transform to 3d space
		cv::Point3d xyz[4];
		for (int i = 0; i < 4; i++) {
			xyz[i] = m_camera_model.projectPixelTo3dRay(uv[i]);
			cv::Point3d xyz_05 = xyz[i] * 0.5;
			xyz[i] *= 5.; // 5meters
			p[i].x = xyz[i].x;
			p[i].y = xyz[i].y;
			p[i].z = xyz[i].z;
			p[i + 4].x = xyz_05.x;
			p[i + 4].y = xyz_05.y;
			p[i + 4].z = xyz_05.z;
		}

		// transform to world coodinates and find axis-aligned bbx
		bbx_min.x() = bbx_min.y() = bbx_min.z() = 1e6;
		bbx_max.x() = bbx_max.y() = bbx_max.z() = -1e6;
		for (int i = 0; i < 8; i++) {
			stamped_in.point = p[i];
			m_tfListener.transformPoint(m_treeFrameId, stamped_in,
					stamped_out);
			p[i].x = stamped_out.point.x;
			p[i].y = stamped_out.point.y;
			p[i].z = stamped_out.point.z;
			if (p[i].x < bbx_min.x())
				bbx_min.x() = p[i].x;
			if (p[i].y < bbx_min.y())
				bbx_min.y() = p[i].y;
			if (p[i].z < bbx_min.z())
				bbx_min.z() = p[i].z;
			if (p[i].x > bbx_max.x())
				bbx_max.x() = p[i].x;
			if (p[i].y > bbx_max.y())
				bbx_max.y() = p[i].y;
			if (p[i].z > bbx_max.z())
				bbx_max.z() = p[i].z;
		}
}
