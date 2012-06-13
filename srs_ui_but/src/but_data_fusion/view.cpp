/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vladimir Blahoz (xblaho02@stud.fit.vutbr.cz)
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

#include <ros/ros.h>

#include <OgreVector3.h>

#include <srs_ui_but/topics_list.h>
#include <srs_ui_but/services_list.h>
#include <srs_ui_but/ButCamMsg.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <stdio.h>
#include <utility>
#include <sstream>
#include <float.h>

// maximal depth of view frustum lines
#define MAX_FRUSTUM_DEPTH 15.0f

// maximal distance of camera display rendering
#define MAX_DISPLAY_DEPTH 15.0f

using namespace std;
using namespace sensor_msgs;

// function prototypes
void countCameraParams(const CameraInfoConstPtr& camInfo);
pair<float, float> countPclDepths(const PointCloud2ConstPtr& pcl);
void publishViewFrustumMarker(const CameraInfoConstPtr cam_info,
		float frustum_depth);
void publishButDisplay(const CameraInfoConstPtr cam_info, float display_depth);
void updateCameraTopic(ros::NodeHandle& nh);

// Camera parameters
float elev_d, steer_r, elev_u, steer_l = 0;

// TF transformation listener for camera info message
tf::TransformListener *tf_cam_info_Listener;

// ROS messages subscribers
message_filters::Subscriber<CameraInfo> *cam_info_sub;

// ROS messages publishers
ros::Publisher frustum_marker_pub, but_display_pub;

// parameters regularly updated from parameter server
//std::string camera_topic_par = "/stereo/left/camera_info";
std::string camera_topic_par = "/cam3d/rgb/camera_info";
double depth_par = 1.0f;

/*
 * @brief Callback for time-synchronised cameraInfo and PointCloud2 messages
 * cameraInfo also with available TF transformation
 *
 * @param nh Node handle
 * @param cam_info CameraInfo message
 * @param pcl PointCloud2 message
 */
void callback(ros::NodeHandle& nh, const CameraInfoConstPtr cam_info,
		const PointCloud2ConstPtr& pcl) {
	ROS_DEBUG("Got everything synced");

	// check parameter server for change of desired camera
	ros::param::getCached(srs_ui_but::Camera_PARAM, camera_topic_par);
	if (!camera_topic_par.empty())
		updateCameraTopic(nh);

	//check parameter server for change of desired polygon depth
	ros::param::getCached(srs_ui_but::Depth_PARAM, depth_par);

	// Count internal parameters of view volume
	countCameraParams(cam_info);

	// count maximal and minimal point cloud distance
	pair<float, float> distances = countPclDepths(pcl);

	ROS_DEBUG_STREAM("nearest point " << distances.first << " most distant point " << distances.second);

	publishViewFrustumMarker(cam_info, distances.second);

	publishButDisplay(cam_info, distances.first*depth_par);
}

/**
 * @brief Updates subscriber for camera parameters according to set topic camera_topic_par
 *
 * @param nh Node handle
 */
void updateCameraTopic(ros::NodeHandle& nh) {
	std::string cam3d("/cam3d/");
	std::string cam3dRGB("/cam3d/rgb/");
	std::string camLeft("/stereo/left/");
	std::string camRight("/stereo/right/");
	if (!camera_topic_par.compare(0, cam3d.size(), cam3d))
		cam_info_sub->subscribe(nh, cam3d.append("camera_info"), 10);
	else if (!camera_topic_par.compare(0, cam3dRGB.size(), cam3dRGB))
		cam_info_sub->subscribe(nh, cam3dRGB.append("camera_info"), 10);
	else if (!camera_topic_par.compare(0, camLeft.size(), camLeft))
		cam_info_sub->subscribe(nh, camLeft.append("camera_info"), 10);
	else if (!camera_topic_par.compare(0, camRight.size(), camRight))
		cam_info_sub->subscribe(nh, camRight.append("camera_info"), 10);
	else
		ROS_ERROR("UNKNOWN CAMERA");
}

/**
 * @brief Publishes message with camera display geometry
 *
 * @param cam_info Given camera info
 * @param display_depth Chosen depth of camera display
 */
void publishButDisplay(const CameraInfoConstPtr cam_info, float display_depth) {

	// recalculate real maximal polygon depth from closest point cloud point
	float real_depth = cos(elev_d)*display_depth;

	srs_ui_but::ButCamMsg rectangle;

	rectangle.header.frame_id = "/map";
	rectangle.header.stamp = cam_info->header.stamp;

	// retrieve transform for display rotation
	tf::StampedTransform cameraToWorldTf;
	try {
		tf_cam_info_Listener->waitForTransform("/map",
				cam_info->header.frame_id, cam_info->header.stamp,
				ros::Duration(0.2));
		tf_cam_info_Listener->lookupTransform("/map",
				cam_info->header.frame_id, cam_info->header.stamp,
				cameraToWorldTf);
	}
	// In case of absence of transformation path
	catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Camera info transform error: " << ex.what()
				<< ", quitting callback");
		return;
	}

	rectangle.pose.orientation.x = cameraToWorldTf.getRotation().x();
	rectangle.pose.orientation.y = cameraToWorldTf.getRotation().y();
	rectangle.pose.orientation.z = cameraToWorldTf.getRotation().z();
	rectangle.pose.orientation.w = cameraToWorldTf.getRotation().w();

	// Three corner vectors of display for display size
	geometry_msgs::PointStamped tl, tr, bl;

	Ogre::Vector3 tl_vec;
	tl_vec.x = +steer_l * real_depth;
	tl_vec.y = +elev_u * real_depth;
	tl_vec.z = real_depth;

	Ogre::Vector3 tr_vec;
	tr_vec.x = -steer_r * real_depth;
	tr_vec.y = +elev_u * real_depth;
	tr_vec.z = real_depth;

	Ogre::Vector3 bl_vec;
	bl_vec.x = +steer_l * real_depth;
	bl_vec.y = -elev_d * real_depth;
	bl_vec.z = real_depth;

	rectangle.scale.x = (tl_vec.distance(tr_vec));
	rectangle.scale.y = (tl_vec.distance(bl_vec));
	rectangle.scale.z = 1.0;

	// bottom right point transformed to /map frame for display position
	geometry_msgs::PointStamped br_map;

	br_map.header.frame_id = cam_info->header.frame_id;
	br_map.header.stamp = cam_info->header.stamp;
	br_map.point.x = -steer_r * real_depth;
	br_map.point.y = -elev_d * real_depth;
	br_map.point.z = real_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", br_map, br_map);

	rectangle.pose.position.x = br_map.point.x;
	rectangle.pose.position.y = br_map.point.y;
	rectangle.pose.position.z = br_map.point.z;

	// publish display message
	but_display_pub.publish(rectangle);

}

/**
 * @brief Publishes the marker representing view frustum
 *
 * @param cam_info given CameraInfo
 * @param frustum_deph chosen depth of view frustum lines
 */
void publishViewFrustumMarker(const CameraInfoConstPtr cam_info,
		float frustum_depth) {
	visualization_msgs::Marker marker;

	// global attributes for all points
	marker.id = 0;
	marker.header.stamp = cam_info->header.stamp;
	marker.header.frame_id = "/map";
	marker.ns = "view_frustum";
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.color.a = 1.0;
	marker.color.r = 1.0;

	// 5 vertices of view frustum and corresponding points
	// transformed to /map frame
	geometry_msgs::PointStamped tl, tr, bl, br, camera;
	geometry_msgs::PointStamped tl_map, tr_map, bl_map, br_map, camera_map;

	/*
	 * central point - camera position
	 */
	camera.header.frame_id = cam_info->header.frame_id;
	camera.header.stamp = cam_info->header.stamp;
	camera.point.x = 0.0;
	camera.point.y = 0.0;
	camera.point.z = 0.0;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", camera, camera_map);

	/*
	 * bottom right point
	 */
	br.header.frame_id = cam_info->header.frame_id;
	br.header.stamp = cam_info->header.stamp;
	br.point.x = -steer_r * frustum_depth;
	br.point.y = -elev_d * frustum_depth;
	br.point.z = frustum_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", br, br_map);

	/*
	 * bottom left point
	 */
	bl.header.frame_id = cam_info->header.frame_id;
	bl.header.stamp = cam_info->header.stamp;
	bl.point.x = +steer_l * frustum_depth;
	bl.point.y = -elev_d * frustum_depth;
	bl.point.z = frustum_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", bl, bl_map);

	/*
	 * top left point
	 */
	tl.header.frame_id = cam_info->header.frame_id;
	tl.header.stamp = cam_info->header.stamp;
	tl.point.x = +steer_l * frustum_depth;
	tl.point.y = +elev_u * frustum_depth;
	tl.point.z = frustum_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", tl, tl_map);

	/*
	 * top right point
	 */
	tr.header.frame_id = cam_info->header.frame_id;
	tr.header.stamp = cam_info->header.stamp;
	tr.point.x = -steer_r * frustum_depth;
	tr.point.y = +elev_u * frustum_depth;
	tr.point.z = frustum_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", tr, tr_map);

	// view frustum is made of 4 lines
	// pushing two point for each line
	marker.points.push_back(camera_map.point);
	marker.points.push_back(br_map.point);
	marker.points.push_back(camera_map.point);
	marker.points.push_back(bl_map.point);
	marker.points.push_back(camera_map.point);
	marker.points.push_back(tl_map.point);
	marker.points.push_back(camera_map.point);
	marker.points.push_back(tr_map.point);

	// publishing marker
	frustum_marker_pub.publish(marker);
	return;
}

/*
 * @brief Counts possible distance for rendering cameraDisplay according to closest
 * point in point cloud (so that display doesn't collide with pcl) and
 * depth of view frustum according to most distant point in point cloud
 */
pair<float, float> countPclDepths(const PointCloud2ConstPtr& pcl) {
	// PCL PointCloud
	pcl::PointCloud<pcl::PointXYZ> pcl_pointCloud;

	// Transfotm PointCloud2 to PCL PointCloud
	pcl::fromROSMsg(*pcl, pcl_pointCloud);

	// depth of view_frustum
	float far_distance = 0.0f;

	// distance of but display from camera
	float near_distance = FLT_MAX;

	// distance of current point from origin (3D camera position)
	float dist;
	// Get closest point and the most distant point
	BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_pointCloud.points)
	{	dist = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
	if (dist > far_distance) far_distance = dist;
	if (dist < near_distance) near_distance = dist;

	}

	far_distance = sqrt(far_distance);
	near_distance = sqrt(near_distance);

	// some points could be too far away from camera causing infinite frustum
	if (far_distance > MAX_FRUSTUM_DEPTH)
	far_distance = MAX_FRUSTUM_DEPTH;
	if (near_distance > MAX_DISPLAY_DEPTH)
	near_distance = MAX_DISPLAY_DEPTH;

	return make_pair(near_distance, far_distance);
}

/*
 * @brief Counts angle parameters of view frustum of one camera specified in given
 * CameraInfo message and sets internal variables elev_d, elev_u, steer_l and steer_r
 *
 * @param camInfo given CamerInfo message
 */
void countCameraParams(const CameraInfoConstPtr& camInfo) {

	// do other distortion models have the same camera matrices?
	if (camInfo->distortion_model != "plumb_bob") {
		ROS_ERROR("Unknown distortion model in CameraInfo message.\\"
				"Optimized only for plumb bob distortion model.");
	}

	unsigned int height = 0;
	unsigned int width = 0;

	// focal lengths and principal point coordinates
	float fx, fy, cx, cy;

	height = camInfo->height;
	width = camInfo->width;

	// getting essential parameters from intrinsic camera matrix
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	fx = camInfo->K[0];

	if (fx == 0) {
		ROS_ERROR("Uncalibrated camera, unable to count view frustum parameters.");
	}

	fy = camInfo->K[4];
	cx = camInfo->K[2];
	cy = camInfo->K[5];

	// counting view frustum parameters from camera parameters
	elev_d = cy / fy;
	steer_r = cx / fx;

	elev_u = (height - cy) / fy;
	steer_l = (width - cx) / fx;

	return;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "view");

	ros::NodeHandle nh;

	// initialize ROS messages publishers
	// frustum is published as simple marker
	frustum_marker_pub = nh.advertise<visualization_msgs::Marker> (
			srs_ui_but::ViewFrustum_TOPIC, 1);
	// but rectangle as srs_ui_but::ButCamMsg
	but_display_pub = nh.advertise<srs_ui_but::ButCamMsg> (srs_ui_but::CameraView_TOPIC,
			1);

	// subscribers to required topics
	cam_info_sub = new message_filters::Subscriber<CameraInfo>(nh,
			camera_topic_par, 10);

	message_filters::Subscriber<PointCloud2> pcl_sub(nh, "/cam3d/depth/points",
			10);

	// initializing of listeners for tf transformation and tf message filter
	tf_cam_info_Listener = new tf::TransformListener();
	tf::MessageFilter<CameraInfo> *cam_info_transform_filter =
			new tf::MessageFilter<CameraInfo>(*cam_info_sub,
					*tf_cam_info_Listener, "/map", 10);

	// synchronization policy - approximate time (exact time has too low hit rate)
	typedef message_filters::sync_policies::ApproximateTime<CameraInfo,
			PointCloud2> App_sync_policy;

	// time-synchronizing both messages with CameraInfo tf transformation
	message_filters::Synchronizer<App_sync_policy> time_sync(
			App_sync_policy(10), *cam_info_transform_filter, pcl_sub);

	//	message_filters::TimeSynchronizer<CameraInfo, PointCloud2> time_sync(
	//			*cam_info_transform_filter, pcl_sub, 10);

	// callback for TimeSynchronizer
	time_sync.registerCallback(boost::bind(&callback, nh, _1, _2));

	ros::spin();

	return 1;
}
