#include <ros/ros.h>

#include <OgreVector3.h>

#include <visualization_msgs/Marker.h>
#include <srs_ui_but/ButCamMsg.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <stdio.h>
#include <utility>
#include <sstream>
#include <float.h>

#define MAX_FRUSTUM_DEPTH 15.0f
#define MAX_DISPLAY_DEPTH 15.0f
#define DISTANCE_CORRECTION 0.1f

using namespace std;
using namespace sensor_msgs;

// function prototypes
void countCameraParams(const CameraInfoConstPtr& camInfo);
pair<float, float> countPclDepths(const PointCloud2ConstPtr& pcl);
void publishViewFrustumMarker(const CameraInfoConstPtr cam_info,
		float frustum_depth);
void publishButDisplay(const CameraInfoConstPtr cam_info, float display_depth);

// Camera parameters
float elev_d, steer_r, elev_u, steer_l = 0;

// TF transformation listener for camera info message
tf::TransformListener *tf_cam_info_Listener;

// ROS messages publishers
ros::Publisher frustum_marker_pub, but_display_pub;

/*
 * callback for time-synchronised cameraInfo and PointCloud2 messages
 * cameraInfo also with available TF transformation
 */
void callback(const CameraInfoConstPtr cam_info, const PointCloud2ConstPtr& pcl) {
	ROS_DEBUG("Got everything together");

	countCameraParams(cam_info);
	pair<float, float> distances = countPclDepths(pcl);

	ROS_DEBUG_STREAM("nearest point " << distances.first << " most distant point " << distances.second);

	publishViewFrustumMarker(cam_info, distances.second);

	publishButDisplay(cam_info, distances.first);
}

void publishButDisplay(const CameraInfoConstPtr cam_info, float display_depth) {

	srs_ui_but::ButCamMsg rectangle;
	visualization_msgs::Marker polygon;

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
	tl_vec.x = +steer_l * display_depth;
	tl_vec.y = +elev_u * display_depth;
	tl_vec.z = display_depth;

	Ogre::Vector3 tr_vec;
	tr_vec.x = -steer_r * display_depth;
	tr_vec.y = +elev_u * display_depth;
	tr_vec.z = display_depth;

	Ogre::Vector3 bl_vec;
	bl_vec.x = +steer_l * display_depth;
	bl_vec.y = -elev_d * display_depth;
	bl_vec.z = display_depth;

	rectangle.scale.x = (tl_vec.distance(tr_vec));
	rectangle.scale.y = (tl_vec.distance(bl_vec));
	rectangle.scale.z = 1.0;

	// bottom right point transformed to /map frame for display position
	geometry_msgs::PointStamped br_map;

	br_map.header.frame_id = cam_info->header.frame_id;
	br_map.header.stamp = cam_info->header.stamp;
	br_map.point.x = -steer_r * display_depth;
	br_map.point.y = -elev_d * display_depth;
	br_map.point.z = display_depth;
	// transform point into /map frame
	tf_cam_info_Listener->transformPoint("/map", br_map, br_map);

	rectangle.pose.position.x = br_map.point.x;
	rectangle.pose.position.y = br_map.point.y;
	rectangle.pose.position.z = br_map.point.z;

	// publish display message
	but_display_pub.publish(rectangle);

}

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
 * Counts possible distance for rendering cameraDisplay according to closest
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
	BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_pointCloud.points) {
		dist = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
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

	return make_pair(near_distance-DISTANCE_CORRECTION, far_distance);
}

/*
 * Counts angle parameters of view frustum of one camera specified in given
 * CameraInfo message
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
			"view_frustum", 1);
	// but rectangle as srs_ui_but::ButCamMsg
	but_display_pub = nh.advertise<srs_ui_but::ButCamMsg> ("but_rectangle", 1);

	// subscribers to required topics
	message_filters::Subscriber<CameraInfo> cam_info_sub(nh,
			"/stereo/left/camera_info", 10);

	message_filters::Subscriber<PointCloud2> pcl_sub(nh, "/cam3d/depth/points",
			10);

	// initializing of listeners for tf transformation and tf message filter
	tf_cam_info_Listener = new tf::TransformListener();
	tf::MessageFilter<CameraInfo> *cam_info_transform_filter =
			new tf::MessageFilter<CameraInfo>(cam_info_sub,
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
	time_sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	return 1;
}
