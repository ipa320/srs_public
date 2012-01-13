/**
 * $Id: detector.cpp 151 2012-01-13 12:25:29Z ihulik $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 * The testing framework for future plane detection using Hough Transform
 *
 * Subscribes cam_info and point cloud messages from COB
 * Tries to detect planes in image and sends them via interactive markers server to display
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

// OpenCV 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <float.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/quaternion.hpp>
#include "plane_det/filtering.h"
#include "plane_det/normals.h"
#include "plane_det/sceneModel.h"
#include "plane_det/dynModelExporter.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;

sensor_msgs::PointCloud2 cloud_msg;
ros::Publisher pub1;
ros::Publisher pub2;
but_scenemodel::DynModelExporter *exporter = NULL;

/**
 * Callback function manages sync of messages
 */
void callback(const PointCloud2ConstPtr& cloud, const CameraInfoConstPtr& cam_info)
{
	ros::Time begin = ros::Time::now();

	// generate depth map (legacy, due to old dataset)
	pcl::PointCloud<pcl::PointXYZ> pointcloud;
	pcl::fromROSMsg (*cloud, pointcloud);

	// Control out
	std::cerr << "Recieved frame..." << std::endl;
	std::cerr << "Topic: " << pointcloud.header.frame_id << std::endl;
	std::cerr << "Width: " << pointcloud.width << " height: " << pointcloud.height << std::endl;
	std::cerr << "=========================================================" << endl;

	Mat depth(cvSize(pointcloud.width, pointcloud.height), CV_16UC1);
	for(int y = 0; y < (int)pointcloud.height; y++)
	for(int x = 0; x < (int)pointcloud.width; x++) {
		depth.at<unsigned short>(y, x) = pointcloud.at(x, y).z * 1000.0;
	}

	// Compute normals and HT
	but_scenemodel::Normals normal(depth, cam_info, but_scenemodel::NormalType::LSQAROUND);
	but_scenemodel::SceneModel model(depth, cam_info, normal);

	// send scene cloud and HT cloud if necessary
	//	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	//	voxelgrid.setInputCloud(cloud);
	//	voxelgrid.setLeafSize(0.05, 0.05, 0.05);
	//	voxelgrid.filter(*cloud);

	model.scene_cloud->header.frame_id = "/openni_depth_frame";
	model.current_hough_cloud->header.frame_id = "/openni_depth_frame";
	pub1.publish(model.scene_cloud);
	//	pub2.publish(model.current_hough_cloud);
	exporter->update(model.planes, model.scene_cloud);

	// Control out
	ros::Time end = ros::Time::now();
	std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
	std::cerr << "=========================================================" << endl<< endl;
}


/**
 * Main detector module body
 */
int main( int argc, char** argv )
{
	ros::init(argc, argv, "plane_detector");
	ros::NodeHandle n;

	exporter = new but_scenemodel::DynModelExporter(&n);

	// MESSAGES
	// message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
	message_filters::Subscriber<CameraInfo> info_sub_depth(n, "/cam3d/camera_info", 1);
	message_filters::Subscriber<PointCloud2 > point_cloud(n, "/cam3d/depth/points", 1);
	pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/point_cloud", 1);
	pub2 = n.advertise<visualization_msgs::Marker> ("/polygons", 1);
	//pub2 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/hough_cloud", 1);


	// sync images
	typedef sync_policies::ApproximateTime<PointCloud2, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_cloud, info_sub_depth);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	std::cerr << "Plane detector initialized and listening..." << std::endl;
	ros::spin();

	return 1;
}
