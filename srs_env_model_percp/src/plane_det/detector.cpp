/******************************************************************************
 * \file
 *
 * $Id: detector.cpp 620 2012-04-16 13:49:27Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
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

/**
 * Description:
 * The testing framework for future plane detection using Hough Transform
 *
 * Subscribes cam_info and point cloud messages from COB
 * Tries to detect planes in image and sends them via interactive markers server to display
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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
#include <pcl_ros/transforms.h>

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
#include <srs_env_model_percp/ClearPlanes.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;

string target_topic = "/map";

sensor_msgs::PointCloud2 cloud_msg;
tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;
tf::TransformListener *tfListener;
ros::Publisher pub1;
ros::Publisher pub2;
but_scenemodel::DynModelExporter *exporter = NULL;

CameraInfo cam_info_legacy;
CameraInfoConstPtr cam_info_aux (&cam_info_legacy);
but_scenemodel::SceneModel *model;

/**
 * Callback function manages sync of messages
 */
void callbackpcl(const PointCloud2ConstPtr& cloud)
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

	// transform to world
	tf::StampedTransform sensorToWorldTf;
    try {
    	tfListener->waitForTransform(target_topic, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
    	tfListener->lookupTransform(target_topic, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    }
    catch(tf::TransformException& ex){
        std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    // transform pointcloud from sensor frame to fixed robot frame



	Mat depth(cvSize(pointcloud.width, pointcloud.height), CV_16UC1);
	for(int y = 0; y < (int)pointcloud.height; y++)
	for(int x = 0; x < (int)pointcloud.width; x++) {
		depth.at<unsigned short>(y, x) = pointcloud.at(x, y).z * 1000.0;
	}
	//pcl::transformPointCloud(pointcloud, pointcloud, sensorToWorld);

	// Compute normals and HT
	// fill aux Kinect cam_info (legacy - need to solve multiple callbacks sync with tf)


	but_scenemodel::Normals normal(depth, cam_info_aux, but_scenemodel::NormalType::LSQAROUND);
	//but_scenemodel::Normals normal(depth, pointcloud, cam_info_aux, but_scenemodel::NormalType::LSQAROUND);

	model->AddNext(depth, cam_info_aux, normal);
	model->recomputePlanes();
	// send scene cloud and HT cloud if necessary
	//	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	//	voxelgrid.setInputCloud(cloud);
	//	voxelgrid.setLeafSize(0.05, 0.05, 0.05);
	//	voxelgrid.filter(*cloud);

	model->scene_cloud->header.frame_id = target_topic;
	pub1.publish(model->scene_cloud);
	//pub2.publish(model->current_hough_cloud);
	exporter->update(model->planes, model->scene_cloud, sensorToWorldTf);

	// Control out
	ros::Time end = ros::Time::now();
	std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
	std::cerr << "=========================================================" << endl<< endl;
}

void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info)
{
	ros::Time begin = ros::Time::now();
	//  Debug info
	std::cerr << "Recieved frame..." << std::endl;
	std::cerr << "Cam info: fx:" << cam_info->K[0] << " fy:" << cam_info->K[4] << " cx:" << cam_info->K[2] <<" cy:" << cam_info->K[5] << std::endl;
	std::cerr << "Depth image h:" << dep->height << " w:" << dep->width << " e:" << dep->encoding << " " << dep->step << endl;
	std::cerr << "=========================================================" << endl;

	//get image from message
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(dep);
	cv::Mat depth = cv_image->image;


	but_scenemodel::Normals normal(depth, cam_info, but_scenemodel::NormalType::LSQAROUND);

	model->AddNext(depth, cam_info_aux, normal);
	model->recomputePlanes();

	model->scene_cloud->header.frame_id = "/head_cam3d_link";

	pub1.publish(model->scene_cloud);
	//	pub2.publish(model->current_hough_cloud);
	exporter->update(model->planes, model->scene_cloud);

	// Control out
	ros::Time end = ros::Time::now();
	std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
	std::cerr << "=========================================================" << endl<< endl;
}

bool clear(srs_env_model_percp::ClearPlanes::Request &req,srs_env_model_percp::ClearPlanes::Response &res)
{
	std::cout << "Clearing plane settings..." << std::endl;
	delete model;
	model = new but_scenemodel::SceneModel();

	res.message = "Hough space successfully reset.\n";
	std::cout << "Hough space successfully reset." << std::endl;
	return true;
}
/**
 * Main detector module body
 */
int main( int argc, char** argv )
{
	ros::init(argc, argv, "plane_detector");
	ros::NodeHandle n;

	string input = "";
	if (argc == 3 || argc == 5)
	{
		if (strcmp(argv[1], "-input")==0)
		{
			if (strcmp(argv[2], "pcl")==0)
				input = "pcl";
			else if (strcmp(argv[2], "kinect")==0)
				input = "kinect";
		}
		if (argc==5)
		{
			if (strcmp(argv[3], "-target")==0)
			target_topic = argv[4];
		}
	}
	model = new but_scenemodel::SceneModel();
	ros::ServiceServer service = n.advertiseService("/detector/clear_planes", clear);

	if (input == "pcl")
	{
		exporter = new but_scenemodel::DynModelExporter(&n);

		// MESSAGES
		// message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
		message_filters::Subscriber<PointCloud2 > point_cloud(n, "/cam3d/depth/points", 1);

		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/point_cloud", 1);

		cam_info_legacy.K[0] = 589.367;
		cam_info_legacy.K[2] = 320.5;
		cam_info_legacy.K[4] = 589.367;
		cam_info_legacy.K[5] = 240.5;

		// sync images
		tfListener = new tf::TransformListener();
		transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2> (point_cloud, *tfListener, target_topic, 1);
		transform_filter->registerCallback(boost::bind(&callbackpcl, _1));
		std::cerr << "Plane detector initialized and listening point clouds..." << std::endl;
		ros::spin();

		return 1;
	}
	else if (input == "kinect")
	{
		exporter = new but_scenemodel::DynModelExporter(&n);
		// MESSAGES
		// message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
		message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
		message_filters::Subscriber<CameraInfo> info_sub_depth(n, "/cam3d/depth/camera_info", 1);


		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/point_cloud", 1);
		//pub2 = n.advertise<visualization_msgs::Marker> ("/polygons", 1);

		typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, info_sub_depth);
		sync.registerCallback(boost::bind(&callbackkinect, _1, _2));
		std::cerr << "Plane detector initialized and listening depth images..." << std::endl;
		ros::spin();

		return 1;
	}
	else
	{
		std::cerr << "Please specify input type (-input pcl or -input kinect)" << std::endl;
		return 0;
	}
}
