/******************************************************************************
 * \file
 *
 * $Id: detector.cpp 777 2012-05-11 11:23:17Z ihulik $
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

#include <srs_env_model_percp/but_plane_detector/plane_detector_node.h>
#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>

#include <srs_env_model_percp/but_seg_utils/filtering.h>
#include <srs_env_model_percp/but_seg_utils/normals.h>

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

#include <float.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/quaternion.hpp>

#include <stdlib.h>
#include <stdio.h>
using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;

namespace srs_env_model_percp
{
	/**
	 * Callback function manages sync of messages
	 */
	void callbackpcl(const PointCloud2ConstPtr& cloud)
	{
		++counter;
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
		tf::StampedTransform worldToSensorTf;
		try {
			tfListener->waitForTransform(target_topic, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
			tfListener->lookupTransform(target_topic, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);

			tfListener->waitForTransform(cloud->header.frame_id, target_topic, cloud->header.stamp, ros::Duration(0.2));
			tfListener->lookupTransform(cloud->header.frame_id, target_topic, cloud->header.stamp, worldToSensorTf);
		}
		catch(tf::TransformException& ex){
			std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
			return;
		}

		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
		// transform pointcloud from sensor frame to fixed robot frame

		pcl::transformPointCloud(pointcloud, pointcloud, sensorToWorld);

		// Compute normals and HT
		// fill aux Kinect cam_info (legacy - need to solve multiple callbacks sync with tf)


		//Normals normal(depth, cam_info_aux, NormalType::LSQAROUND);
		Normals normal(pointcloud);

		model->AddNext(normal);
//
//	//	if (counter % 5 == 0)
//	//	{
//	//		std::cerr << "Clearing noise...";
//	//		model->clearNoise(20.0);
//	//		std::cerr << "done..." << std::endl;
//	//	}
		model->recomputePlanes();

		model->scene_cloud->header.frame_id = target_topic;


		pcl::PointCloud<pcl::PointXYZRGB> outcloud;
		outcloud.header.frame_id = target_topic;
		int colorr = 0;
		int colorg = 0;
		int colorb = 0;
		for (unsigned int a = 0; a < model->planes.size(); ++a)
		{
			colorr = std::rand() % 255;
			colorg = std::rand() % 255;
			colorb = std::rand() % 255;

			for (unsigned int i = 0; i < normal.m_points.rows; ++i)
				for (unsigned int j = 0; j < normal.m_points.cols; ++j)
				{
					Vec3f point = normal.m_points.at<Vec3f>(i, j);
					cv::Vec4f localPlane = normal.m_planes.at<cv::Vec4f>(i, j);
					Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
					if (model->planes[a].distance(point) < 0.05 && model->planes[a].isSimilar(aaa, 0.01, 0.01))
					{
						pcl::PointXYZRGB current;
						current.x = point[0];
						current.y = point[1];
						current.z = point[2];
						current.r = colorr;
						current.g = colorg;
						current.b = colorb;
						outcloud.push_back(current);
					}
				}
		}


		pub1.publish(outcloud);//model->scene_cloud);
		//pub2.publish(model->current_hough_cloud);
		exporter->update(model->planes, normal, sensorToWorldTf);

		// Control out
		ros::Time end = ros::Time::now();
		std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
		std::cerr << "=========================================================" << endl<< endl;
	}

	void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info)
	{
		++counter;
		ros::Time begin = ros::Time::now();
		//  Debug info
		std::cerr << "Recieved frame..." << std::endl;
		std::cerr << "Cam info: fx:" << cam_info->K[0] << " fy:" << cam_info->K[4] << " cx:" << cam_info->K[2] <<" cy:" << cam_info->K[5] << std::endl;
		std::cerr << "Depth image h:" << dep->height << " w:" << dep->width << " e:" << dep->encoding << " " << dep->step << endl;
		std::cerr << "=========================================================" << endl;

		//get image from message
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(dep);
		cv::Mat depth = cv_image->image;


		Normals normal(depth, cam_info, NormalType::LSQAROUND);

		model->AddNext(depth, cam_info_aux, normal);
		model->recomputePlanes();

		model->scene_cloud->header.frame_id = "/head_cam3d_link";

		pub1.publish(model->scene_cloud);
		//	pub2.publish(model->current_hough_cloud);
		//exporter->update(model->planes, normal);

		// Control out
		ros::Time end = ros::Time::now();
		std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
		std::cerr << "=========================================================" << endl<< endl;
	}

	bool clear(srs_env_model_percp::ClearPlanes::Request &req,srs_env_model_percp::ClearPlanes::Response &res)
	{
		std::cout << "Clearing plane settings..." << std::endl;
		delete model;
		model = new SceneModel();

		res.message = "Hough space successfully reset.\n";
		std::cout << "Hough space successfully reset." << std::endl;
		return true;
	}
}


/**
 * Main detector module body
 */
int main( int argc, char** argv )
{
	using namespace srs_env_model_percp;

	ros::init(argc, argv, NODE_NAME);
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
	model = new SceneModel(3.0, -20.0, 20.0, 512, 4096, 11, 11, 0.02, 0.05);
	ros::ServiceServer service = n.advertiseService(SERVICE_CLEAR_PLANES, clear);

	if (input == "pcl")
	{
		exporter = new DynModelExporter(&n);

		// MESSAGES
		// message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
		message_filters::Subscriber<PointCloud2 > point_cloud(n, INPUT_POINT_CLOUD_TOPIC, 1);

		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > (OUTPUT_POINT_CLOUD_TOPIC, 1);

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
		exporter = new DynModelExporter(&n);
		// MESSAGES
		// message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
		message_filters::Subscriber<Image> depth_sub(n, INPUT_IMAGE_TOPIC, 1);
		message_filters::Subscriber<CameraInfo> info_sub_depth(n, INPUT_CAM_INFO_TOPIC, 1);


		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > (OUTPUT_POINT_CLOUD_TOPIC, 1);
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
