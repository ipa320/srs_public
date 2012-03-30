/******************************************************************************
 * \file
 *
 * $Id:$
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
#include "plane_det/dynModelExporter.h"
#include "plane_det/normals.h"

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;

#define NUM_PLANES_RANSAC 100
#define NUM_PLANE_MIN 50
sensor_msgs::PointCloud2 cloud_msg;
tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;
tf::TransformListener *tfListener;
ros::Publisher pub1;
ros::Publisher pub2;
but_scenemodel::DynModelExporter *exporter = NULL;

CameraInfo cam_info_legacy;
CameraInfoConstPtr cam_info_aux (&cam_info_legacy);

/**
 * Callback function manages sync of messages
 */
void callback(const PointCloud2ConstPtr& cloud)
{
	ros::Time begin = ros::Time::now();

	// generate depth map (legacy, due to old dataset)
	pcl::PointCloud<pcl::PointXYZ> pointcloud;
	pcl::fromROSMsg (*cloud, pointcloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg (*cloud, *cloud2);

	// Control out
	std::cerr << "Recieved frame..." << std::endl;
	std::cerr << "Topic: " << pointcloud.header.frame_id << std::endl;
	std::cerr << "Width: " << pointcloud.width << " height: " << pointcloud.height << std::endl;
	std::cerr << "=========================================================" << endl;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Get transformation

	// transform to world
	tf::StampedTransform sensorToWorldTf;
    try {
    	tfListener->waitForTransform("/map", cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
    	tfListener->lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    }
    catch(tf::TransformException& ex){
        std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    pcl::transformPointCloud(*cloud2, *cloud2, sensorToWorld);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Do RANSAC

    // found planes
	std::vector<but_scenemodel::Plane<float> > planes;

	// search for NUM_PLANES_RANSAC planes
	for (int i = 0; i < NUM_PLANES_RANSAC; ++i)
	{
		// if we have used the whole cloud
		if (cloud2->size() <= 0) break;

		// Inliers indices
		std::vector<int> inliers;

		// created RandomSampleConsensus object and compute the appropriated model
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud2));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		// If we found a plane with minimal size
		if (inliers.size() > NUM_PLANE_MIN)
		{
			// Create index object - for filtering
			pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
			fInliers->indices = inliers;

			// save points for least squares into vector and generate output cloud
			std::vector<cv::Vec3f> points;
			for (unsigned int i = 0; i < inliers.size(); ++i)
			{
				pcl::PointXYZ pt = cloud2->at(inliers[i]);
				pcl::PointXYZRGB pt2;
				pt2.x = pt.x;
				pt2.y = pt.y;
				pt2.z = pt.z;
				points.push_back(cv::Vec3f(pt.x, pt.y, pt.z));
				output->push_back(pt2);
			}

			// Get plane
			planes.push_back(but_scenemodel::Normals::LeastSquaresPlane(points));

			// Substract inliers point from original cloud and repeat
			pcl::ExtractIndices<pcl::PointXYZ> extract ;
			extract.setInputCloud (cloud2);
			extract.setIndices (fInliers);
			extract.setNegative (true);
			extract.filter (*cloud2);
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	output->header.frame_id = "/map";

	// Call interactive markers service
	exporter->update(planes, output);

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
	message_filters::Subscriber<tf::tfMessage>transform(n, "/tf", 1);

	pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/point_cloud", 1);
	pub2 = n.advertise<visualization_msgs::Marker> ("/polygons", 1);
	//pub2 = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/hough_cloud", 1);


	cam_info_legacy.K[0] = 589.367;
	cam_info_legacy.K[2] = 320.5;
	cam_info_legacy.K[4] = 589.367;
	cam_info_legacy.K[5] = 240.5;

	// sync images
	tfListener = new tf::TransformListener();
	transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2> (point_cloud, *tfListener, "/map", 1);
	transform_filter->registerCallback(boost::bind(&callback, _1));
	//typedef sync_policies::ApproximateTime<PointCloud2, CameraInfo> MySyncPolicy;
	//Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_cloud, info_sub_depth);
	//sync.registerCallback(boost::bind(&callback, _1, _2));

	std::cerr << "Plane detector initialized and listening..." << std::endl;
	ros::spin();

	return 1;
}
