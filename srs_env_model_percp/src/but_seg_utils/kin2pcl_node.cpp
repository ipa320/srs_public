/******************************************************************************
 * \file
 *
 * $Id: kin2pcl.cpp 619 2012-04-16 13:47:28Z ihulik $
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
 * Module exports depth map images point cloud messages
 *
 */

#include <srs_env_model_percp/but_seg_utils/kin2pcl_node.h>
#include <srs_env_model_percp/but_segmentation/normals.h>

// CV <-> ROS bridge
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace but_plane_detector;

namespace srs_env_model_percp
{

	void callback( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info)
	{
		ros::Time begin = ros::Time::now();
		//  Debug info
		std::cerr << "Recieved frame..." << std::endl;
		std::cerr << "Cam info: fx:" << cam_info->K[0] << " fy:" << cam_info->K[4] << " cx:" << cam_info->K[2] <<" cy:" << cam_info->K[5] << std::endl;
		std::cerr << "Depth image h:" << dep->height << " w:" << dep->width << " e:" << dep->encoding << " " << dep->step << endl;

		//get image from message
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(dep);
		cv::Mat depth = cv_image->image;

		Normals normal(depth, cam_info);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		for (int i = 0; i < normal.m_points.rows; ++i)
		for (int j = 0; j < normal.m_points.cols; ++j)
		{
			Vec3f vector = normal.m_points.at<Vec3f>(i, j);

			//pcl::Vec
			cloud->push_back(pcl::PointXYZ(vector[0], vector[1], vector[2]));
		}

		pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
		voxelgrid.setInputCloud(cloud);
		voxelgrid.setLeafSize(0.05, 0.05, 0.05);
		voxelgrid.filter(*cloud);

		cloud->header.frame_id = OUTPUT_POINT_CLOUD_FRAMEID;

		pub.publish(cloud);


		ros::Time end = ros::Time::now();
		std::cerr << "Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
		std::cerr << "=========================================================" << endl;
	}
}

int main( int argc, char** argv )
{
	using namespace srs_env_model_percp;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;

	// subscribe depth info
	message_filters::Subscriber<Image> depth_sub(n, INPUT_IMAGE_TOPIC, 1);
	message_filters::Subscriber<CameraInfo> info_sub_depth(n, INPUT_CAM_INFO_TOPIC, 1);

	pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > (OUTPUT_POINT_CLOUD_TOPIC, 1);
	// sync images
	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, info_sub_depth);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	std::cerr << "Kinect depth map to point cloud converter initialized and listening..." << std::endl;
	ros::spin();

	return 1;
}
