/***********************************B*******************************************
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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/math/quaternion.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <srs_env_model_percp/ClearPlanes.h>

#include <srs_env_model_percp/but_segmentation/filtering.h>
#include <srs_env_model_percp/but_segmentation/normals.h>

#include <srs_env_model_percp/but_plane_detector/scene_model.h>
#include <srs_env_model_percp/but_plane_detector/dyn_model_exporter.h>
#include <srs_env_model_percp/but_plane_detector/plane_detector_params.h>

#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>

// PCL
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
#include <pcl/ModelCoefficients.h>

#include <cstdlib>
#include <cstdio>
#include <cfloat>

#include <float.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/math/quaternion.hpp>
#include "srs_env_model_percp/but_plane_detector/plane_detector_params.h"
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include <srs_env_model_percp/LoadSave.h>
#include <srs_env_model_percp/ResetPlanes.h>
using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace but_plane_detector;

namespace srs_env_model_percp
{
    int counter = 0;
    sensor_msgs::PointCloud2 cloud_msg;
    tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;

    tf::MessageFilter<Image> *transform_filterDepth;
    tf::MessageFilter<CameraInfo> *transform_filterCam;
    tf::MessageFilter<Image> *transform_filterRGB;
    tf::TransformListener *tfListener;
    tf::TransformListener *tfListenerCam;
    tf::TransformListener *tfListenerRGB;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;
    DynModelExporter *exporter = NULL;

    sensor_msgs::CameraInfo cam_info_legacy;
    sensor_msgs::CameraInfoConstPtr cam_info_aux (&cam_info_legacy);
    SceneModel *model;

    PlaneDetectorSettings settings;

    ros::NodeHandle *n = NULL;

    /**
     * Callback functions manage sync of messages
     */
    void callbackpcl(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& cam_info);
    bool clear(srs_env_model_percp::ClearPlanes::Request &req,srs_env_model_percp::ClearPlanes::Response &res);
    bool getParams(ros::NodeHandle nh);

    void getPlanes(Normals &normal, pcl::PointCloud<pcl::PointXYZ> &pointcloud, const sensor_msgs::ImageConstPtr* rgb = NULL)
    {
		// Add point cloud to current Hough space and recompute planes
		model->AddNext(normal, settings.param_ht_min_smooth);
		model->recomputePlanes( settings.param_search_minimum_current_space,
								settings.param_search_minimum_global_space,
								settings.param_search_maxima_search_blur,
								settings.param_search_maxima_search_neighborhood,
								settings.param_ht_step_substraction);

		// update current sent planes
		if (rgb && settings.param_visualisation_color == "mean_color")
		{
			cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(*rgb);
			cv::Mat rgb_mat = cv_image->image;
			exporter->update(model->planes, normal, settings.param_visualisation_color, rgb_mat);
		}
		else
		{
			exporter->update(model->planes, normal, settings.param_visualisation_color);
		}

		visualization_msgs::MarkerArray marker_array;
    	cob_3d_mapping_msgs::ShapeArray shape_array;
    	exporter->getMarkerArray(marker_array, settings.param_output_frame);
    	exporter->getShapeArray(shape_array, settings.param_output_frame);
    	std::cerr << "Total no of sent planes:  " << marker_array.markers.size() << std::endl;
    	pub2.publish(marker_array);
    	pub3.publish(shape_array);
    }

    void pcl_proc(const PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr* rgb = NULL)
    {
    	++counter;
   		ros::Time begin = ros::Time::now();

   		// make the pointcloud
   		pcl::PointCloud<pcl::PointXYZ> pointcloud;
   		pcl::fromROSMsg (*cloud, pointcloud);

   		// Control out
   		std::cerr << "Recieved frame..." << std::endl;
   		std::cerr << "Topic: " << pointcloud.header.frame_id << std::endl;
   		std::cerr << "Width: " << pointcloud.width << " height: " << pointcloud.height << std::endl;
   		std::cerr << "=========================================================" << endl;

   		// get indices of points to be deleted (max depth)
   		std::vector<unsigned int> zero_indices;
   		for (unsigned int i = 0; i < pointcloud.points.size(); ++i)
   		{
   			if (pointcloud.points[i].z > settings.param_ht_maxdepth)
   				zero_indices.push_back(i);
		}

   		// transform to world
   		tf::StampedTransform sensorToWorldTf;
   		try {
   			tfListener->waitForTransform(settings.param_output_frame, pointcloud.header.frame_id, pointcloud.header.stamp, ros::Duration(5.0));
   			tfListener->lookupTransform(settings.param_output_frame, pointcloud.header.frame_id, pointcloud.header.stamp, sensorToWorldTf);
   		}
   		catch(tf::TransformException& ex){
   			std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
   			return;
   		}

   		// transform pointcloud from sensor frame to fixed robot frame
   		Eigen::Matrix4f sensorToWorld;
   		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
   		pcl::transformPointCloud(pointcloud, pointcloud, sensorToWorld);

    	for (unsigned int i = 0; i < pointcloud.points.size(); ++i)
    	{
    		if (pointcloud.points[i].z > settings.param_ht_maxheight || pointcloud.points[i].z < settings.param_ht_minheight)
    			zero_indices.push_back(i);
    	}

    	// set all zero indices to point(0,0,0)
    	for (unsigned int i = 0; i < zero_indices.size(); ++i)
    	{
    		pointcloud.points[zero_indices[i]].x = 0.0;
    		pointcloud.points[zero_indices[i]].y = 0.0;
    		pointcloud.points[zero_indices[i]].z = 0.0;
    	}

    	// Compute normals on point cloud
    	Normals normal(pointcloud,
					   settings.param_normal_type,
					   settings.param_normal_neighborhood,
					   settings.param_normal_threshold,
					   settings.param_normal_outlierThreshold,
					   settings.param_normal_iter);

    	// get planes from point cloud
    	getPlanes(normal, pointcloud, rgb);

    	// Control out
    	ros::Time end = ros::Time::now();
    	std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
    	std::cerr << "=========================================================" << endl<< endl;
    }
    /**
	 * Callback function manages sync of messages
	 */
	void callbackpcl(const PointCloud2ConstPtr& cloud)
	{
		pcl_proc(cloud);
	}

	void callbackpcl_rgb(const PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& rgb)
	{
		ROS_INFO("Data received");
		pcl_proc(cloud, &rgb);
	}

	void kinect_proc( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info, const sensor_msgs::ImageConstPtr* rgb = NULL)
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

		pcl::PointCloud<PointXYZ> pointcloud;
		pointcloud.resize(depth.rows * depth.cols);
		pointcloud.width = depth.cols;
		pointcloud.height = depth.rows;
		pointcloud.header = dep->header;
		// get indices of points to be deleted (max depth)
		std::vector<unsigned int> zero_indices;

		unsigned short aux;
		for (int i = 0; i < depth.rows; ++i)
			for (int j = 0; j < depth.cols; ++j)
			{
				pcl::PointXYZ point(0.0, 0.0, 0.0);
				if ((aux = depth.at<unsigned short>(i, j)) != 0)
				{
					point.z = aux/1000.0;
					point.x = ( (j - cam_info->K[2]) * point.z / cam_info->K[0] );
					point.y = ( (i - cam_info->K[5]) * point.z / cam_info->K[4] );

					if (point.z > settings.param_ht_maxdepth)
						zero_indices.push_back(i);
				}
				else
					zero_indices.push_back(i);

				pointcloud[i * depth.rows + j] = point;
			}

		// transform to world
		tf::StampedTransform sensorToWorldTf;
		try {
			tfListener->waitForTransform(settings.param_output_frame, pointcloud.header.frame_id, pointcloud.header.stamp, ros::Duration(5.0));
			tfListener->lookupTransform(settings.param_output_frame, pointcloud.header.frame_id, pointcloud.header.stamp, sensorToWorldTf);
		}
		catch(tf::TransformException& ex){
			std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
			return;
		}

		// transform pointcloud from sensor frame to fixed robot frame
		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
		pcl::transformPointCloud(pointcloud, pointcloud, sensorToWorld);

		for (unsigned int i = 0; i < pointcloud.points.size(); ++i)
		{
			if (pointcloud.points[i].z > settings.param_ht_maxheight || pointcloud.points[i].z < settings.param_ht_minheight)
				zero_indices.push_back(i);
		}

		// set all zero indices to point(0,0,0)
		for (unsigned int i = 0; i < zero_indices.size(); ++i)
		{
			pointcloud.points[zero_indices[i]].x = 0.0;
			pointcloud.points[zero_indices[i]].y = 0.0;
			pointcloud.points[zero_indices[i]].z = 0.0;
		}

		Normals normal(pointcloud,
					   settings.param_normal_type,
					   settings.param_normal_neighborhood,
					   settings.param_normal_threshold,
					   settings.param_normal_outlierThreshold,
					   settings.param_normal_iter);
		getPlanes(normal, pointcloud, rgb);

		// Control out
		ros::Time end = ros::Time::now();
		std::cerr << "DONE.... Computation time: " << (end-begin).nsec/1000000.0 << " ms." << std::endl;
		std::cerr << "=========================================================" << endl<< endl;
	}
	void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info)
	{
		kinect_proc(dep, cam_info);
	}

	void callbackkinect_rgb( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info, const sensor_msgs::ImageConstPtr&rgb)
	{
		kinect_proc(dep, cam_info, &rgb);
	}


	bool getParams(ros::NodeHandle nh)
	{
		nh.param(PARAM_NODE_INPUT, settings.param_node_input, PARAM_NODE_INPUT_DEFAULT);

        nh.param(PARAM_NODE_OUTPUT_FRAME, settings.param_output_frame, PARAM_NODE_OUTPUT_FRAME_DEFAULT);
        nh.param(PARAM_NODE_ORIGINAL_FRAME, settings.param_original_frame, PARAM_NODE_ORIGINAL_FRAME_DEFAULT);

        nh.param(PARAM_HT_KEEPTRACK, settings.param_ht_keeptrack, PARAM_HT_KEEPTRACK_DEFAULT);
		nh.param(PARAM_HT_MAXDEPTH, settings.param_ht_maxdepth, PARAM_HT_MAXDEPTH_DEFAULT);
		nh.param(PARAM_HT_MAXHEIGHT, settings.param_ht_maxheight, PARAM_HT_MAXHEIGHT_DEFAULT);
		nh.param(PARAM_HT_MINHEIGHT, settings.param_ht_minheight, PARAM_HT_MINHEIGHT_DEFAULT);

		nh.param(PARAM_HT_MINSHIFT, settings.param_ht_minshift, PARAM_HT_MINSHIFT_DEFAULT);
		nh.param(PARAM_HT_MAXSHIFT, settings.param_ht_maxshift, PARAM_HT_MAXSHIFT_DEFAULT);
		nh.param(PARAM_HT_ANGLE_RES, settings.param_ht_angle_res, PARAM_HT_ANGLE_RES_DEFAULT);
		nh.param(PARAM_HT_SHIFT_RES, settings.param_ht_shift_res, PARAM_HT_SHIFT_RES_DEFAULT);
		nh.param(PARAM_HT_GAUSS_ANGLE_RES, settings.param_ht_gauss_angle_res, PARAM_HT_GAUSS_ANGLE_RES_DEFAULT);
		nh.param(PARAM_HT_GAUSS_SHIFT_RES, settings.param_ht_gauss_shift_res, PARAM_HT_GAUSS_SHIFT_RES_DEFAULT);
		nh.param(PARAM_HT_GAUSS_ANGLE_SIGMA, settings.param_ht_gauss_angle_sigma, PARAM_HT_GAUSS_ANGLE_SIGMA_DEFAULT);
		nh.param(PARAM_HT_GAUSS_SHIFT_SIGMA, settings.param_ht_gauss_shift_sigma, PARAM_HT_GAUSS_SHIFT_SIGMA_DEFAULT);
		nh.param(PARAM_HT_LVL1_GAUSS_ANGLE_RES, settings.param_ht_lvl1_gauss_angle_res, PARAM_HT_LVL1_GAUSS_ANGLE_RES_DEFAULT);
		nh.param(PARAM_HT_LVL1_GAUSS_SHIFT_RES, settings.param_ht_lvl1_gauss_shift_res, PARAM_HT_LVL1_GAUSS_SHIFT_RES_DEFAULT);
		nh.param(PARAM_HT_LVL1_GAUSS_ANGLE_SIGMA, settings.param_ht_lvl1_gauss_angle_sigma, PARAM_HT_LVL1_GAUSS_ANGLE_SIGMA_DEFAULT);
		nh.param(PARAM_HT_LVL1_GAUSS_SHIFT_SIGMA, settings.param_ht_lvl1_gauss_shift_sigma, PARAM_HT_LVL1_GAUSS_SHIFT_SIGMA_DEFAULT);
		nh.param(PARAM_HT_MIN_SMOOTH, settings.param_ht_min_smooth, PARAM_HT_MIN_SMOOTH_DEFAULT);

		nh.param(PARAM_HT_PLANE_MERGE_ANGLE, settings.param_ht_plane_merge_angle, PARAM_HT_PLANE_MERGE_ANGLE_DEFAULT);
		nh.param(PARAM_HT_PLANE_MERGE_SHIFT, settings.param_ht_plane_merge_shift, PARAM_HT_PLANE_MERGE_SHIFT_DEFAULT);
		nh.param(PARAM_HT_STEP_SUBSTRACTION, settings.param_ht_step_substraction, PARAM_HT_STEP_SUBSTRACTION_DEFAULT);

		nh.param(PARAM_VISUALISATION_DISTANCE, settings.param_visualisation_distance, PARAM_VISUALISATION_DISTANCE_DEFAULT);
		nh.param(PARAM_VISUALISATION_PLANE_NORMAL_DEV, settings.param_visualisation_plane_normal_dev, PARAM_VISUALISATION_PLANE_NORMAL_DEV_DEFAULT);
		nh.param(PARAM_VISUALISATION_PLANE_SHIFT_DEV, settings.param_visualisation_plane_shift_dev, PARAM_VISUALISATION_PLANE_SHIFT_DEV_DEFAULT);
        nh.param(PARAM_VISUALISATION_MIN_COUNT, settings.param_visualisation_min_count, PARAM_VISUALISATION_MIN_COUNT_DEFAULT);
        nh.param(PARAM_VISUALISATION_COLOR, settings.param_visualisation_color, PARAM_VISUALISATION_COLOR_DEFAULT);
        nh.param(PARAM_VISUALISATION_TTL, settings.param_visualisation_ttl, PARAM_VISUALISATION_TTL_DEFAULT);


		nh.param(PARAM_SEARCH_MINIMUM_CURRENT_SPACE, settings.param_search_minimum_current_space, PARAM_SEARCH_MINIMUM_CURRENT_SPACE_DEFAULT);
		nh.param(PARAM_SEARCH_MINIMUM_GLOBAL_SPACE, settings.param_search_minimum_global_space, PARAM_SEARCH_MINIMUM_GLOBAL_SPACE_DEFAULT);
		nh.param(PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD, settings.param_search_maxima_search_neighborhood, PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD_DEFAULT);
		nh.param(PARAM_SEARCH_MAXIMA_SEARCH_BLUR, settings.param_search_maxima_search_blur, PARAM_SEARCH_MAXIMA_SEARCH_BLUR_DEFAULT);

		nh.param(PARAM_NORMAL_ITER, settings.param_normal_iter, PARAM_NORMAL_ITER_DEFAULT);
		nh.param(PARAM_NORMAL_NEIGHBORHOOD, settings.param_normal_neighborhood, PARAM_NORMAL_NEIGHBORHOOD_DEFAULT);
		nh.param(PARAM_NORMAL_OUTLIERTHRESH, settings.param_normal_outlierThreshold, PARAM_NORMAL_OUTLIERTHRESH_DEFAULT);
		nh.param(PARAM_NORMAL_THRESHOLD, settings.param_normal_threshold, PARAM_NORMAL_THRESHOLD_DEFAULT);
		nh.param(PARAM_NORMAL_TYPE, settings.param_normal_type, PARAM_NORMAL_TYPE_DEFAULT);


		return true;
	}

	bool onReset(srs_env_model_percp::ResetPlanes::Request &req, srs_env_model_percp::ResetPlanes::Response &res)
	{
		std::cout << "Resetting plane settings..." << std::endl;

		visualization_msgs::MarkerArray marker_array;
		exporter->getMarkerArray(marker_array, settings.param_output_frame);

		for (unsigned int i = 0; i < marker_array.markers.size(); ++i)
			marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

		std::cerr << "Total no of deleted planes:  " << marker_array.markers.size() << std::endl;
		pub2.publish(marker_array);



		delete model;
		model = new SceneModel(	settings.param_ht_maxdepth,
				settings.param_ht_minshift,
				settings.param_ht_maxshift,
				settings.param_ht_angle_res,
				settings.param_ht_shift_res,
				settings.param_ht_gauss_angle_res,
				settings.param_ht_gauss_shift_res,
				settings.param_ht_gauss_angle_sigma,
				settings.param_ht_gauss_shift_sigma,
				settings.param_ht_lvl1_gauss_angle_res,
				settings.param_ht_lvl1_gauss_shift_res,
				settings.param_ht_lvl1_gauss_angle_sigma,
				settings.param_ht_lvl1_gauss_shift_sigma,
				settings.param_ht_plane_merge_angle,
				settings.param_ht_plane_merge_shift);

		delete exporter;
		exporter = new DynModelExporter(n,
									   settings.param_original_frame,
			                           settings.param_output_frame,
			                           settings.param_visualisation_min_count,
									   settings.param_visualisation_distance,
									   settings.param_visualisation_plane_normal_dev,
									   settings.param_visualisation_plane_shift_dev,
									   settings.param_ht_keeptrack,
									   settings.param_visualisation_ttl);

		res.message = "Hough space successfully reset.\n";
		std::cout << "Hough space successfully reset." << std::endl;
		return true;
	}

	bool onSave(srs_env_model_percp::LoadSave::Request &req, srs_env_model_percp::LoadSave::Response &res)
	{
		std::cerr << "Saving planes...." << std::endl;
		exporter->xmlFileExport(req.filename);


		res.all_ok = 1;
		std::cerr << "Environment model successfuly saved into " << req.filename << "." << std::endl;
		return true;
	}

	bool onLoad(srs_env_model_percp::LoadSave::Request &req, srs_env_model_percp::LoadSave::Response &res)
	{
		std::cerr << "Loading planes...." << std::endl;

		visualization_msgs::MarkerArray marker_array;
		exporter->getMarkerArray(marker_array, settings.param_output_frame);

		for (unsigned int i = 0; i < marker_array.markers.size(); ++i)
			marker_array.markers[i].action = visualization_msgs::Marker::DELETE;

		std::cerr << "Total no of deleted planes:  " << marker_array.markers.size() << std::endl;
		pub2.publish(marker_array);

		delete model;
		model = new SceneModel(	settings.param_ht_maxdepth,
						settings.param_ht_minshift,
						settings.param_ht_maxshift,
						settings.param_ht_angle_res,
						settings.param_ht_shift_res,
						settings.param_ht_gauss_angle_res,
						settings.param_ht_gauss_shift_res,
						settings.param_ht_gauss_angle_sigma,
						settings.param_ht_gauss_shift_sigma,
						settings.param_ht_lvl1_gauss_angle_res,
						settings.param_ht_lvl1_gauss_shift_res,
						settings.param_ht_lvl1_gauss_angle_sigma,
						settings.param_ht_lvl1_gauss_shift_sigma,
						settings.param_ht_plane_merge_angle,
						settings.param_ht_plane_merge_shift);

		delete exporter;
		exporter = new DynModelExporter(n,
									   settings.param_original_frame,
			                           settings.param_output_frame,
			                           settings.param_visualisation_min_count,
			                           settings.param_visualisation_distance,
									   settings.param_visualisation_plane_normal_dev,
									   settings.param_visualisation_plane_shift_dev,
									   settings.param_ht_keeptrack,
									   settings.param_visualisation_ttl);

		exporter->xmlFileImport(req.filename);

		marker_array.markers.clear();
		cob_3d_mapping_msgs::ShapeArray shape_array;
		exporter->getMarkerArray(marker_array, settings.param_output_frame);
		exporter->getShapeArray(shape_array, settings.param_output_frame);
		std::cerr << "Total no of sent planes:  " << marker_array.markers.size() << std::endl;
		pub2.publish(marker_array);
		pub3.publish(shape_array);


		res.all_ok = 1;
		std::cerr << "Environment model successfuly loaded from " << req.filename << "." << std::endl;
		return true;
	}

}


/**
 * Main detector module body
 */
int main( int argc, char** argv )
{
	using namespace srs_env_model_percp;

	ros::init(argc, argv, "but_plane_detector");

	n = new ros::NodeHandle();
	// Private node handle to read private node parameters
	ros::NodeHandle private_nh("~");
	getParams(private_nh);

	// Init scene model
	model = new SceneModel(	settings.param_ht_maxdepth,
							settings.param_ht_minshift,
							settings.param_ht_maxshift,
							settings.param_ht_angle_res,
							settings.param_ht_shift_res,
							settings.param_ht_gauss_angle_res,
							settings.param_ht_gauss_shift_res,
							settings.param_ht_gauss_angle_sigma,
							settings.param_ht_gauss_shift_sigma,
							settings.param_ht_lvl1_gauss_angle_res,
							settings.param_ht_lvl1_gauss_shift_res,
							settings.param_ht_lvl1_gauss_angle_sigma,
							settings.param_ht_lvl1_gauss_shift_sigma,
							settings.param_ht_plane_merge_angle,
							settings.param_ht_plane_merge_shift);

	// Print out parameters
	std::cerr << std::endl;
	std::cerr << "Given HS parameters:" << std::endl;
	std::cerr << "====================" << std::endl;
	std::cerr << "Max point cloud z value:               " << settings.param_ht_maxdepth << std::endl;
	std::cerr << "Minimal plane shift (d param) value:   " << settings.param_ht_minshift << std::endl;
	std::cerr << "Maximal plane shift (d param) value:   " << settings.param_ht_maxshift << std::endl;
	std::cerr << "Hough space angle resolution:          " << settings.param_ht_angle_res << std::endl;
	std::cerr << "Hough space shift resolution:          " << settings.param_ht_shift_res << std::endl;
	std::cerr << "Plane Gauss function angle resolution: " << settings.param_ht_gauss_angle_res << std::endl;
	std::cerr << "Plane Gauss function shift resolution: " << settings.param_ht_gauss_shift_res << std::endl;
	std::cerr << "Plane Gauss function angle sigma:      " << settings.param_ht_gauss_angle_sigma << std::endl;
	std::cerr << "Plane Gauss function shift sigma:      " << settings.param_ht_gauss_shift_sigma << std::endl;
	std::cerr << std::endl;
	std::cerr << "Given plane search parameters:" << std::endl;
	std::cerr << "==============================" << std::endl;
	std::cerr << "Minimum plane value in HS for current frame:   " << settings.param_search_minimum_current_space << std::endl;
	std::cerr << "Minimum plane value in HS for global space:    " << settings.param_search_minimum_global_space << std::endl;
	std::cerr << "Hough space local maximum neighborhood search: " << settings.param_search_maxima_search_neighborhood << std::endl;
	std::cerr << "Hough space local maximum search blur:         " << settings.param_search_maxima_search_blur << std::endl;
	std::cerr << std::endl;
	std::cerr << "Given visualisation parameters:" << std::endl;
	std::cerr << "==============================" << std::endl;
	std::cerr << "Maximum point distance from plane:                               " << settings.param_visualisation_distance << std::endl;
	std::cerr << "Maximum point normal deviation from plane:                       " << settings.param_visualisation_plane_normal_dev << std::endl;
	std::cerr << "Maximum point plane shift (d param) from plane:                  " << settings.param_visualisation_plane_shift_dev << std::endl;
	std::cerr << "Minimum point count for visualised plane in current point cloud: " << settings.param_visualisation_min_count << std::endl;
	std::cerr << std::endl;

    ROS_INFO("Plane det. input: %s", settings.param_node_input.c_str());   
    ROS_INFO("Plane coloring: %s", settings.param_visualisation_color.c_str());   

	// if PCL input
	if (settings.param_node_input == PARAM_NODE_INPUT_PCL)
	{
		exporter = new DynModelExporter(n,
										settings.param_original_frame,
	                                    settings.param_output_frame,
										settings.param_visualisation_min_count,
										settings.param_visualisation_distance,
										settings.param_visualisation_plane_normal_dev,
										settings.param_visualisation_plane_shift_dev,
										settings.param_ht_keeptrack,
										settings.param_visualisation_ttl);

		// MESSAGES
		message_filters::Subscriber<PointCloud2 > point_cloud(*n, DET_INPUT_POINT_CLOUD_TOPIC, 10);

		pub1 = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> > (DET_OUTPUT_POINT_CLOUD_TOPIC, 1);
		pub2 = n->advertise<visualization_msgs::MarkerArray > (DET_OUTPUT_MARKER_TOPIC, 1);
		pub3 = n->advertise<cob_3d_mapping_msgs::ShapeArray > (DET_OUTPUT_MARKER_SRS_TOPIC, 1);

		ros::ServiceServer service1 = n->advertiseService(DET_SERVICE_RESET_PLANES, onReset);
		ros::ServiceServer service2 = n->advertiseService(DET_SERVICE_SAVE_PLANES, onSave);
		ros::ServiceServer service3 = n->advertiseService(DET_SERVICE_LOAD_PLANES, onLoad);

		if (settings.param_visualisation_color == "mean_color")
		{
			// sync images
			tfListener = new tf::TransformListener();
			transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2> (point_cloud, *tfListener, settings.param_output_frame, 10);

			message_filters::Subscriber<Image> sub_rgb(*n, DET_INPUT_RGB_IMAGE_TOPIC, 10);

            // Majkl 2012/11/07: TimeSyncrhonizer doesn't work when playing data from a bag file
//			message_filters::TimeSynchronizer<PointCloud2, Image> sync(*transform_filter, sub_rgb, 10);
            typedef sync_policies::ApproximateTime<PointCloud2, Image> tSyncPolicy;
            Synchronizer<tSyncPolicy> sync(tSyncPolicy(10), *transform_filter, sub_rgb);
 
			// register callback called when everything synchronized arrives
			sync.registerCallback(boost::bind(&callbackpcl_rgb, _1, _2));

			std::cerr << "Plane detector initialized and listening point clouds..." << std::endl;
			ros::spin();

			return 1;
		}
		else
		{
			// sync images
			tfListener = new tf::TransformListener();
			transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2> (point_cloud, *tfListener, settings.param_output_frame, 10);
			transform_filter->registerCallback(boost::bind(&callbackpcl, _1));

			std::cerr << "Plane detector initialized and listening point clouds..." << std::endl;
			ros::spin();

			return 1;
		}
	}

	// if kinect input
	else if (settings.param_node_input == PARAM_NODE_INPUT_KINECT)
	{
		exporter = new DynModelExporter(n,
										settings.param_original_frame,
	                                    settings.param_output_frame,
										settings.param_visualisation_min_count,
										settings.param_visualisation_distance,
										settings.param_visualisation_plane_normal_dev,
										settings.param_visualisation_plane_shift_dev,
										settings.param_ht_keeptrack,
										settings.param_visualisation_ttl);

		// MESSAGES
		message_filters::Subscriber<Image> depth_sub(*n, DET_INPUT_IMAGE_TOPIC, 10);
		message_filters::Subscriber<CameraInfo> info_sub_depth(*n, DET_INPUT_CAM_INFO_TOPIC, 10);

		tfListener = new tf::TransformListener();
		transform_filterDepth = new tf::MessageFilter<Image> (depth_sub, *tfListener, settings.param_output_frame, 10);

		tfListenerCam = new tf::TransformListener();
		transform_filterCam = new tf::MessageFilter<CameraInfo> (info_sub_depth, *tfListenerCam, settings.param_output_frame, 10);

		pub1 = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> > (DET_OUTPUT_POINT_CLOUD_TOPIC, 1);
		pub2 = n->advertise<visualization_msgs::MarkerArray > (DET_OUTPUT_MARKER_TOPIC, 1);
		pub3 = n->advertise<cob_3d_mapping_msgs::ShapeArray > (DET_OUTPUT_MARKER_SRS_TOPIC, 1);

		if (settings.param_visualisation_color == "mean_color")
		{
			message_filters::Subscriber<Image> sub_rgb(*n, DET_INPUT_RGB_IMAGE_TOPIC, 10);

			tfListenerRGB = new tf::TransformListener();
			transform_filterRGB = new tf::MessageFilter<Image> (sub_rgb, *tfListenerRGB, settings.param_output_frame, 10);

            // Majkl 2012/11/07: TimeSynchronizer doesn't work when playing data from a bag file
//			message_filters::TimeSynchronizer<Image, CameraInfo, Image> sync(*transform_filterDepth, *transform_filterCam, *transform_filterRGB, 10);
            typedef sync_policies::ApproximateTime<Image, CameraInfo, Image> tSyncPolicy;
            Synchronizer<tSyncPolicy> sync(tSyncPolicy(10), *transform_filterDepth, *transform_filterCam, *transform_filterRGB);

			// register callback called when everything synchronized arrives
			sync.registerCallback(boost::bind(&callbackkinect_rgb, _1, _2, _3));

			std::cerr << "Plane detector initialized and listening depth images..." << std::endl;
			ros::spin();

			return 1;
		}
		else
		{
            // Majkl 2012/11/07: TimeSynchronizer doesn't work when playing data from a bag file
//			message_filters::TimeSynchronizer<Image, CameraInfo> sync(*transform_filterDepth, *transform_filterCam, 10);
            typedef sync_policies::ApproximateTime<Image, CameraInfo> tSyncPolicy;
            Synchronizer<tSyncPolicy> sync(tSyncPolicy(10), *transform_filterDepth, *transform_filterCam);

			// register callback called when everything synchronized arrives
			sync.registerCallback(boost::bind(&callbackkinect, _1, _2));

			std::cerr << "Plane detector initialized and listening depth images..." << std::endl;
			ros::spin();

			return 1;
		}
	}
}

