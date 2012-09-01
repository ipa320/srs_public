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

//#include <but_segmentation/filtering.h>
//#include <but_segmentation/normals.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/math/quaternion.hpp>
#include "srs_env_model_percp/but_plane_detector/plane_detector_params.h"
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
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
    tf::TransformListener *tfListener;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;
    DynModelExporter *exporter = NULL;

    sensor_msgs::CameraInfo cam_info_legacy;
    sensor_msgs::CameraInfoConstPtr cam_info_aux (&cam_info_legacy);
    SceneModel *model;

    PlaneDetectorSettings settings;

    /**
     * Callback functions manage sync of messages
     */
    void callbackpcl(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void callbackkinect( const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& cam_info);
    bool clear(srs_env_model_percp::ClearPlanes::Request &req,srs_env_model_percp::ClearPlanes::Response &res);
    bool getParams(ros::NodeHandle nh);

    /**
	 * Callback function manages sync of messages
	 */
	void callbackpcl(const PointCloud2ConstPtr& cloud)
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

		// transform to world
		tf::StampedTransform sensorToWorldTf;
		try {
			tfListener->waitForTransform(settings.param_output_frame, cloud->header.frame_id, cloud->header.stamp, ros::Duration(2.0));
			tfListener->lookupTransform(settings.param_output_frame, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
		}
		catch(tf::TransformException& ex){
			std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
			return;
		}

		// transform pointcloud from sensor frame to fixed robot frame
		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
		pcl::transformPointCloud(pointcloud, pointcloud, sensorToWorld);

		// Compute normals on point cloud
		Normals normal(pointcloud);

		// Add point cloud to current Hough space and recompute planes
		model->AddNext(normal);
		model->recomputePlanes( settings.param_search_minimum_current_space,
								settings.param_search_minimum_global_space,
								settings.param_search_maxima_search_blur,
								settings.param_search_maxima_search_neighborhood);

				// TODO debug point cloud output
				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				pcl::PointCloud<pcl::PointXYZRGB> outcloud;
				outcloud.header.frame_id = settings.param_output_frame;

				// Majkl's note: The following line has been commented out and modified because
				// it's necessary to use aligned allocator when creating STL containers and
				// classes containing Eigen types.
//				std::vector<pcl::PointCloud<pcl::PointXYZ> > planecloud(model->planes.size());

				typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
				typedef std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > PlaneCloud;
				PlaneCloud planecloud(model->planes.size());

				for (int i = 0; i < normal.m_points.rows; ++i)
				for (int j = 0; j < normal.m_points.cols; ++j)
				{
					Vec3f point = normal.m_points.at<Vec3f>(i, j);
					cv::Vec4f localPlane = normal.m_planes.at<cv::Vec4f>(i, j);
					Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
					double dist = DBL_MAX;
					int chosen = -1;
					// find the best plane
					for (unsigned int a = 0; a < model->planes.size(); ++a)
					{
						if (model->planes[a].distance(point) < dist && model->planes[a].distance(point) < settings.param_visualisation_distance &&
							model->planes[a].isSimilar(aaa, settings.param_visualisation_plane_normal_dev, settings.param_visualisation_plane_shift_dev))
						{
							dist = model->planes[a].distance(point);
							chosen = a;
						}
					}
					// if there is good plane, insert point into point cloud
					if (chosen > -1)
					{
						pcl::PointXYZRGB current;
						current.x = point[0];
						current.y = point[1];
						current.z = point[2];
						// some not good debug info about planes
						current.r = abs(model->planes[chosen].a)*255;
						current.g = abs(model->planes[chosen].b)*255;
						current.b = abs(model->planes[chosen].d)*255;
						outcloud.push_back(current);

						pcl::PointXYZ current2;
						current2.x = point[0];
						current2.y = point[1];
						current2.z = point[2];
						planecloud[chosen].points.push_back(current2);


					}
				}
				pub1.publish(outcloud);
				////////////////////////////////////////////////////////////////////////////////////////////////////////////

//			visualization_msgs::MarkerArray marker_array;
//			for (unsigned int i = 0; i < planecloud.size(); ++i)
//			{
//				if (planecloud[i].points.size() > 20)
//				{
//				pcl::ModelCoefficients coefs;
//				coefs.values.push_back(model->planes[i].a);
//				coefs.values.push_back(model->planes[i].b);
//				coefs.values.push_back(model->planes[i].c);
//				coefs.values.push_back(model->planes[i].d);
//
//				visualization_msgs::Marker marker;
//				DynModelExporter::createMarkerForConvexHull(planecloud[i], coefs, marker);
//				marker.header.frame_id = settings.param_output_frame;
//				marker.header.stamp = pointcloud.header.stamp;
//				marker.id = i;
//				marker.ns = "Normals";
//				marker.pose.position.x = 0.0;
//				marker.pose.position.y = 0.0;
//				marker.pose.position.z = 0.0;
//				marker.pose.orientation.x = 0.0;
//				marker.pose.orientation.y = 0.0;
//				marker.pose.orientation.z = 0.0;
//				marker.pose.orientation.w = 1.0;
//				marker.scale.x = 1;
//				marker.scale.y = 1;
//				marker.scale.z = 1;
//				marker.color.r = 0.5;
//				marker.color.g = 0.0;
//				marker.color.b = 0.0;
//				marker.color.a = 0.8;
//				marker_array.markers.push_back(marker);
//				}
//			}
//			pub2.publish(marker_array);
		// update planes on server
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
//		//  Debug info
//		std::cerr << "Recieved frame..." << std::endl;
//		std::cerr << "Cam info: fx:" << cam_info->K[0] << " fy:" << cam_info->K[4] << " cx:" << cam_info->K[2] <<" cy:" << cam_info->K[5] << std::endl;
//		std::cerr << "Depth image h:" << dep->height << " w:" << dep->width << " e:" << dep->encoding << " " << dep->step << endl;
//		std::cerr << "=========================================================" << endl;
//
//		//get image from message
//		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(dep);
//		cv::Mat depth = cv_image->image;
//
//
//		Normals normal(depth, cam_info, NormalType::LSQAROUND);
//
//		model->AddNext(depth, cam_info_aux, normal);
//		model->recomputePlanes();
//
//		model->scene_cloud->header.frame_id = "/head_cam3d_link";
//
//		pub1.publish(model->scene_cloud);
//		//	pub2.publish(model->current_hough_cloud);
//		//exporter->update(model->planes, normal);

		//get image from message
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(dep);
		cv::Mat depth = cv_image->image;

		pcl::PointCloud<pcl::PointXYZ> pointcloud;
		pointcloud.resize(depth.rows*depth.cols/16);
		pointcloud.height = depth.rows/4;
		pointcloud.width = depth.cols/4;
		pointcloud.header.frame_id = dep->header.frame_id;
		pointcloud.header.stamp = dep->header.stamp;

		for (unsigned int w = 0; w < pointcloud.width; ++w)
		for (unsigned int h = 0; h < pointcloud.height; ++h)
		{
			if (w % 4 == 0 && h % 4 == 0)
			{
			float aux = depth.at<unsigned short>(h, w);
			PointXYZ realPoint;

			realPoint.z = aux/1000.0;
			realPoint.x = ( (w - cam_info->K[2]) * realPoint.z / cam_info->K[0] );
			realPoint.y = ( (h - cam_info->K[5]) * realPoint.z / cam_info->K[4] );

			pointcloud.points[h/4 * pointcloud.width + w/4] = realPoint;
			depth.at<unsigned short>(h, w) = realPoint.z/3.5 * 255;
			}
		}

		ros::Time testbegin = ros::Time::now();
		// Compute normals on point cloud
		Normals normal(pointcloud);

		// Add point cloud to current Hough space and recompute planes
		model->AddNext(normal);
		model->recomputePlanes( settings.param_search_minimum_current_space,
								settings.param_search_minimum_global_space,
								settings.param_search_maxima_search_blur,
								settings.param_search_maxima_search_neighborhood);
		/////////////////////////////////////////////t///////////////////////
		ros::Time testend = ros::Time::now();
		std::cout.setf(ios::fixed, ios::floatfield);
		std::cout.precision(10);

				// TODO debug point cloud output
				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				pcl::PointCloud<pcl::PointXYZRGB> outcloud;
					outcloud.header.frame_id = settings.param_output_frame;

						std::vector<pcl::PointCloud<pcl::PointXYZ> > planecloud(model->planes.size());

						for (int i = 0; i < normal.m_points.rows; ++i)
						for (int j = 0; j < normal.m_points.cols; ++j)
						{
							Vec3f point = normal.m_points.at<Vec3f>(i, j);
							cv::Vec4f localPlane = normal.m_planes.at<cv::Vec4f>(i, j);
							Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
							double dist = DBL_MAX;
							int chosen = -1;
							// find the best plane
							for (unsigned int a = 0; a < model->planes.size(); ++a)
							{
								if (model->planes[a].distance(point) < dist && model->planes[a].distance(point) < settings.param_visualisation_distance &&
									model->planes[a].isSimilar(aaa, 0.4, settings.param_visualisation_plane_shift_dev))
								{
									dist = model->planes[a].distance(point);
									chosen = a;
								}
							}
							// if there is good plane, insert point into point cloud
							if (chosen > -1)
							{
								pcl::PointXYZRGB current;
								current.x = point[0];
								current.y = point[1];
								current.z = point[2];
								// some not good debug info about planes
								current.r = abs(model->planes[chosen].a)*255;
								current.g = abs(model->planes[chosen].b)*255;
								current.b = abs(model->planes[chosen].d)*255;
								outcloud.push_back(current);

								pcl::PointXYZ current2;
								current2.x = point[0];
								current2.y = point[1];
								current2.z = point[2];
								planecloud[chosen].points.push_back(current2);


							}
						}
						pub1.publish(outcloud);
						////////////////////////////////////////////////////////////////////////////////////////////////////////////

					visualization_msgs::MarkerArray marker_array;
					visualization_msgs::Marker marker;
					marker.header.frame_id = settings.param_output_frame;
					marker.header.stamp = pointcloud.header.stamp;
					marker.id = 1;
					marker.ns = "Normals";
					marker.pose.position.x = 0.0;
					marker.pose.position.y = 0.0;
					marker.pose.position.z = 0.0;
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 1;
					marker.color.r = 0.5;
					marker.color.g = 0.0;
					marker.color.b = 0.0;
					marker.color.a = 0.8;
					for (unsigned int i = 0; i < planecloud.size(); ++i)
					{
						if (planecloud[i].points.size() > 20)
						{
						pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);;
						coefs->values.push_back(model->planes[i].a);
						coefs->values.push_back(model->planes[i].b);
						coefs->values.push_back(model->planes[i].c);
						coefs->values.push_back(model->planes[i].d);

						std::ofstream fileout("out_node.csv", std::ofstream::ios_base::app);

												fileout << "Hough node;" << pointcloud.header.stamp.nsec << ";" << model->planes[i].a << ";" << model->planes[i].b << ";"
														<< model->planes[i].c << ";" << model->planes[i].d << ";" << (testend-testbegin).nsec
														<< ";" << model->space.getSize()*sizeof(double) << std::endl;


												fileout.close();

						DynModelExporter::createMarkerForConvexHull(planecloud[i], coefs, marker);

						//marker_array.markers.push_back(marker);
						}
					}
					pub2.publish(marker);
					pub1.publish(outcloud);

					cv_image->image = depth;
					pub3.publish(cv_image->toImageMsg());
				// update planes on server
				//exporter->update(model->planes, normal, sensorToWorldTf);
		// Control out
		std::cerr << "Kinect input not ready yet.... doing nothing." << std::endl;
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


	bool getParams(ros::NodeHandle nh)
	{
		nh.param(PARAM_NODE_INPUT, settings.param_node_input, PARAM_NODE_INPUT_DEFAULT);

        nh.param(PARAM_NODE_OUTPUT_FRAME, settings.param_output_frame, PARAM_NODE_OUTPUT_FRAME_DEFAULT);
        nh.param(PARAM_NODE_ORIGINAL_FRAME, settings.param_original_frame, PARAM_NODE_ORIGINAL_FRAME_DEFAULT);

		nh.param(PARAM_HT_MAXDEPTH, settings.param_ht_maxdepth, PARAM_HT_MAXDEPTH_DEFAULT);
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

		nh.param(PARAM_VISUALISATION_DISTANCE, settings.param_visualisation_distance, PARAM_VISUALISATION_DISTANCE_DEFAULT);
		nh.param(PARAM_VISUALISATION_PLANE_NORMAL_DEV, settings.param_visualisation_plane_normal_dev, PARAM_VISUALISATION_PLANE_NORMAL_DEV_DEFAULT);
		nh.param(PARAM_VISUALISATION_PLANE_SHIFT_DEV, settings.param_visualisation_plane_shift_dev, PARAM_VISUALISATION_PLANE_SHIFT_DEV_DEFAULT);
        nh.param(PARAM_VISUALISATION_MIN_COUNT, settings.param_visualisation_min_count, PARAM_VISUALISATION_MIN_COUNT_DEFAULT);

		nh.param(PARAM_SEARCH_MINIMUM_CURRENT_SPACE, settings.param_search_minimum_current_space, PARAM_SEARCH_MINIMUM_CURRENT_SPACE_DEFAULT);
		nh.param(PARAM_SEARCH_MINIMUM_GLOBAL_SPACE, settings.param_search_minimum_global_space, PARAM_SEARCH_MINIMUM_GLOBAL_SPACE_DEFAULT);
		nh.param(PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD, settings.param_search_maxima_search_neighborhood, PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD_DEFAULT);
		nh.param(PARAM_SEARCH_MAXIMA_SEARCH_BLUR, settings.param_search_maxima_search_blur, PARAM_SEARCH_MAXIMA_SEARCH_BLUR_DEFAULT);

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
	ros::NodeHandle n;

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
							settings.param_ht_lvl1_gauss_shift_sigma);
	ros::ServiceServer service = n.advertiseService(DET_SERVICE_CLEAR_PLANES, clear);

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

	// if PCL input
	if (settings.param_node_input == PARAM_NODE_INPUT_PCL)
	{
		exporter = new DynModelExporter(&n,
		                                settings.param_original_frame,
                                        settings.param_output_frame,
										settings.param_visualisation_min_count,
										settings.param_visualisation_distance,
										settings.param_visualisation_plane_normal_dev,
										settings.param_visualisation_plane_shift_dev);

		// MESSAGES
		message_filters::Subscriber<PointCloud2 > point_cloud(n, DET_INPUT_POINT_CLOUD_TOPIC, 1);

		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (DET_OUTPUT_POINT_CLOUD_TOPIC, 1);
		pub2 = n.advertise<visualization_msgs::MarkerArray > (DET_OUTPUT_MARKER_TOPIC, 1);

		// sync images
		tfListener = new tf::TransformListener();
		transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2> (point_cloud, *tfListener, settings.param_output_frame, 1);
		transform_filter->registerCallback(boost::bind(&callbackpcl, _1));

		std::cerr << "Plane detector initialized and listening point clouds..." << std::endl;
		ros::spin();

		return 1;
	}

	// if kinect input
	else if (settings.param_node_input == PARAM_NODE_INPUT_KINECT)
	{
		exporter = new DynModelExporter(&n,
                                        settings.param_original_frame,
                                        settings.param_output_frame,
                                        settings.param_visualisation_min_count,
                                        settings.param_visualisation_distance,
                                        settings.param_visualisation_plane_normal_dev,
                                        settings.param_visualisation_plane_shift_dev);

		// MESSAGES
		message_filters::Subscriber<Image> depth_sub(n, DET_INPUT_IMAGE_TOPIC, 1);
		message_filters::Subscriber<CameraInfo> info_sub_depth(n, DET_INPUT_CAM_INFO_TOPIC, 1);

		pub1 = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (DET_OUTPUT_POINT_CLOUD_TOPIC, 1);
		pub2 = n.advertise<visualization_msgs::MarkerArray > (DET_OUTPUT_MARKER_TOPIC, 1);
		pub3 = n.advertise<Image> (DET_OUTPUT_IMAGE_TOPIC, 1);

		typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, info_sub_depth);
		sync.registerCallback(boost::bind(&callbackkinect, _1, _2));
		std::cerr << "Plane detector initialized and listening depth images..." << std::endl;
		ros::spin();

		return 1;
	}
}
