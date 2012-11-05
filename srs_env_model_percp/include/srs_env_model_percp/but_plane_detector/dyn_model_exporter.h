/******************************************************************************
 * \file
 *
 * $Id: dynModelExporter.h 693 2012-04-20 09:22:39Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
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
 *	 Encapsulates a class of plane exporter (export to but_gui module/interactive markers)
 */

#pragma once
#ifndef BUT_PLANE_DET_DYNMODELEXPORTER_H
#define BUT_PLANE_DET_DYNMODELEXPORTER_H

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <opencv2/highgui/highgui.hpp>

#include <visualization_msgs/Marker.h>

// but_scenemodel
#include <srs_env_model_percp/but_segmentation/normals.h>


namespace srs_env_model_percp
{
	/**
	 * Encapsulates a class of plane exporter (export to but_gui module/interactive markers)
	 */

	class DynModelExporter
	{
		public:
			typedef but_plane_detector::Plane<float> tPlane;
			typedef std::vector<tPlane, Eigen::aligned_allocator<tPlane> > tPlanes;

			/**
			 * Initialization
			 */
			DynModelExporter(ros::NodeHandle *node,
                             const std::string& original_frame,
			                 const std::string& output_frame,
			                 int minOutputCount,
			                 double max_distance,
			                 double max_plane_normal_dev,
			                 double max_plane_shift_dev
			                 );

			/**
			 * Updates sent planes using but environment model server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param sensorToWorldTf Sendor to map transformation matrix
			 */
			void update(tPlanes & planes, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf);
			
			/**
			 * Updates sent planes using but environment model server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 */
			void update(tPlanes & planes, but_plane_detector::Normals &normals, tf::StampedTransform &sensorToWorldTf);

			/**
			 * Updates sent planes using direct but interactive marker server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param sensorToWorldTf Sendor to map transformation matrix
			 */
			void updateDirect(tPlanes & planes, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf);

			static void createMarkerForConvexHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, pcl::ModelCoefficients::Ptr& plane_coefficients, visualization_msgs::Marker& marker);

		private:
			/**
			 * Returns center and scale of plane marker
			 * @param plane Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param center Sendor to map transformation matrix
			 * @param scale Sendor to map transformation matrix
			 */
			bool getCenterAndScale(but_plane_detector::Plane<float> &plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud, pcl::PointXYZ &center, pcl::PointXYZ &scale);

			bool getCenterAndScale(but_plane_detector::Plane<float> &plane, but_plane_detector::Normals &normals, pcl::PointXYZ &center, pcl::PointXYZ &scale);
			void getCenterSAndScale(tPlanes & planes, but_plane_detector::Normals &normals, std::vector<cv::Vec3f> &centers, std::vector<cv::Vec3f> &scales, std::vector<bool> &flags);

			/**
			 * Auxiliary node handle variable
			 */
			ros::NodeHandle *n;
			
			/**
			 * Auxiliary index vector for managing modifications
			 */
			std::vector<bool> managedInd;

			std::string original_frame_, output_frame_;

			int m_minOutputCount;
			double m_max_distance;
			double m_max_plane_normal_dev;
			double m_max_plane_shift_dev;
	};
}

#endif
