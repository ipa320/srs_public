/******************************************************************************
 * \file
 *
 * $Id: plane.h 693 2012-10-20 09:22:39Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 15.06.2012 (version 1.0)
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

#ifndef PLANE_EXT_H
#define PLANE_EXT_H

#include <srs_env_model_percp/but_segmentation/normals.h>
#include <srs_env_model_percp/but_plane_detector/clipper.hpp>
#include <srs_env_model_percp/but_plane_detector/polypartition.h>
#include <visualization_msgs/Marker.h>
#include <cob_3d_mapping_msgs/Shape.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/transforms.h>

// conversion between any type to ClipperLib long long type and back
#define CONVERT_TO_LONG(a) a * 1000000
#define CONVERT_FROM_LONG(a) (float)a / 1000000

namespace srs_env_model_percp
{
	class PlaneExt : public but_plane_detector::Plane<float>
	{
	public:
		typedef std::vector<cob_3d_mapping_msgs::Shape, Eigen::aligned_allocator<cob_3d_mapping_msgs::Shape> > tShapeMarker;

	//	typedef std::vector<pcl::Vertices, Eigen::aligned_allocator<pcl::Vertices> > tVertices;
		typedef std::vector<pcl::Vertices> tVertices;

		public:
			/**
			 * Creates an instance of this type... Must be on the basis of existing plane
			 */
			PlaneExt(but_plane_detector::Plane<float> plane);

			/**
			 * Make a new marker and polygonized hull
			 */
			visualization_msgs::Marker NewPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);

			/**
			 * Update marker and polygonized hull
			 */
			visualization_msgs::Marker AddPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);

			/**
			 * Get polygon
			 */
			ClipperLib::ExPolygons 	   &getPolygons();

			void setPolygons(ClipperLib::ExPolygons &polys);

			/**
			 * Get Mesh structure
			 */
			std::list<TPPLPoly>   	   &getMesh();

			/**
			 * Get MarkerArray message
			 */
			visualization_msgs::Marker &getMeshMarker();

			/**
			 * Get Shape messages
			 */
			tShapeMarker &getShapeMarker();

			void setColor(std_msgs::ColorRGBA &new_color);

			// Triangulates plane polygon
			void TriangulatePlanePolygon();

			std_msgs::ColorRGBA color;
		protected:
			// Computes concave hull of set of points
			tVertices ComputeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull);

			// Computes hull U current polygon
			bool ConcaveHullJoinCurrent(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices);

			// Rewrites current plane with this hull
			void ConcaveHullRewrite(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices);

			// Polygonizes current hull
			ClipperLib::ExPolygons PolygonizeConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_hull, tVertices &polygon_indices);

			Eigen::Affine3f				planeTransXY;
			double planeShift;
			pcl::ModelCoefficients::Ptr planeCoefficients;

			ClipperLib::ExPolygons  	planePolygonsClipper;
			visualization_msgs::Marker 	planeTriangles;
			tShapeMarker  planeTrianglesSRS;
			Eigen::Quaternion<float> rotationQuaternion;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#endif
