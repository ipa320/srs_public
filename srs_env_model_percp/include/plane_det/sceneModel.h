/******************************************************************************
 * \file
 *
 * $Id: sceneModel.h 619 2012-04-16 13:47:28Z ihulik $
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
 *	 Class encapsulating A scene model (i.e. found planes)
 */

#ifndef SCENEMODEL_H_
#define SCENEMODEL_H_

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

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// srs_but
#include "normals.h"
#include "plane_det/parameterSpace.h"
#include "plane_det/parameterSpaceHierarchy.h"

using namespace pcl;

namespace but_scenemodel
{
	class SceneModel
	{
		public:
		/**
		 * Constructor - initializes a scene and allocates necessary space - a space of (angle, angle, d) where angles are angles of plane normal and d is d parameter of plane equation.
		 * @param max_depth Maximum computed depth by each frame in meters (default 3.0)
		 * @param min_shift Minimal d parameter value (ax + by + cz + d = 0) in Hough space (default -40.0)
		 * @param max_shift Maximal d parameter value (ax + by + cz + d = 0) in Hough space (default 40.0)
		 * @param angle_resolution Angle coordinates resolution (Hough space size in angle directions) (default 512)
		 * @param shift_resolution d parameter coordinates resolution (Hough space size in angle directions) (default 4096)
		 * @param gauss_angle_res Angle resolution of added Gauss function (default 11)
		 * @param gauss_shift_res d parameter resolution of added Gauss function (default 11)
		 * @param gauss_angle_sigma Sigma of added Gauss function in angle coordinates (default 11)
		 * @param gauss_shift_sigma Sigma of added Gauss function in d parameter coordinates (default 11)
		 */
		SceneModel(	double max_depth = 3.0,
					double min_shift = -40.0,
					double max_shift = 40.0,
					int angle_resolution = 512,
					int shift_resolution = 4096,
					int gauss_angle_res = 11,
					int gauss_shift_res = 11,
					double gauss_angle_sigma = 0.04,
					double gauss_shift_sigma = 0.15);

		/**
		 * Function adds a depth map with computed normals into existing Hough space
		 * @param depth Depth image
		 * @param cam_info Camera info object
		 * @param normals Computed normals of depth image
		 */
		void AddNext(cv::Mat &depth, const CameraInfoConstPtr& cam_info, Normals &normals);

		/**
		 * Function recomputes a list of planes saved in this class (scene model)
		 */
		void recomputePlanes();

		/**
		 * Point cloud representation of Hough space for visualisation purposes
		 */
		pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud;

		/**
		 * Detected planes vector
		 */
		std::vector<Plane<float> > planes;

	protected:
		/**
		 * Hough space representation
		 */
		ParameterSpaceHierarchy space;

		/**
		 * Cached Hough space aux
		 */
		ParameterSpaceHierarchy cache_space;

		/**
		 * Discretized Gauss function
		 */
		ParameterSpace gauss;

		/**
		 * Maximum plane index
		 */
		int max_plane;

		/**
		 *
		 */
		double indexFactor;

		/**
		 *
		 */
		double m_depth;
	};
}

#endif /* SCENEMODEL_H_ */
