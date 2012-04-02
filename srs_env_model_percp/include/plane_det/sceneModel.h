/******************************************************************************
 * \file
 *
 * $Id: sceneModel.h 397 2012-03-29 12:50:30Z spanel $
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
//better opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "normals.h"

using namespace pcl;

namespace but_scenemodel
{
	class SceneModel
	{
		public:
		/**
		 * Constructor - creates a HT from depth map and finds planes
		 */
		SceneModel(cv::Mat &depth, const CameraInfoConstPtr& cam_info, Normals &normals);

		pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud;
		pcl::PointCloud<PointXYZI>::Ptr current_hough_cloud;
		std::vector<Plane<float> > planes;
		int max_plane;
	};
}

#endif /* SCENEMODEL_H_ */
