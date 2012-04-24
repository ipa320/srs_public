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

#ifndef DYNMODELEXPORTER_H
#define DYNMODELEXPORTER_H

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// but_scenemodel
#include "plane_det/normals.h"

using namespace pcl;

namespace but_scenemodel
{
	/**
	 * Encapsulates a class of plane exporter (export to but_gui module/interactive markers)
	 */
	class DynModelExporter
	{
		public:
			/**
			 * Initialization
			 */
			DynModelExporter(ros::NodeHandle *node);

			/**
			 * Updates sent planes using but environment model server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param sensorToWorldTf Sendor to map transformation matrix
			 */
			void update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf);
			
			/**
			 * Updates sent planes using but environment model server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 */
			void update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud);

			/**
			 * Updates sent planes using direct but interactive marker server
			 * @param planes Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param sensorToWorldTf Sendor to map transformation matrix
			 */
			void updateDirect(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf);

		private:
			/**
			 * Returns center and scale of plane marker
			 * @param plane Vector of found planes
			 * @param scene_cloud point cloud of the scene
			 * @param center Sendor to map transformation matrix
			 * @param scale Sendor to map transformation matrix
			 */
			bool getCenterAndScale(Plane<float> &plane, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, PointXYZ &center, PointXYZ &scale);

			/**
			 * Auxiliary node handle variable
			 */
			ros::NodeHandle *n;
			
			/**
			 * Auxiliary index vector for managing modifications
			 */
			std::vector<bool> managedInd;
	};
} // but_scenemodel

#endif
