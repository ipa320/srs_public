/******************************************************************************
 * \file
 *
 * $Id: dynModelExporter.h 397 2012-03-29 12:50:30Z spanel $
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
 *	 Encapsulates a class of plane exporter (export to but_gui module - interactive markers)
 */

#ifndef DYNMODELEXPORTER_H
#define DYNMODELEXPORTER_H

#include <ros/ros.h>
#include "normals.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>

using namespace pcl;

namespace but_scenemodel
{
class DynModelExporter
{
	public:
		/**
		 * Initialization
		 */
		DynModelExporter(ros::NodeHandle *node);

		/**
		 * Updates sent planes using rosservice
		 */
		void update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf);
		void update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud);

	private:
		/**
		 * Returns center and scale of plane marker
		 */
		bool getCenterAndScale(Plane<float> &plane, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, PointXYZ &center, PointXYZ &scale);

		ros::NodeHandle *n;
		std::vector<bool> managedInd;
};
}

#endif
