/**
 * $Id: dynModelExporter.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *	 Encapsulates a class of plane exporter (export to but_gui module - interactive markers)
 */

#ifndef DYNMODELEXPORTER_H
#define DYNMODELEXPORTER_H

#include <ros/ros.h>
#include "normals.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
		void update(std::vector<Plane<float> > & planes, PointCloud<PointXYZRGB>::Ptr scene_cloud);

	private:
		/**
		 * Returns center and scale of plane marker
		 */
		bool getCenterAndScale(Plane<float> &plane, PointCloud<PointXYZRGB>::Ptr scene_cloud, PointXYZ &center, PointXYZ &scale);

		ros::NodeHandle *n;
		std::vector<bool> managedInd;
};
}

#endif
