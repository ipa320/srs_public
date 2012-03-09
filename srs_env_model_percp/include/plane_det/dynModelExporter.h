/**
 * $Id: dynModelExporter.h 263 2012-02-27 13:49:53Z ihulik $
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
