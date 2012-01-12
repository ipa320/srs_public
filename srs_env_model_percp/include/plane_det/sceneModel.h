/**
 * $Id: sceneModel.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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

		PointCloud<PointXYZRGB>::Ptr scene_cloud;
		PointCloud<PointXYZI>::Ptr current_hough_cloud;
		std::vector<Plane<float> > planes;
		int max_plane;
	};
}

#endif /* SCENEMODEL_H_ */
