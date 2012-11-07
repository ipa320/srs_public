/******************************************************************************
 * \file
 *
 * $Id: DynModelExporter2.cpp 814 2012-05-22 14:00:19Z ihulik $
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
 *	 Encapsulates a class of plane exporter (export to but_gui module/interactive markers)
 */

#include <srs_env_model_percp/but_plane_detector/dyn_model_exporter2.h>
#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>


#include <srs_interaction_primitives/AddPlane.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_interaction_primitives/plane.h>
#include <pcl/filters/project_inliers.h>

#include <srs_env_model/InsertPlanes.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/transforms.h>



using namespace pcl;
using namespace but_plane_detector;
using namespace cv;

namespace srs_env_model_percp
{

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Updates sent planes using but environment model server
	// @param planes Vector of found planes
	// @param scene_cloud point cloud of the scene
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DynModelExporter2::update(tPlanes & planes, Normals &normals, std::string color_method, cv::Mat rgb)
	{
		if (m_keep_tracking == 0)
			displayed_planes.clear();
		std::vector<PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<PointCloud<pcl::PointXYZ> > > planesInPCloud(planes.size());
		std::vector<std_msgs::ColorRGBA, Eigen::aligned_allocator<std_msgs::ColorRGBA> > colors(planes.size());

		for (int i = 0; i < normals.m_points.rows; ++i)
		for (int j = 0; j < normals.m_points.cols; ++j)
		{
			Vec3f point = normals.m_points.at<Vec3f>(i, j);
			if (point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0)
			{
				cv::Vec4f localPlane = normals.m_planes.at<cv::Vec4f>(i, j);
				Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);

				double dist = DBL_MAX;

				int chosen = -1;
				// find the best plane
				for (unsigned int a = 0; a < planes.size(); ++a)
				{
					if (planes[a].distance(point) < dist && planes[a].distance(point) < m_max_distance &&
						planes[a].isSimilar(aaa, m_max_plane_normal_dev, m_max_plane_shift_dev))
					{
						dist = planes[a].distance(point);
						chosen = a;
					}
				}

				// if there is good plane, insert point into point cloud
				if (chosen > -1)
				{
					PointXYZ pclpoint(point[0], point[1], point[2]);
					planesInPCloud[chosen].push_back(pclpoint);

					if (color_method == "mean_color")
					{
						cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
						colors[chosen].r += (float)color[0]/255.0;
						colors[chosen].g += (float)color[1]/255.0;
						colors[chosen].b += (float)color[2]/255.0;
						//std::cerr << color[0] << " " << color[1] << " " << color[2] << std::endl;
					}
					else if (color_method == "mean_color")
					{
						cv::Vec<unsigned char, 3> color = rgb.at<cv::Vec<unsigned char, 3> >(i, j);
						colors[chosen].r += point[0] / 5.0;
						colors[chosen].g += point[1] / 5.0;
						colors[chosen].b += point[2] / 5.0;
											//std::cerr << color[0] << " " << color[1] << " " << color[2] << std::endl;
					}
//					if (point[2] > 0.9 && point[2] < 1.0 && (planes[chosen].d < -0.5 || planes[chosen].d > 0.5) && (planes[chosen].c < -0.5 || planes[chosen].c > 0.5))
//					{
//						std::cerr << planes[chosen].a << " " << planes[chosen].b << " " << planes[chosen].c << " " << planes[chosen].d << " --- > ";
//						std::cerr << point[0] << " " << point[1] << " " << point[2] << std::endl;
//					}
				}
			}
		}

		if (color_method == "mean_color" || color_method == "centroid")
		{
			for (unsigned int i = 0; i < colors.size(); ++i)
				if(planesInPCloud.size() > 0)
				{
					colors[i].r /= planesInPCloud[i].size();
					colors[i].g /= planesInPCloud[i].size();
					colors[i].b /= planesInPCloud[i].size();
					colors[i].a = 1.0;
				}
		}
		// Indexed in point cloud
		////////////////////////////////////////////////////////////////////////////////////////////////

		for (unsigned int j = 0; j < planesInPCloud.size(); ++j)
		{
			if (planesInPCloud[j].size() > 20)
			{
				double maxangle = DBL_MAX;
				double maxdist = DBL_MAX;
				int index = -1;
				for (unsigned int i = 0; i < displayed_planes.size(); ++i)
				{
					double angle = acos(((planes[j].a * displayed_planes[i].plane.a) + (planes[j].b * displayed_planes[i].plane.b) + (planes[j].c * displayed_planes[i].plane.c)));
					double xd = planes[j].d - displayed_planes[i].plane.d;
					xd = (xd > 0 ? xd : - xd);

					// Pretty nasty workaround... todo
					if (angle != angle) angle = 0.0;

					if (angle <= maxangle  && xd <= maxdist && angle < m_max_plane_normal_dev  && xd < m_max_plane_shift_dev)
					{
						maxangle = angle;
						maxdist = xd;
						index = i;
					}
				}

				if (index >= 0)
				{
					DynModelExporter2::addMarkerToConcaveHull(planesInPCloud[j], displayed_planes[index].plane);
				}
				else
				{
					ExportedPlane newplane;
					newplane.plane = PlaneExt(planes[j]);
					DynModelExporter2::createMarkerForConcaveHull(planesInPCloud[j], newplane.plane);

					newplane.id = displayed_planes.size();
					newplane.plane.getMeshMarker().id = newplane.id;
					if (color_method == "mean_color")
					{
						std::cerr << "setting color: " << colors[j].r << " " << colors[j].g << " " << colors[j].b << std::endl;
						newplane.plane.setColor(colors[j]);
					}
					else if (color_method == "random")
					{
						colors[j].r = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].g = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].b = (float)rand()/INT_MAX * 0.5 + 0.2;
						colors[j].a = 1.0;
						newplane.plane.setColor(colors[j]);
					}
					//newplane.plane.getShapeMarker().id = newplane.id;
					displayed_planes.push_back(newplane);
				}
			}
		}
	}




	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DynModelExporter2::DynModelExporter2(ros::NodeHandle *node,
	                                   const std::string& original_frame,
	                                   const std::string& output_frame,
	                                   int minOutputCount,
	                                   double max_distance,
	                                   double max_plane_normal_dev,
	                                   double max_plane_shift_dev,
	                                   int keep_tracking
	                                   )
	    : original_frame_(original_frame)
	    , output_frame_(output_frame)
	{
		n = node;
		m_minOutputCount = minOutputCount;
		m_max_distance = max_distance;
		m_max_plane_normal_dev = max_plane_normal_dev;
		m_max_plane_shift_dev = max_plane_shift_dev;
		m_keep_tracking = keep_tracking;
	}

	void DynModelExporter2::createMarkerForConcaveHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, srs_env_model_percp::PlaneExt& plane)
	{
		plane.NewPlanePoints(plane_cloud.makeShared());
	    plane.getMeshMarker().type = visualization_msgs::Marker::TRIANGLE_LIST;
	    plane.getMeshMarker().action = visualization_msgs::Marker::ADD;

	    plane.getMeshMarker().ns = "Normals";
	    plane.getMeshMarker().pose.position.x = 0.0;
	    plane.getMeshMarker().pose.position.y = 0.0;
	    plane.getMeshMarker().pose.position.z = 0.0;
	    plane.getMeshMarker().pose.orientation.x = 0.0;
	    plane.getMeshMarker().pose.orientation.y = 0.0;
	    plane.getMeshMarker().pose.orientation.z = 0.0;
	    plane.getMeshMarker().pose.orientation.w = 1.0;
	    plane.getMeshMarker().scale.x = 1.00;
	    plane.getMeshMarker().scale.y = 1.00;
	    plane.getMeshMarker().scale.z = 1.00;
//

	}

	void DynModelExporter2::addMarkerToConcaveHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, srs_env_model_percp::PlaneExt& plane)
	{
		plane.AddPlanePoints(plane_cloud.makeShared());
	}

}// but_scenemodel
