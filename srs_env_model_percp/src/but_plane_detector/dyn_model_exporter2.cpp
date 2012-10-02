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
#include <pcl/surface/convex_hull.h>
#include <srs_env_model/InsertPlanes.h>
#include <pcl/point_cloud.h>

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
	void DynModelExporter2::update(std::vector<Plane<float> > & planes, Normals &normals)
	{
		if (m_keep_tracking == 0)
			displayed_planes.clear();
		std::vector<PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<PointCloud<pcl::PointXYZ> > > planesInPCloud(planes.size());

		for (int i = 0; i < normals.m_points.rows; ++i)
		for (int j = 0; j < normals.m_points.cols; ++j)
		{
			Vec3f point = normals.m_points.at<Vec3f>(i, j);
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

					if (angle <= maxangle  && xd <= maxdist && angle < 0.2  && xd < 0.2)
					{
						maxangle = angle;
						maxdist = xd;
						index = i;
					}
				}

				if (index >= 0)
				{
					for (unsigned int i = 0; i < displayed_planes[index].marker.points.size(); ++i)
					{
						geometry_msgs::Point p = displayed_planes[index].marker.points[i];
						planesInPCloud[j].push_back(PointXYZ(p.x, p.y, p.z));
					}

					pcl::ModelCoefficientsPtr coefs(new pcl::ModelCoefficients());
					coefs->values.push_back(displayed_planes[index].plane.a);
					coefs->values.push_back(displayed_planes[index].plane.b);
					coefs->values.push_back(displayed_planes[index].plane.c);
					coefs->values.push_back(displayed_planes[index].plane.d);

					DynModelExporter2::createMarkerForConvexHull(planesInPCloud[j], coefs, displayed_planes[index].marker);
				}
				else
				{
					pcl::ModelCoefficientsPtr coefs(new pcl::ModelCoefficients());

					coefs->values.push_back(planes[j].a);
					coefs->values.push_back(planes[j].b);
					coefs->values.push_back(planes[j].c);
					coefs->values.push_back(planes[j].d);


//					std::cerr << planes[j].a << " ";
//					std::cerr << planes[j].b << " ";
//					std::cerr << planes[j].c << " ";
//					std::cerr << planes[j].d << std::endl;

					visualization_msgs::Marker marker;

					DynModelExporter2::createMarkerForConvexHull(planesInPCloud[j], coefs, marker);
					marker.id = displayed_planes.size()+1;
					marker.ns = "Normals";
					marker.pose.position.x = 0.0;
					marker.pose.position.y = 0.0;
					marker.pose.position.z = 0.0;
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 1;
					marker.color.r = 0.5;
					marker.color.g = 0.0;
					marker.color.b = 0.0;
					marker.color.a = 0.8;

					ExportedPlane newplane;
					newplane.id = index;
					newplane.marker = marker;
					newplane.plane = planes[j];
					displayed_planes.push_back(newplane);
				}
			}
		}
		std::cerr << displayed_planes.size() << std::endl;
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

	void DynModelExporter2::createMarkerForConvexHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, pcl::ModelCoefficients::Ptr& plane_coefficients, visualization_msgs::Marker& marker)
	{
		// init marker
	    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	    marker.action = visualization_msgs::Marker::ADD;

	    // project the points of the plane on the plane
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::ProjectInliers<pcl::PointXYZ> proj;
	    proj.setModelType (pcl::SACMODEL_PLANE);
	    proj.setInputCloud (plane_cloud.makeShared());
	    proj.setModelCoefficients (plane_coefficients);
	    proj.filter(*cloud_projected);

	    // create the convex hull in the plane
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::ConvexHull<pcl::PointXYZ > chull;
	    chull.setInputCloud (cloud_projected);
	    chull.reconstruct(*cloud_hull);

	    // work around known bug in ROS Diamondback perception_pcl: convex hull is centered around centroid of input cloud (fixed in pcl svn revision 443)
	    // thus: we shift the mean of cloud_hull to the mean of cloud_projected (fill dx, dy, dz and apply when creating the marker points)
	    Eigen::Vector4f meanPointCH, meanPointCP;
	    pcl::compute3DCentroid(*cloud_projected, meanPointCP);
	    pcl::compute3DCentroid(*cloud_hull, meanPointCH);
	    //float dx = 0;//meanPointCP[0]-meanPointCH[0];
	    //float dy = 0;//meanPointCP[1]-meanPointCH[1];
	    //float dz = 0;//meanPointCP[2]-meanPointCH[2];

	    // create colored part of plane by creating marker for each triangle between neighbored points on contour of convex hull an midpoint
	    marker.points.clear();
	    for (unsigned int j = 0; j < cloud_hull->points.size(); ++j)
	    {
	    	geometry_msgs::Point p;

	        p.x = cloud_hull->points[j].x; p.y = cloud_hull->points[j].y; p.z = cloud_hull->points[j].z;
	        marker.points.push_back( p );

	        p.x = cloud_hull->points[(j+1)%cloud_hull->points.size() ].x; p.y = cloud_hull->points[(j+1)%cloud_hull->points.size()].y; p.z = cloud_hull->points[(j+1)%cloud_hull->points.size()].z;
	        marker.points.push_back( p );

	        p.x = meanPointCP[0]; p.y = meanPointCP[1]; p.z = meanPointCP[2];
	        marker.points.push_back( p );

	    }

	// scale of the marker
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;

	}

}// but_scenemodel
