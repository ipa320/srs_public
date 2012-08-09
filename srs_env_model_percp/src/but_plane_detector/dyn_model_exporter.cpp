/******************************************************************************
 * \file
 *
 * $Id: dynModelExporter.cpp 814 2012-05-22 14:00:19Z ihulik $
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

#include <srs_env_model_percp/but_plane_detector/dyn_model_exporter.h>
#include <srs_env_model_percp/topics_list.h>
#include <srs_env_model_percp/services_list.h>

#include <srs_interaction_primitives/AddPlane.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_interaction_primitives/plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <srs_env_model/InsertPlanes.h>

using namespace pcl;

namespace srs_env_model_percp
{

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Updates sent planes using direct but interactive marker server
	// @param planes Vector of found planes
	// @param scene_cloud point cloud of the scene
	// @param sensorToWorldTf Sendor to map transformation matrix
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DynModelExporter::updateDirect(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf)
	{
		// but dynamic model
		ros::ServiceClient plane = n->serviceClient<srs_interaction_primitives::AddPlane> ("insert_plane2");

		// For each plane, call a server...
		for (unsigned int i = 0; i < planes.size(); ++i)
		{
			float x, y, z, w;
			planes[i].getQuaternionRotation(x, y, z, w);

			PointXYZ center;
			PointXYZ scale;

			if (getCenterAndScale(planes[i], scene_cloud, center, scale))
			{
				// Fill in coords
				tf::Transformer t;
				t.setTransform(sensorToWorldTf);

				tf::Stamped<btVector3> pose, pose2;
				tf::Stamped<btQuaternion> orientation, orientation2;
				pose.setX(center.x);
				pose.setY(center.y);
				pose.setZ(center.z);
				pose.frame_id_ = original_frame_;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = original_frame_;
				t.transformPoint(output_frame_, pose, pose2);
				t.transformQuaternion(output_frame_, orientation, orientation2);


				std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
				srs_interaction_primitives::AddPlane planeSrv;
				planeSrv.request.frame_id = output_frame_;
				planeSrv.request.pose.position.x = pose2.getX();
				planeSrv.request.pose.position.y = pose2.getY();
				planeSrv.request.pose.position.z = pose2.getZ();
				planeSrv.request.pose.orientation.x = orientation2.getX();
				planeSrv.request.pose.orientation.y = orientation2.getY();
				planeSrv.request.pose.orientation.z = orientation2.getZ();
				planeSrv.request.pose.orientation.w = orientation2.getW();
				planeSrv.request.scale.x = scale.x;
				planeSrv.request.scale.y = scale.y;
				planeSrv.request.scale.z = scale.z;

				// push into array
				plane.call(planeSrv);
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Updates sent planes using but environment model server
	// @param planes Vector of found planes
	// @param scene_cloud point cloud of the scene
	// @param sensorToWorldTf Sendor to map transformation matrix
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DynModelExporter::update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf)
	{
		// but dynamic model
		ros::ServiceClient plane = n->serviceClient<srs_env_model::InsertPlanes> (DET_SERVICE_INSERT_PLANES);

		// Create calls
		srs_env_model::InsertPlanes planeSrv;
		srs_env_model_msgs::PlaneArray planeArray;

		// for each plane
		for (unsigned int i = 0; i < planes.size(); ++i)
		{
			float x, y, z, w;
			planes[i].getQuaternionRotation(x, y, z, w);

			PointXYZ center;
			PointXYZ scale;

			if (getCenterAndScale(planes[i], scene_cloud, center, scale))
			{
				srs_env_model_msgs::PlaneDesc planeDyn;

				// test if we modify or insert
				if (managedInd.size() > i)
					planeDyn.flags = srs_env_model_msgs::PlaneDesc::MODIFY;
				else
				{
					planeDyn.flags = srs_env_model_msgs::PlaneDesc::INSERT;
					managedInd.push_back(true);
				}

				// Fill in coords
				tf::Transformer t;
				t.setTransform(sensorToWorldTf);
				// point transform
				// normal inverse transform


				tf::Stamped<btVector3> pose, pose2;
				tf::Stamped<btQuaternion> orientation, orientation2;
				pose.setX(center.x);
				pose.setY(center.y);
				pose.setZ(center.z);
				pose.frame_id_ = original_frame_;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = original_frame_;
				t.transformPoint(output_frame_, pose, pose2);
				t.transformQuaternion(output_frame_, orientation, orientation2);

				std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
				std::cout << "position: " << pose2.getX() << " " << pose2.getY() << " " << pose2.getZ() << std::endl;
				planeDyn.id = i;
				planeDyn.pose.position.x = pose2.getX();
				planeDyn.pose.position.y = pose2.getY();
				planeDyn.pose.position.z = pose2.getZ();
				planeDyn.pose.orientation.x = orientation2.getX();
				planeDyn.pose.orientation.y = orientation2.getY();
				planeDyn.pose.orientation.z = orientation2.getZ();
				planeDyn.pose.orientation.w = orientation2.getW();
				planeDyn.scale.x = scale.x;
				planeDyn.scale.y = scale.y;
				planeDyn.scale.z = scale.z;

				// push into array
				planeSrv.request.plane_array.planes.push_back(planeDyn);
			}
		}
		// fill in header and send
		planeSrv.request.plane_array.header.frame_id = output_frame_;
		planeSrv.request.plane_array.header.stamp = ros::Time::now();
		plane.call(planeSrv);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Updates sent planes using but environment model server
	// @param planes Vector of found planes
	// @param scene_cloud point cloud of the scene
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DynModelExporter::update(std::vector<Plane<float> > & planes, Normals &normals, tf::StampedTransform &sensorToWorldTf)
	{
		// but dynamic model
		ros::ServiceClient plane = n->serviceClient<srs_env_model::InsertPlanes> (DET_SERVICE_INSERT_PLANES);

		// Create calls
		srs_env_model::InsertPlanes planeSrv;
		srs_env_model_msgs::PlaneArray planeArray;

		std::vector<cv::Vec3f> centers;
		std::vector<cv::Vec3f> scales;
		std::vector<bool> flags;
		getCenterSAndScale(planes, normals, centers, scales, flags);
		// for each plane
		for (unsigned int i = 0; i < planes.size(); ++i)
		{
			float x, y, z, w;


//			tf::Stamped<btVector3> planeorignormal, planeorignormal2;
//			planeorignormal.setX(0);
//			planeorignormal.setY(0);
//			planeorignormal.setZ(0);
//			planeorignormal.frame_id_ = DET_OUTPUT_PLANE_ORIGINAL_FRAMEID;
//			tf::Transformer t;
//			t.setTransform(sensorToWorldTf);
//			t.transformVector(DET_OUTPUT_PLANE_FRAMEID, planeorignormal, planeorignormal2);

			cv::Vec3f rotate(0, 0, 1);
			//Plane<float> test(1, 0, 0, 5);
			//test.getQuaternionRotation(x, y, z, w, rotate);
			planes[i].getQuaternionRotation(x, y, z, w, rotate);

			PointXYZ center;
			PointXYZ scale;

			if (flags[i])
			{
				srs_env_model_msgs::PlaneDesc planeDyn;

				// test if we modify or insert
				if (managedInd.size() > i)
					planeDyn.flags = srs_env_model_msgs::PlaneDesc::MODIFY;
				else
				{
					planeDyn.flags = srs_env_model_msgs::PlaneDesc::INSERT;
					managedInd.push_back(true);
				}

				// Fill in coords
				tf::Stamped<btVector3> pose;
				tf::Stamped<btQuaternion> orientation;
				pose.setX(centers[i][0]);
				pose.setY(centers[i][1]);

				pose.setZ(centers[i][2]);
				pose.frame_id_ = output_frame_;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = output_frame_;


				std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
				planeDyn.id = i;
				planeDyn.pose.position.x = pose.getX();
				planeDyn.pose.position.y = pose.getY();
				planeDyn.pose.position.z = pose.getZ();
				planeDyn.pose.orientation.x = orientation.getX();
				planeDyn.pose.orientation.y = orientation.getY();
				planeDyn.pose.orientation.z = orientation.getZ();
				planeDyn.pose.orientation.w = orientation.getW();
				planeDyn.scale.x = scales[i][0];
				planeDyn.scale.y = scales[i][1];
				planeDyn.scale.z = scales[i][2];

				// push into array
				planeSrv.request.plane_array.planes.push_back(planeDyn);
			}
		}
		// fill in header and send
		planeSrv.request.plane_array.header.frame_id = output_frame_;
		planeSrv.request.plane_array.header.stamp = ros::Time::now();
		plane.call(planeSrv);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Returns center and scale of plane marker
	// @param plane Vector of found planes
	// @param scene_cloud point cloud of the scene
	// @param center Sendor to map transformation matrix
	// @param scale Sendor to map transformation matrix
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool DynModelExporter::getCenterAndScale(Plane<float> &plane, Normals &normals, PointXYZ &center, PointXYZ &scale)
	{
		center.x = 0.0;
		center.y = 0.0;
		center.z = 0.0;

		float size = 0.0;
		PointXYZ min(9999999.0, 9999999.0, 9999999.0);
		PointXYZ max(-9999999.0, -9999999.0, -9999999.0);

		for (unsigned int i = 0; i < normals.m_points.rows; ++i)
		for (unsigned int j = 0; j < normals.m_points.cols; ++j)
		{
			 // 1cm TODO - make as param
			cv::Vec3f point = normals.m_points.at<cv::Vec3f>(i, j);
			cv::Vec4f localPlane = normals.m_planes.at<cv::Vec4f>(i, j);
			Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
			 if (plane.distance(point) < m_max_distance && plane.isSimilar(aaa, m_max_plane_normal_dev, m_max_plane_shift_dev))
			 {
				 center.x += point[0];
				 center.y += point[1];
				 center.z += point[2];
				 size += 1.0;

				 if (point[0] < min.x) min.x = point[0];
				 if (point[1] < min.y) min.y = point[1];
				 if (point[2] < min.z) min.z = point[2];

				 if (point[0] > max.x) max.x = point[0];
				 if (point[1] > max.y) max.y = point[1];
				 if (point[2] > max.z) max.z = point[2];
			 }
		}

		//std::cout << std::endl << std::endl << min << " " << max << std::endl << std::endl;
		if (size > m_minOutputCount)
		{
			center.x /= size;
			center.y /= size;
			center.z /= size;
			//center.z = -(center.x*plane.a + center.y*plane.b + plane.d)/plane.c;
		}
		else return false;

		scale.x = max.x - min.x;
		if (scale.x > 3)
			scale.x = 3;
		scale.y = max.y - min.y;
		if (scale.y > 3)
			scale.y = 3;
		scale.z = max.z - min.z;
		if (scale.z > 3)
			scale.z = 3;

		std::cout << size << std::endl;
		return true;
	}

	void DynModelExporter::getCenterSAndScale(std::vector<Plane<float> > & planes, Normals &normals, std::vector<cv::Vec3f> &centers, std::vector<cv::Vec3f> &scales, std::vector<bool> &flags)
	{
		using namespace cv;
		centers.resize(planes.size(), Vec3f(0.0, 0.0, 0.0));
		scales.resize(planes.size(), Vec3f(0.0, 0.0, 0.0));
		flags.resize(planes.size());
		std::vector<int> sizes(planes.size(), 0);
		std::vector<Vec3f> mins(planes.size(), Vec3f(FLT_MAX, FLT_MAX, FLT_MAX));
		std::vector<Vec3f> maxs(planes.size(), Vec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));

		for (unsigned int i = 0; i < normals.m_points.rows; ++i)
		for (unsigned int j = 0; j < normals.m_points.cols; ++j)
		{
			Vec3f point = normals.m_points.at<Vec3f>(i, j);
			cv::Vec4f localPlane = normals.m_planes.at<cv::Vec4f>(i, j);
			Plane<float> aaa(localPlane[0], localPlane[1], localPlane[2], localPlane[3]);
			double dist = DBL_MAX;
			int chosen = -1;
			for (unsigned int a = 0; a < planes.size(); ++a)
			{
				if (point[0] == point[0] && point[1] == point[1] && point[2] == point[2] && planes[a].distance(point) < dist && planes[a].distance(point) < m_max_distance && planes[a].isSimilar(aaa, m_max_plane_normal_dev, m_max_plane_shift_dev))
				{

					dist = planes[a].distance(point);
					chosen = a;
				}
			}
			if (chosen > -1)
			{
				sizes[chosen] += 1;
				centers[chosen] += point;

				if (point[0] < mins[chosen][0]) mins[chosen][0] = point[0];
				if (point[1] < mins[chosen][1]) mins[chosen][1] = point[1];
				if (point[2] < mins[chosen][2]) mins[chosen][2] = point[2];

				if (point[0] > maxs[chosen][0]) maxs[chosen][0] = point[0];
				if (point[1] > maxs[chosen][1]) maxs[chosen][1] = point[1];
				if (point[2] > maxs[chosen][2]) maxs[chosen][2] = point[2];
			}
		}

		for (unsigned int i = 0; i < planes.size(); ++i)
		{
			if (sizes[i] < m_minOutputCount) flags[i] = false;
			else flags[i] = true;

			centers[i][0] /= sizes[i];
			centers[i][1] /= sizes[i];
			centers[i][2] /= sizes[i];

			tf::Transform planeInverseTransform;
			planeInverseTransform.setOrigin(btVector3(0,0,0));
			float x, y, z, w;
			planes[i].getInverseQuaternionRotation(x, y, z, w);
			planeInverseTransform.setRotation(btQuaternion(x, y, z, w));
			btVector3 max(maxs[i][0], maxs[i][1], maxs[i][2]);
			btVector3 min(mins[i][0], mins[i][1], mins[i][2]);

			max = planeInverseTransform * max;
			min = planeInverseTransform * min;

			scales[i][0] = max.x() - min.x();
			scales[i][1] = max.y() - min.y();
			scales[i][2] = max.z() - min.z();
		}
	}
	bool DynModelExporter::getCenterAndScale(Plane<float> &plane, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, PointXYZ &center, PointXYZ &scale)
		{
			center.x = 0.0;
			center.y = 0.0;
			center.z = 0.0;

			float size = 0.0;
			PointXYZ min(9999999.0, 9999999.0, 9999999.0);
			PointXYZ max(-9999999.0, -9999999.0, -9999999.0);

			for (pcl::PointCloud<PointXYZRGB>::iterator it = scene_cloud->begin(); it != scene_cloud->end(); ++it)
			{
				 // 1cm TODO - make as param
				 if (plane.distance(cv::Vec3f(it->x, it->y, it->z)) < m_max_distance)
				 {
					 center.x += it->x;
					 center.y += it->y;
					 center.z += it->z;
					 size += 1.0;

					 if (it->x < min.x) min.x = it->x;
					 if (it->y < min.y) min.y = it->y;
					 if (it->z < min.z) min.z = it->z;

					 if (it->x > max.x) max.x = it->x;
					 if (it->y > max.y) max.y = it->y;
					 if (it->z > max.z) max.z = it->z;
				 }
			}

			//std::cout << std::endl << std::endl << min << " " << max << std::endl << std::endl;
			if (size > 10)
			{
				center.x /= size;
				center.y /= size;
				center.z /= size;
				//center.z = -(center.x*plane.a + center.y*plane.b + plane.d)/plane.c;
			}
			else return false;

			scale.x = max.x - min.x;
			if (scale.x > 3)
				scale.x = 3;
			scale.y = max.y - min.y;
			if (scale.y > 3)
				scale.y = 3;
			scale.z = max.z - min.z;
			if (scale.z > 3)
				scale.z = 3;

			return true;
		}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DynModelExporter::DynModelExporter(ros::NodeHandle *node,
	                                   const std::string& original_frame,
	                                   const std::string& output_frame,
	                                   int minOutputCount,
	                                   double max_distance,
	                                   double max_plane_normal_dev,
	                                   double max_plane_shift_dev
	                                   )
	    : original_frame_(original_frame)
	    , output_frame_(output_frame)
	{
		n = node;
		m_minOutputCount = minOutputCount;
		m_max_distance = max_distance;
		m_max_plane_normal_dev = max_plane_normal_dev;
		m_max_plane_shift_dev = max_plane_shift_dev;
	}

	void DynModelExporter::createMarkerForConvexHull(pcl::PointCloud<pcl::PointXYZ>& plane_cloud, pcl::ModelCoefficients& plane_coefficients, visualization_msgs::Marker& marker)
	{
		// init marker
	    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	    marker.action = visualization_msgs::Marker::ADD;

	    // project the points of the plane on the plane
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
	    pcl::ProjectInliers<pcl::PointXYZ> proj;
	    proj.setModelType (pcl::SACMODEL_PLANE);
	    proj.setInputCloud (plane_cloud.makeShared());
	    proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients> (plane_coefficients));
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
	    float dx = meanPointCP[0]-meanPointCH[0];
	    float dy = meanPointCP[1]-meanPointCH[1];
	    float dz = meanPointCP[2]-meanPointCH[2];

	    // create colored part of plane by creating marker for each triangle between neighbored points on contour of convex hull an midpoint
	    marker.points.clear();
	    for (unsigned int j = 0; j < cloud_hull->points.size(); ++j)
	    {
	    	geometry_msgs::Point p;

	        p.x = cloud_hull->points[j].x+dx; p.y = cloud_hull->points[j].y+dy; p.z = cloud_hull->points[j].z+dz;
	        marker.points.push_back( p );

	        p.x = cloud_hull->points[(j+1)%cloud_hull->points.size() ].x+dx; p.y = cloud_hull->points[(j+1)%cloud_hull->points.size()].y+dy; p.z = cloud_hull->points[(j+1)%cloud_hull->points.size()].z+dz;
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
