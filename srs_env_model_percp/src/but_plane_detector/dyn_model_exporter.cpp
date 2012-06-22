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
				pose.frame_id_ = DET_OUTPUT_PLANE_ORIGINAL_FRAMEID;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = DET_OUTPUT_PLANE_ORIGINAL_FRAMEID;
				t.transformPoint(DET_OUTPUT_PLANE_FRAMEID, pose, pose2);
				t.transformQuaternion(DET_OUTPUT_PLANE_FRAMEID, orientation, orientation2);


				std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
				srs_interaction_primitives::AddPlane planeSrv;
				planeSrv.request.frame_id = DET_OUTPUT_PLANE_FRAMEID;
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
				pose.frame_id_ = DET_OUTPUT_PLANE_ORIGINAL_FRAMEID;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = DET_OUTPUT_PLANE_ORIGINAL_FRAMEID;
				t.transformPoint(DET_OUTPUT_PLANE_FRAMEID, pose, pose2);
				t.transformQuaternion(DET_OUTPUT_PLANE_FRAMEID, orientation, orientation2);

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
		planeSrv.request.plane_array.header.frame_id = DET_OUTPUT_PLANE_FRAMEID;
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

			if (getCenterAndScale(planes[i], normals, center, scale))
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
				pose.setX(center.x);
				pose.setY(center.y);
				pose.setZ(center.z);
				pose.frame_id_ = DET_OUTPUT_PLANE_FRAMEID;

				orientation.setX(x);
				orientation.setY(y);
				orientation.setZ(z);
				orientation.setW(w);
				orientation.frame_id_ = DET_OUTPUT_PLANE_FRAMEID;


				std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
				planeDyn.id = i;
				planeDyn.pose.position.x = pose.getX();
				planeDyn.pose.position.y = pose.getY();
				planeDyn.pose.position.z = pose.getZ();
				planeDyn.pose.orientation.x = orientation.getX();
				planeDyn.pose.orientation.y = orientation.getY();
				planeDyn.pose.orientation.z = orientation.getZ();
				planeDyn.pose.orientation.w = orientation.getW();
				planeDyn.scale.x = scale.x;
				planeDyn.scale.y = scale.y;
				planeDyn.scale.z = scale.z;

				// push into array
				planeSrv.request.plane_array.planes.push_back(planeDyn);
			}
		}
		// fill in header and send
		planeSrv.request.plane_array.header.frame_id = DET_OUTPUT_PLANE_FRAMEID;
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
			 if (plane.distance(point) < 0.1 && plane.isSimilar(aaa, 0.3, 0.5))
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
		if (size > 1000)
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
				 if (plane.distance(cv::Vec3f(it->x, it->y, it->z)) < 0.05)
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
	DynModelExporter::DynModelExporter(ros::NodeHandle *node)
	{
		n = node;
	}

}// but_scenemodel
