/**
 * $Id: dynModelExporter.cpp 320 2012-03-09 13:27:35Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: dd.mm.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 */

#include "plane_det/dynModelExporter.h"
#include <srs_env_model/AddPlane.h>
#include <srs_env_model/AddPlanes.h>
#include <srs_env_model/RemovePrimitive.h>


namespace but_scenemodel
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynModelExporter::update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud, tf::StampedTransform &sensorToWorldTf)
{
	// but dynamic model
	ros::ServiceClient plane = n->serviceClient<srs_env_model::AddPlanes> ("insert_plane");

	// Create calls
	srs_env_model::AddPlanes planeSrv;
	srs_env_model_msgs::PlaneArray planeArray;

	for (unsigned int i = 0; i < planes.size(); ++i)
	{
		float x, y, z, w;
		planes[i].getQuaternionRotation(x, y, z, w);

		PointXYZ center;
		PointXYZ scale;

		// for each plane
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

			tf::Stamped<btVector3> pose, pose2;
			tf::Stamped<btQuaternion> orientation, orientation2;
			pose.setX(center.x);
			pose.setY(center.y);
			pose.setZ(center.z);
			pose.frame_id_ = "/head_cam3d_link";

			orientation.setX(x);
			orientation.setY(y);
			orientation.setZ(z);
			orientation.setW(w);
			orientation.frame_id_ = "/head_cam3d_link";
			t.transformPoint("/map", pose, pose2);
			t.transformQuaternion("/map", orientation, orientation2);



			std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
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
	planeSrv.request.plane_array.header.frame_id = "/map";
	planeSrv.request.plane_array.header.stamp = ros::Time::now();

	plane.call(planeSrv);
}

void DynModelExporter::update(std::vector<Plane<float> > & planes, pcl::PointCloud<PointXYZRGB>::Ptr scene_cloud)
{
	// but dynamic model
	ros::ServiceClient plane = n->serviceClient<srs_env_model::AddPlanes> ("insert_plane");

	// Create calls
	srs_env_model::AddPlanes planeSrv;
	srs_env_model_msgs::PlaneArray planeArray;

	for (unsigned int i = 0; i < planes.size(); ++i)
	{
		float x, y, z, w;
		planes[i].getQuaternionRotation(x, y, z, w);

		PointXYZ center;
		PointXYZ scale;

		// for each plane
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
			tf::Stamped<btVector3> pose;
			tf::Stamped<btQuaternion> orientation;
			pose.setX(center.x);
			pose.setY(center.y);
			pose.setZ(center.z);
			pose.frame_id_ = "/head_cam3d_link";

			orientation.setX(x);
			orientation.setY(y);
			orientation.setZ(z);
			orientation.setW(w);
			orientation.frame_id_ = "/head_cam3d_link";


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
	planeSrv.request.plane_array.header.frame_id = "/map";
	planeSrv.request.plane_array.header.stamp = ros::Time::now();

	plane.call(planeSrv);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
		 // 1cm
		 if (plane.distance(cv::Vec3f(it->x, it->y, it->z)) < 0.01)
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

	if (size > 10)
	{
		center.x /= size;
		center.y /= size;
		center.z /= size;
	}
	else return false;


	scale.x = max.x - min.x;
	scale.y = max.y - min.y;
	scale.z = max.z - min.z;

	return true;
}

DynModelExporter::DynModelExporter(ros::NodeHandle *node)
{
	n = node;
}

}
