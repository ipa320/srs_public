/**
 * $Id: dynModelExporter.cpp 134 2012-01-12 13:52:36Z spanel $
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
#include <srs_env_model/RemoveObject.h>


namespace but_scenemodel
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void DynModelExporter::update(std::vector<Plane<float> > & planes, PointCloud<PointXYZRGB>::Ptr scene_cloud)
{
	// but dynamic model
	ros::ServiceClient plane = n->serviceClient<srs_env_model::AddPlanes> ("insert_plane");

	// Create calls
	srs_env_model::AddPlanes planeSrv;
	srs_env_model::PlaneArray planeArray;

	for (unsigned int i = 0; i < planes.size(); ++i)
	{
		float x, y, z, w;
		planes[i].getQuaternionRotation(x, y, z, w);

		PointXYZ center;
		PointXYZ scale;

		// for each plane
		if (getCenterAndScale(planes[i], scene_cloud, center, scale))
		{
			srs_env_model::PlaneDesc planeDyn;

			// test if we modify or insert
			if (managedInd.size() > i)
				planeDyn.flags = srs_env_model::PlaneDesc::MODIFY;
			else
			{
				planeDyn.flags = srs_env_model::PlaneDesc::INSERT;
				managedInd.push_back(true);
			}

			// Fill in coords
			std::cout << "Sending plane: " << planes[i].a << "x + " << planes[i].b << "y + " << planes[i].c << "z + " << planes[i].d << " = 0.0" << std::endl;
			planeDyn.id = i;
			planeDyn.pose.position.x = center.x;
			planeDyn.pose.position.y = center.y;
			planeDyn.pose.position.z = center.z;
			planeDyn.pose.orientation.x = x;
			planeDyn.pose.orientation.y = y;
			planeDyn.pose.orientation.z = z;
			planeDyn.pose.orientation.w = w;
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
bool DynModelExporter::getCenterAndScale(Plane<float> &plane, PointCloud<PointXYZRGB>::Ptr scene_cloud, PointXYZ &center, PointXYZ &scale)
{
	center.x = 0.0;
	center.y = 0.0;
	center.z = 0.0;

	float size = 0.0;
	PointXYZ min(9999999.0, 9999999.0, 9999999.0);
	PointXYZ max(-9999999.0, -9999999.0, -9999999.0);


	for (PointCloud<PointXYZRGB>::iterator it = scene_cloud->begin(); it != scene_cloud->end(); ++it)
	{
		 // 1cm
		 if (plane.distance(cv::Vec3f(it->x, it->y, it->z)) < 0.005)
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

	if (size != 0)
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
