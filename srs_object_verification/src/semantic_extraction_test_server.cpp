#include "ros/ros.h"

#include <cob_3d_mapping_msgs/GetObjectsOfClass.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>


bool serviceCb(cob_3d_mapping_msgs::GetObjectsOfClass::Request  &req,
		cob_3d_mapping_msgs::GetObjectsOfClass::Response &res )
{
	if(req.class_id.data != 1)
	{
		ROS_WARN("Object class not supported");
		return false;
	}
	cob_3d_mapping_msgs::Shape table1;
	table1.type = cob_3d_mapping_msgs::Shape::POLYGON;
	table1.params.push_back(0);
	table1.params.push_back(0);
	table1.params.push_back(1);
	table1.params.push_back(0.8);
	table1.params.push_back(0);
	table1.params.push_back(0);
	table1.params.push_back(0);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointXYZ p;
	p.x = -0.5;
	p.y = -0.5;
	p.z = 0.8;
	cloud.points.push_back(p);
	p.x = 0.5;
	cloud.points.push_back(p);
	p.y = 0.5;
	cloud.points.push_back(p);
	p.x = -0.5;
	cloud.points.push_back(p);
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);
	table1.points.push_back(cloud_msg);

	cob_3d_mapping_msgs::Shape table2;
	table2.type = cob_3d_mapping_msgs::Shape::POLYGON;
	table2.params.push_back(0);
	table2.params.push_back(0);
	table2.params.push_back(1);
	table2.params.push_back(0.8);
	table2.params.push_back(5);
	table2.params.push_back(5);
	table2.params.push_back(0);
	p.x = 4.5;
	p.y = 4.5;
	p.z = 0.8;
	cloud.points.push_back(p);
	p.x = 5.5;
	cloud.points.push_back(p);
	p.y = 5.5;
	cloud.points.push_back(p);
	p.x = 4.5;
	cloud.points.push_back(p);
	pcl::toROSMsg(cloud, cloud_msg);
	table2.points.push_back(cloud_msg);

	res.objects.shapes.push_back(table1);
	res.objects.shapes.push_back(table2);

	ROS_INFO("sending back response");
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_objects_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_objects_of_class", serviceCb);
  ROS_INFO("Ready to return objects.");
  ros::spin();

  return 0;
}

