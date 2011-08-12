#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "MapTF");
	ros::NodeHandle node;

	ros::Publisher map_coords = node.advertise<geometry_msgs::Pose2D>("maptf", 10);
	tf::TransformListener listener;
	
	ros::ServiceClient client = node.serviceClient<nav_msgs::GetMap>("/static_map");
	nav_msgs::GetMap srv;
	if (client.call(srv))
	{
		int w = 0, h = 0;
		double ppm;
		w = srv.response.map.info.width;
		h = srv.response.map.info.height;
		ppm = srv.response.map.info.resolution;
		ros::Rate rate(10.0);
		while (node.ok())
		{
			tf::StampedTransform transform;
			try
			{
				listener.lookupTransform("/odom_combined", "/map", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
			}

			geometry_msgs::Pose2D pose;
			
			pose.x = transform.getOrigin().x() / ppm + w / 2;
			pose.y = h - (transform.getOrigin().y() / ppm + h / 2);
			
			ROS_INFO("%d %d %lf %lf %lf %lf %lf", w, h, ppm, pose.x, pose.y, transform.getOrigin().x(), transform.getOrigin().y());
						
			pose.theta = transform.getRotation().z();

			map_coords.publish(pose);

			rate.sleep();
		}
	}
	else
	{
		ROS_ERROR("Failed to call service GetMap");
		return 1;
	}
	
	return 0;
};
