#include <ros/ros.h>

#include "gdatabase/GetObjectsOnMap.h"
#include "gdatabase/GetInfoObject.h"
#include "gdatabase/GetDrawObject.h"

#include "MixedRealityServer/DrawObject.h"

#include <iostream>

#define MAP_ZOOM_FACTOR 3.2f

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ControlMRS");
	ros::NodeHandle node;

	ros::Publisher control = node.advertise<MixedRealityServer::DrawObject>("control_mrs", 10, false); //true - latched
	MixedRealityServer::DrawObject obj;
	ros::Rate rate(1.0);
	ros::ServiceClient clnt4, clnt9, clnt10;
	gdatabase::GetObjectsOnMap srv4;
  	gdatabase::GetInfoObject srv9;
  	gdatabase::GetDrawObject srv10;
  	
	clnt4 = node.serviceClient<gdatabase::GetObjectsOnMap>("GetObjectsOnMap");
	clnt9 = node.serviceClient<gdatabase::GetInfoObject>("GetInfoObject");
	clnt10 = node.serviceClient<gdatabase::GetDrawObject>("GetDrawObject");
	
	while (node.ok())
	{
		srv4.request.mapID = 1;

	    // call the server and display the result
	    if (clnt4.call(srv4))
	    {
	    	obj.command = "REMOVE";
			obj.topic = "/map";
			obj.id = -10;
		 	control.publish(obj);
		 	ROS_INFO("Removed all objects");

	  		int num_obj = srv4.response.objectID.size();
	  		for(int i = 0; i < num_obj; i++)
	  	  	{
				srv9.request.objectID = srv4.response.objectID[i];
				srv10.request.objectID = srv4.response.objectID[i];
				if (clnt9.call(srv9) && clnt10.call(srv10))
				{
					int res = srv9.response.objectName.size();
					if (res > 0)
					{
						/*std::cout<<"object Name: "<< srv9.response.objectName << std::endl
								 <<"object Pose: "<< std::endl
								 <<srv9.response.objectPose << std::endl;
						std::cout<<"related HHobject ID: "<< srv9.response.HHobjectID << std::endl
							     <<"Last seen time:"<< srv9.response.lastseenTime << std::endl;*/
						obj.command = "ADD";
						obj.topic = "/map";//"/stereo/left/image_raw";
						obj.id = srv4.response.objectID[i];
						obj.type = srv10.response.shape;
						obj.label = srv9.response.objectName;
						obj.x = MAP_ZOOM_FACTOR * srv9.response.objectPose.position.x;
						obj.y = MAP_ZOOM_FACTOR * srv9.response.objectPose.position.y;
						obj.width = MAP_ZOOM_FACTOR * srv10.response.width;
						obj.height = MAP_ZOOM_FACTOR * srv10.response.height;
						obj.angle = srv9.response.objectPose.orientation.x * 180 / M_PI;
						obj.clr = srv10.response.color;
					 	control.publish(obj);
					 	ROS_INFO("Sent object %d", srv4.response.objectID[i]);
					}
				}
				else
				{
					ROS_ERROR("Failed to call service getInfoObject");
					return 1;
				}
				
		  		
			}
	  		//std::cout<<"object ID: "<<srv4.response.objectID[i]<<", classID: "<<srv4.response.classID[i]<<std::endl;
	    }
	    else
	    {
	    	ROS_ERROR("Failed to call service GetObjectsOnMap");
	    	return 1;
	    }
	

		rate.sleep();
	}
		
/*	if (client.call(srv))
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
				listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
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
	}*/
	
	return 0;
};
