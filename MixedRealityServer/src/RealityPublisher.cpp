#include <ros/ros.h>
#include "gdatabase/GetObjectsOnMap.h"
#include "gdatabase/GetInfoObject.h"
#include "gdatabase/GetDrawObject.h"
#include "MixedRealityServer/ActiveAreas.h"
#include <iostream>
#include "nav_msgs/GetMap.h"


#define MAP_ZOOM_FACTOR 3.2f

int main(int argc, char** argv)
{
        const std::string frame_id = "/map";

	ros::init(argc, argv, "RealityPublisher");
	ros::NodeHandle node;
	ros::Publisher control = node.advertise<MixedRealityServer::ActiveAreas>("map_active_areas", 10, false); //true - latched
	MixedRealityServer::ActiveAreas obj;
	ros::Rate rate(1.0);
	ros::ServiceClient client1, client2, client3;
	gdatabase::GetObjectsOnMap service1;
  	gdatabase::GetInfoObject   service2;
  	gdatabase::GetDrawObject   service3;
  	
	client1 = node.serviceClient<gdatabase::GetObjectsOnMap>("GetObjectsOnMap");
	client2 = node.serviceClient<gdatabase::GetInfoObject>("GetInfoObject");
	client3 = node.serviceClient<gdatabase::GetDrawObject>("GetDrawObject");

	
	while (node.ok())
	{
	    service1.request.mapID = 1;

	    if (client1.call(service1))
	    {
	  		int num_obj = service1.response.objectID.size(); //number of objects  in the General Database
                        double ppm;

                        nav_msgs::GetMap srv;
                        ros::ServiceClient client = node.serviceClient<nav_msgs::GetMap>("/static_map");
                        if (client.call(srv))
                        {
                            ppm = srv.response.map.info.resolution;
                            ROS_INFO("Map resolution is %lf",ppm);
                        }
                        obj.header.frame_id = frame_id;
                        obj.header.stamp = ros::Time::now();
                        obj.object_id.clear();
                        obj.object_name.clear();
                        obj.object_base_shape.clear();
                        obj.object_type.clear();
                        obj.x1.clear();
                        obj.y1.clear();
                        obj.x2.clear();
                        obj.y2.clear();
                        obj.angle.clear();
                        obj.graspable.clear();

	  		for(int i = 0; i < num_obj; i++)
	  	  	{
				service2.request.objectID = service1.response.objectID[i];
				service3.request.objectID = service1.response.objectID[i];
				if (client2.call(service2) && client3.call(service3))
				{
				        int res = service2.response.objectName.size();
					if (res > 0)
					{
						obj.object_id.push_back(service1.response.objectID[i]);
						obj.object_name.push_back(service2.response.objectName);
						// obj.object_base_shape.push_back(service3.response.shape);
                                                if (service3.response.shape==2) obj.object_base_shape.push_back("ellipse");
						else obj.object_base_shape.push_back("rectangle");
						obj.object_type.push_back(1);
						obj.x1.push_back(MAP_ZOOM_FACTOR * service2.response.objectPose.position.x - MAP_ZOOM_FACTOR * service2.response.objectPose.position.x/2);
						obj.x2.push_back(MAP_ZOOM_FACTOR * service2.response.objectPose.position.x + MAP_ZOOM_FACTOR * service2.response.objectPose.position.x/2);
						obj.y1.push_back(MAP_ZOOM_FACTOR * service2.response.objectPose.position.y - MAP_ZOOM_FACTOR * service2.response.objectPose.position.y/2);
						obj.y2.push_back(MAP_ZOOM_FACTOR * service2.response.objectPose.position.y + MAP_ZOOM_FACTOR * service2.response.objectPose.position.y/2);
						obj.angle.push_back(service2.response.objectPose.orientation.x * 180 / M_PI);
						obj.graspable.push_back(true);

					}

				}
				else
				{
					ROS_ERROR("Failed to call service getInfoObject");
					return 1;
				}
			control.publish(obj);
			ROS_INFO("Sent array of objects");
				
		  		
			}
	    }
	    else
	    {
	    	ROS_ERROR("Failed to call service GetObjectsOnMap");
	    	return 1;
	    }
	

		rate.sleep();
	}
		
	return 0;
};
