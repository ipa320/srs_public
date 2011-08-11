#include "ros/ros.h"
#include "gdatabase/GetObjectsOnMap.h"
#include <ros/duration.h>
#include <cstdlib>
#include <stdio.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_objects_on_map_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gdatabase::GetObjectsOnMap>("GetObjectsOnMap");
  //the service
  gdatabase::GetObjectsOnMap srv;

  // assign map ID
  srv.request.mapID = atoll(argv[1]);

  // call the server and display the result
  if (client.call(srv))
  {
	  int num=srv.response.objectID.size();
	  for(int i=0;i<num;i++){
		  cout<<"object ID: "<<srv.response.objectID.at(i)<<", classID: "<<srv.response.classID.at(i)<<endl;
	  }
  }
  else
  {
    ROS_ERROR("Failed to call service GetObjectsOnMap");
    return 1;
  }

  return 0;
}

