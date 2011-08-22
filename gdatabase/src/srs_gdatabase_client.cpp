#include "ros/ros.h"
#include "gdatabase/GetObjectsOnMap.h"
#include "gdatabase/GetWorkspaceOnMap.h"
#include "gdatabase/GetObjectsOnWorkspace.h"
#include "gdatabase/GetChildObjects.h"
#include "gdatabase/GetParentObject.h"
#include <ros/duration.h>
#include <cstdlib>
#include <cstdio>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "srs_gdatabase_client");

  ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<gdatabase::GetObjectsOnWorkspace>("GetObjectsOnWorkspace");
   //the service
   gdatabase::GetObjectsOnWorkspace srv;

   // assign map ID
   srv.request.objectID = atol(argv[1]);

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
     ROS_ERROR("Failed to call service GetObjectsOnWorkspace");
     return 1;
   }


  /*
  ros::NodeHandle wm;
  ros::ServiceClient clientworkspaceonmap = wm.serviceClient<gdatabase::GetWorkspaceOnMap>("GetWorkspaceOnMap");
  gdatabase::GetWorkspaceOnMap srv;
  srv.request.mapID = atoll(argv[1]);
  if (clientworkspaceonmap.call(srv))
  {
	  int num=srv.response.objectID.size();
	  for(int i=0;i<num;i++){
		  cout<<"object ID: "<<srv.response.objectID.at(i)<<", classID: "<<srv.response.classID.at(i)<<endl;
	  }
  }
  else
  {
    ROS_ERROR("Failed to call service GetWorkspaceOnMap");
    return 1;
  }
  */

  /*

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

   */


  return 0;
}


