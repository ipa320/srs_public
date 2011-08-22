#include "ros/ros.h"
#include "srs_msgs/Component.h"
#include "srs_msgs/SRS_Action.h"
#include "gdatabase/GetObjectsOnMap.h"
#include "gdatabase/GetWorkspaceOnMap.h"
#include "gdatabase/GetObjectsOnWorkspace.h"
#include "gdatabase/GetChildObjects.h"
#include "gdatabase/GetParentObject.h"
#include "gdatabase/GetActionsByObject.h"
#include "gdatabase/GetSymbolic.h"
#include "gdatabase/UpdatePosInfo.h"
#include "gdatabase/GetInfoObject.h"
#include "gdatabase/InsertObject.h"
#include <ros/duration.h>
#include <cstdlib>
#include <stdio.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/format.hpp"
using namespace std;
using namespace boost::posix_time;
using namespace boost::gregorian;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "srs_gdatabase_client");
  ros::NodeHandle n;
  ros::ServiceClient client;
  gdatabase::UpdatePosInfo srv1;
  gdatabase::GetSymbolic srv2;
  gdatabase::GetActionsByObject srv3;
  gdatabase::GetObjectsOnMap srv4;
  gdatabase::GetChildObjects srv5;
  gdatabase::GetParentObject srv6;
  gdatabase::GetObjectsOnWorkspace srv7;
  gdatabase::GetWorkspaceOnMap srv8;
  gdatabase::GetInfoObject srv9;
  gdatabase::InsertObject srv10;
  ptime m_now;

  int service_id = atoll(argv[1]);

  if (service_id>10||service_id<1)
  {
	  ROS_INFO("service id (the first parameter) between 1 and 10");
	  return 1;
  }

  switch (service_id) {
  case 1:  //updatePosInfo
	  if (argc !=10)
	  {
	    ROS_INFO("updatePosInfo needs: 1 (as service_id), pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w, objectID");
	    return 1;
	  }
	  client = n.serviceClient<gdatabase::UpdatePosInfo>("UpdatePosInfo");

	  srv1.request.newPose.position.x = atof(argv[2]);
	  srv1.request.newPose.position.y = atof(argv[3]);
	  srv1.request.newPose.position.z = atof(argv[4]);
	  srv1.request.newPose.orientation.x = 	atof(argv[5]);
	  srv1.request.newPose.orientation.y = atof(argv[6]);
	  srv1.request.newPose.orientation.z = atof(argv[7]);
	  srv1.request.newPose.orientation.w = atof(argv[8]);

	  //cout<<'x'<<new_pose.position.x<<'y'<<new_pose.position.y<<'z'<<new_pose.position.z<<endl;
	  //srv1.request.thepose=new_pose;
	  srv1.request.objectID=atoll(argv[9]);
	  if (client.call(srv1))
	   {
		  if (srv1.response.success)
		  {
			 cout<<"New pose has been updated successfully to the object ID: "<<srv1.request.objectID<<endl;
		  }
		  else
		  {
			 cout<<"Failed to update the new pose to the object ID: "<< srv1.request.objectID<<endl;
		  }
	   }
	  else
	   {
	     ROS_ERROR("Failed to call service updatePosInfo");
	     return 1;
	   }
	  break;

  case 2:  //getSymbolic
	  if (argc !=5)
	  {
	    ROS_INFO("getSymbolic needs: 2 (as service_id), objectID, actionID, componentID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetSymbolic>("GetSymbolic");
	     // assign object ID
	  srv2.request.objectID = atoll(argv[2]);
	  srv2.request.actionID = atoll(argv[3]);
	  srv2.request.componentID = atoll(argv[4]);
	   // call the server and display the result
	  if (client.call(srv2))
	   {
	 	  int num=srv2.response.symbolicID.size();
	 	  for(int i=0;i<num;i++){
	 		  cout<<"symbolic ID: "<<srv2.response.symbolicID.at(i)<<endl;
	 	  }
	   }
	  else
	   {
	     ROS_ERROR("Failed to call service GetSymbolic");
	     return 1;
	   }
	  break;

  case 3:  //getActionsByObject
	  if (argc !=3)
	  {
	    ROS_INFO("getActionsByObject needs: 3 (as service_id), object_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetActionsByObject>("GetActionsByObject");
	    srv3.request.objectID = atoll(argv[2]);

	    // call the server and display the result
	    if (client.call(srv3))
	    {
	  	  int num=srv3.response.actionID.size();
	  	  for(int i=0;i<num;i++){
	  		  cout<<"action ID: "<<srv3.response.actionID.at(i)<<", actionName: "<<srv3.response.actionName.at(i)<<", required component"<<srv3.response.requiredComponent.at(i)<<endl;
	  	  }
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service getActionsByObject");
	      return 1;
	    }
	  break;

  case 4:  //getObjectsOnMap

	  if (argc !=3)
	  {
	    ROS_INFO("getObjectsOnMap needs: 4 (as service_id), map_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetObjectsOnMap>("GetObjectsOnMap");
	    srv4.request.mapID = atoll(argv[2]);

	    // call the server and display the result
	    if (client.call(srv4))
	    {
	  	  int num=srv4.response.objectID.size();
	  	  for(int i=0;i<num;i++){
	  		  cout<<"object ID: "<<srv4.response.objectID.at(i)<<", classID: "<<srv4.response.classID.at(i)<<endl;
	  	  }
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service GetObjectsOnMap");
	      return 1;
	    }
	  break;

  case 5:  //getChildObjects
	  if (argc !=3)
	  {
	    ROS_INFO("getChildObjects needs: 5 (as service_id), object_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetChildObjects>("GetChildObjects");
	    srv5.request.objectID = atoll(argv[2]);

	    // call the server and display the result
	    if (client.call(srv5))
	    {
	    	if (srv5.response.hasChildren)
	    	{
	    		int num=srv5.response.children_objectIDs.size();
	    		for(int i=0;i<num;i++){
	    			cout<<"children objectIDs: "<<srv5.response.children_objectIDs.at(i)<<endl;
	    		}
	    	}
	    	else
	    	{
	    		cout<<"no child for the given object"<<endl;
	    	}
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service GetChildObjects");
	      return 1;
	    }
	  break;
  case 6:  //getParentObject
	  if (argc !=3)
	  {
	    ROS_INFO("getParentObject needs: 6 (as service_id), object_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetParentObject>("GetParentObject");
	    srv6.request.objectID = atoll(argv[2]);

	    // call the server and display the result
	    if (client.call(srv6))
	    {
	    	if (srv6.response.hasParent)
	    	{
	    		int num=srv6.response.parent_object_id.size();
	    		for(int i=0;i<num;i++){
	    			cout<<"Parent objectIDs: "<<srv6.response.parent_object_id.at(i)<<endl;
	    		}
	    	}
	    	else
	    	{
	    		cout<<"no parents for the given object"<<endl;
	    	}
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service GetParentObject");
	      return 1;
	    }
	  break;

  case 7:  //getObjectsOnWorkspace
	  if (argc !=3)
	  {
	    ROS_INFO("getObjectsOnWorkspace needs: 7 (as service_id), object_ID");
	    return 1;
	  }
	  client = n.serviceClient<gdatabase::GetObjectsOnWorkspace>("GetObjectsOnWorkspace");
	   // assign map ID
	  srv7.request.objectID = atoll(argv[2]);
	   // call the server and display the result
	  if (client.call(srv7))
	   {
	 	  int num=srv7.response.objectID.size();
	 	  for(int i=0;i<num;i++){
	 		  cout<<"object ID: "<<srv7.response.objectID.at(i)<<", classID: "<<srv7.response.classID.at(i)<<endl;
	 	  }
	   }
	  else
	   {
	     ROS_ERROR("Failed to call service GetObjectsOnWorkspace");
	     return 1;
	   }
	  break;

  case 8:  //getWorkspaceOnMap

	  if (argc !=3)
	  {
	    ROS_INFO("getWorkspaceOnMap needs: 8 (as service_id), map_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetWorkspaceOnMap>("GetWorkspaceOnMap");

	    srv8.request.mapID = atoll(argv[2]);
	    if (client.call(srv8))
	    {
	  	  int num=srv8.response.objectID.size();
	  	  for(int i=0;i<num;i++){
	  		  cout<<"object ID: "<<srv8.response.objectID.at(i)<<", classID: "<<srv8.response.classID.at(i)<<endl;
	  	  }
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service GetWorkspaceOnMap");
	      return 1;
	    }
	  break;

  case 9:  //getInfoObject
	  if (argc !=3)
	  {
	    ROS_INFO("getInfoObject needs: 9 (as service_id), object_ID");
	    return 1;
	  }

	  client = n.serviceClient<gdatabase::GetInfoObject>("GetInfoObject");

	    srv9.request.objectID = atoll(argv[2]);
	    if (client.call(srv9))
	    {
	  	  int num=srv9.response.objectName.size();
	  	  if (num>0){
	  		  cout<<"object Name: "<<srv9.response.objectName<<endl<<"object Pose: "<<endl<<srv9.response.objectPose<<endl;
	  		  cout<<"related HHobject ID: "<<srv9.response.HHobjectID<<endl<<"Last seen time:"<<srv9.response.lastseenTime<<endl;
	  	  }
	    }
	    else
	    {
	      ROS_ERROR("Failed to call service getInfoObject");
	      return 1;
	    }
	  break;

  case 10:  //insertObject

	  if (argc !=12)
	  	  {
	  	    ROS_INFO("insertObject needs: 10 (as service_id), object_name, classID, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w, HHobjectID");
	  	    return 1;
	  	  }
	  	  client = n.serviceClient<gdatabase::InsertObject>("InsertObject");

	  	  srv10.request.objectName=argv[2];
	  	  srv10.request.classID=atoll(argv[3]);
	  	  srv10.request.thePose.position.x = atof(argv[4]);
	  	  srv10.request.thePose.position.y = atof(argv[5]);
	  	  srv10.request.thePose.position.z = atof(argv[6]);
	  	  srv10.request.thePose.orientation.x = atof(argv[7]);
	  	  srv10.request.thePose.orientation.y = atof(argv[8]);
	  	  srv10.request.thePose.orientation.z = atof(argv[9]);
	  	  srv10.request.thePose.orientation.w = atof(argv[10]);
	  	  srv10.request.HHobjectID = atoll(argv[11]);

	  	  m_now = second_clock::universal_time();
	  	  srv10.request.lastseenTime=::to_iso_string(m_now);

	  	  if (client.call(srv10))
	  	   {
	  		  if (srv10.response.success)
	  		  {
	  			 cout<<"New object has been inserted successfully"<<endl;
	  		  }
	  		  else
	  		  {
	  			 cout<<"Failed to insert the new  object"<< endl;
	  		  }
	  	   }
	  	  else
	  	   {
	  	     ROS_ERROR("Failed to call service updatePosInfo");
	  	     return 1;
	  	   }
	  	  break;

  default:

	  break;
  }

  return 0;
}


