#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <mysql/mysql.h>
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
#include "gdatabase/GetDrawObject.h"
#include "gdatabase/InsertObject.h"
#include <sstream>
#include <ros/duration.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/format.hpp"
using namespace std;
using namespace boost::posix_time;
using namespace boost::gregorian;
MYSQL *connection;
MYSQL mysql;
MYSQL_RES *result;
MYSQL_ROW row;
int query_state;

bool updatePosInfo(gdatabase::UpdatePosInfo::Request  &req,
		gdatabase::UpdatePosInfo::Response &res )
{
	ROS_INFO("updatePosInfo is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}
	res.success=false;
	string checking_String("SELECT object_name FROM object WHERE object_id=");
		ostringstream object;
		object<<req.objectID;
		checking_String+=object.str();
		ROS_INFO("%s\n", checking_String.c_str());
		mysql_query(connection,checking_String.c_str());
		if (mysql_num_rows(mysql_store_result(connection))==0)
		{
			ROS_INFO("Object ID does not exist in the database");
			return 1;
		}


	string query_String("UPDATE object SET position_x=");
	ostringstream para;
	para<<req.newPose.position.x;
	para<<", position_y=";
	para<<req.newPose.position.y;
	para<<", position_z=";
	para<<req.newPose.position.z;
	para<<", orientation_x=";
	para<<req.newPose.orientation.x;
	para<<", orientation_y=";
	para<<req.newPose.orientation.y;
	para<<", orientation_z=";
	para<<req.newPose.orientation.z;
	para<<", orientation_w=";
	para<<req.newPose.orientation.w;
	para<<", time='";
	ptime m_now = second_clock::universal_time();
	std::string s_t = ::to_iso_string(m_now);
	para<<s_t;
	para<<"' WHERE object_id=";
	para<<req.objectID;
	query_String+=para.str();

	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());


	if (mysql_query(connection,query_String.c_str())==0)
	{
		res.success=true;
	}

	mysql_close(connection);
	return true;
}


bool getSymbolic(gdatabase::GetSymbolic::Request  &req,
		gdatabase::GetSymbolic::Response &res )
{
	ROS_INFO("getSymbolic is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT symbolic_id FROM oacs WHERE object_id=");
	ostringstream para;
	para<<req.objectID;
	para<<" and action_id=";
	para<<req.actionID;
	para<<" and component_id=";
	para<<req.componentID;
	query_String+=para.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);

	while ((row = mysql_fetch_row(result))){
		int syb_id=atoi(row[0]);
		res.symbolicID.push_back(syb_id);
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}


bool getActionsByObject(gdatabase::GetActionsByObject::Request  &req,
		gdatabase::GetActionsByObject::Response &res )
{
	ROS_INFO("getActionsByObject is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT action.action_id, action.action_name, component.component_id, component.component_name FROM action JOIN oacs ON action.action_id = oacs.action_id JOIN component ON component.component_id=oacs.component_id WHERE oacs.object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	srs_msgs::Component component;
	while ((row = mysql_fetch_row(result))){
		int ac_id=atoi(row[0]);
		string ac_name=row[1];
		component.componentID = atoi(row[2]);
		component.componentName = row[3];
	  res.actionID.push_back(ac_id);
	  res.actionName.push_back(ac_name);
	  res.requiredComponent.push_back(component);

	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}


bool getObjectsOnMap(gdatabase::GetObjectsOnMap::Request  &req,
		gdatabase::GetObjectsOnMap::Response &res )
{
	ROS_INFO("getObjectsOnMap is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT object_id,object_class_id FROM object WHERE map_id=");
	ostringstream map;
	map<<req.mapID;
	query_String+=map.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	while ((row = mysql_fetch_row(result))){
		int ob_id=atoi(row[0]);
		int cl_id=atoi(row[1]);
	  res.objectID.push_back(ob_id);
	  res.classID.push_back(cl_id);
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}


bool getChildObjects(gdatabase::GetChildObjects::Request  &req,
		gdatabase::GetChildObjects::Response &res )
{
	ROS_INFO("getChildObjects is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT child_object_id FROM semantic WHERE semantic.parent_object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	res.hasChildren=false;
	while ((row = mysql_fetch_row(result))){
		int ob_id=atoi(row[0]);
	  res.children_objectIDs.push_back(ob_id);
	  res.hasChildren=true;
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}

bool getParentObject(gdatabase::GetParentObject::Request  &req,
		gdatabase::GetParentObject::Response &res )
{
	ROS_INFO("getParentObject is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT parent_object_id FROM semantic WHERE semantic.child_object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	res.hasParent=false;
	while ((row = mysql_fetch_row(result))){
		int ob_id=atoi(row[0]);
		res.parent_object_id.push_back(ob_id);
		res.hasParent=true;
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}

bool getObjectsOnWorkspace(gdatabase::GetObjectsOnWorkspace::Request  &req,
		gdatabase::GetObjectsOnWorkspace::Response &res )
{
	ROS_INFO("GetObjectsOnWorkspace is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT object_id,object_class_id FROM object JOIN semantic ON object_id=semantic.child_object_id WHERE semantic.parent_object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	while ((row = mysql_fetch_row(result))){
		int ob_id=atoi(row[0]);
		int cl_id=atoi(row[1]);
	  res.objectID.push_back(ob_id);
	  res.classID.push_back(cl_id);
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}


bool getWorkspaceOnMap(gdatabase::GetWorkspaceOnMap::Request  &req,
		gdatabase::GetWorkspaceOnMap::Response &res )
{
	ROS_INFO("getWorkspaceOnMap is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("SELECT object_id,object_class_id FROM object JOIN semantic ON object_id=semantic.parent_object_id WHERE map_id=");
	ostringstream map;
	map<<req.mapID;
	query_String+=map.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	while ((row = mysql_fetch_row(result))){
		int ob_id=atoi(row[0]);
		int cl_id=atoi(row[1]);
	  res.objectID.push_back(ob_id);
	  res.classID.push_back(cl_id);
	}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}

bool getDrawObject(gdatabase::GetDrawObject::Request  &req,
		gdatabase::GetDrawObject::Response &res )
{
	ROS_INFO("getDrawObject is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("Select object_shape, object_color, object_width, object_height from object WHERE object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	row = mysql_fetch_row(result);
        res.shape=atol(row[0]);
        res.color=row[1];
        res.width=atof(row[2]);
        res.height=atof(row[3]);
	mysql_free_result(result);
	mysql_close(connection);
	return true;

}
bool getInfoObject(gdatabase::GetInfoObject::Request  &req,
		gdatabase::GetInfoObject::Response &res )
{
	ROS_INFO("getInfoObject is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}

	string query_String("Select object_name, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, object_class_id, HH_object_ID, time from object WHERE object_id=");
	ostringstream object;
	object<<req.objectID;
	query_String+=object.str();
	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	ROS_INFO("%s\n", query_String.c_str());

	mysql_query(connection,query_String.c_str());
	result=mysql_store_result(connection);
	row = mysql_fetch_row(result);
			if (::size_t(row[0])>0){res.objectName=row[0];}
			if (::size_t(row[1])>0){
			res.objectPose.position.x=atof(row[1]);
			res.objectPose.position.y=atof(row[2]);
			res.objectPose.position.z=atof(row[3]);
			res.objectPose.orientation.x=atof(row[4]);
			res.objectPose.orientation.y=atof(row[5]);
			res.objectPose.orientation.z=atof(row[6]);
			res.objectPose.orientation.w=atof(row[7]);
			}
			if (::size_t(row[8])>0){res.classID=atoll(row[8]);}else{res.classID=0;}
			if (::size_t(row[9])>0){res.HHobjectID=atoll(row[9]);}else{res.HHobjectID=0;}
			if (::size_t(row[10])>0){res.lastseenTime=row[10];}

	mysql_free_result(result);
	mysql_close(connection);
	return true;
}

bool insertObject(gdatabase::InsertObject::Request  &req,
		gdatabase::InsertObject::Response &res )
{
	ROS_INFO("insertObject is called.");
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","srs","srs","srs_gdatabase",0,0,0);
	if (connection==NULL){
		cout<<mysql_error(&mysql)<<endl;
		return 1;
	}


	string checking_String("SELECT object_id FROM object WHERE object_name=");
	ostringstream object;
	object<<"'"<<req.objectName<<"'";
	checking_String+=object.str();
	ROS_INFO("%s\n", checking_String.c_str());
	mysql_query(connection,checking_String.c_str());
	if (mysql_num_rows(mysql_store_result(connection))>0)
	{
		ROS_INFO("Object name already exist in the database");
		return 1;
	}

	string query_String("INSERT INTO object (object_name, object_class_id, position_x, position_y,position_z,orientation_x,orientation_y,orientation_z, orientation_w, HH_object_ID,time ) VALUES (");
	ostringstream para;
	para<<"'";
	para<<req.objectName;
	para<<"'";
	para<<", ";
	para<<req.classID;
	para<<", ";
	para<<req.thePose.position.x;
	para<<", ";
	para<<req.thePose.position.y;
	para<<", ";
	para<<req.thePose.position.z;
	para<<", ";
	para<<req.thePose.orientation.x;
	para<<", ";
	para<<req.thePose.orientation.y;
	para<<", ";
	para<<req.thePose.orientation.z;
	para<<", ";
	para<<req.thePose.orientation.w;
	para<<", ";
	para<<req.HHobjectID;
	para<<", ";
	para<<"'";
	para<<req.lastseenTime;
	para<<"'";
	para<<" )";

	if (query_state!=0){
		cout<<mysql_error(connection)<<endl;
		return 1;
	}
	query_String+=para.str();
	ROS_INFO("%s\n", query_String.c_str());

	res.success=false;
	if (mysql_query(connection,query_String.c_str())==0)
	{
		res.success=true;
	}
	mysql_close(connection);
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "srs_gdatabase");
	ros::NodeHandle n;
	ros::ServiceServer GetObjectsOnMap = n.advertiseService("GetObjectsOnMap", getObjectsOnMap);
	ros::ServiceServer GetWorkspaceOnMap = n.advertiseService("GetWorkspaceOnMap", getWorkspaceOnMap);
	ros::ServiceServer GetObjectsOnWorkspace = n.advertiseService("GetObjectsOnWorkspace", getObjectsOnWorkspace);
	ros::ServiceServer GetParentObject = n.advertiseService("GetParentObject", getParentObject);
	ros::ServiceServer GetChildObjects = n.advertiseService("GetChildObjects", getChildObjects);
	ros::ServiceServer GetActionsByObject = n.advertiseService("GetActionsByObject", getActionsByObject);
	ros::ServiceServer GetSymbolic = n.advertiseService("GetSymbolic", getSymbolic);
	ros::ServiceServer UpdatePosInfo = n.advertiseService("UpdatePosInfo", updatePosInfo);
	ros::ServiceServer GetInfoObject = n.advertiseService("GetInfoObject", getInfoObject);
	ros::ServiceServer GetDrawObject = n.advertiseService("GetDrawObject", getDrawObject);
	ros::ServiceServer InsertObject = n.advertiseService("InsertObject",insertObject);

	ROS_INFO("srs_gdatabase Running.");

	ros::spin();

	return 0;
}
