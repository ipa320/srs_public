#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <mysql/mysql.h>
#include "gdatabase/GetObjectsOnMap.h"
#include <sstream>
#include <ros/duration.h>
using namespace std;
MYSQL *connection;
MYSQL mysql;
MYSQL_RES *result;
MYSQL_ROW row;
int query_state;

bool getObjectsOnMap(gdatabase::GetObjectsOnMap::Request  &req,
		gdatabase::GetObjectsOnMap::Response &res )
{
	mysql_init(&mysql);
	connection=mysql_real_connect(&mysql,"localhost","root","srs","srs_database",0,0,0);
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_objects_on_map_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("GetObjectsOnMap", getObjectsOnMap);
	ROS_INFO("Ready to get objects on map.");
	ros::spin();

	return 0;
}
