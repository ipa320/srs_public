/*
 *******************************************************************************
 * $Id: but_gui_client.cpp 157 2012-01-19 17:23:56Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 5.12.2011
 * Description:
 *      This is example!
 *******************************************************************************
 */

#include <ros/ros.h>
#include <srs_env_model/AddObject.h>
#include <srs_env_model/AddMarker.h>
#include <srs_env_model/AddBoundingBox.h>
#include <srs_env_model/AddBillboard.h>
#include <srs_env_model/AddPlane.h>
#include <but_gui/Primitive.h>
#include <but_gui/Billboard.h>
#include <srs_env_model/BillboardType.h>

// Client for comunication with but_gui
ros::ServiceClient markerClient, bboxClient, billboardClient, planeClient;

/*==============================================================================
 * Main function
 */
int main(int argc, char **argv)
{
  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "but_gui_client");

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Create a client for the but_gui service
  markerClient = n.serviceClient<srs_env_model::AddMarker> ("add_marker");
  bboxClient = n.serviceClient<srs_env_model::AddBoundingBox> ("add_bounding_box");
  billboardClient = n.serviceClient<srs_env_model::AddBillboard> ("add_billboard");
  planeClient = n.serviceClient<srs_env_model::AddPlane> ("add_plane");

  srs_env_model::AddMarker markerSrv;
  markerSrv.request.name = "Sphere marker";
  markerSrv.request.frame_id = "/world";
  markerSrv.request.description = "This is Sphere marker";
  markerSrv.request.type = Marker::SPHERE;
  markerSrv.request.pose.position.x = 1.0;
  markerSrv.request.pose.position.y = 1.0;
  markerSrv.request.pose.position.z = 1.0;
  markerSrv.request.pose.orientation.x = 0.0;
  markerSrv.request.pose.orientation.y = 0.0;
  markerSrv.request.pose.orientation.z = 0.0;
  markerSrv.request.pose.orientation.w = 1.0;
  markerSrv.request.scale.x = 2.0;
  markerSrv.request.scale.y = 3.0;
  markerSrv.request.scale.z = 4.0;
  markerSrv.request.color.r = 1.0;
  markerSrv.request.color.g = 0.0;
  markerSrv.request.color.b = 1.0;
  markerSrv.request.color.a = 1.0;
  markerClient.call(markerSrv);

  srs_env_model::AddBoundingBox bboxSrv;
  bboxSrv.request.name = "Sphere bounding box";
  bboxSrv.request.object_name = markerSrv.request.name;
  bboxSrv.request.frame_id = markerSrv.request.frame_id;
  bboxSrv.request.description = "This is Sphere Bounding Box";
  bboxSrv.request.pose = markerSrv.request.pose;
  bboxSrv.request.scale = markerSrv.request.scale;
  bboxSrv.request.color = markerSrv.request.color;
  bboxClient.call(bboxSrv);

  srs_env_model::AddBillboard billboardSrv;
  billboardSrv.request.name = "Billboard";
  billboardSrv.request.type = srs_env_model::BillboardType::PERSON;
  billboardSrv.request.frame_id = markerSrv.request.frame_id;
  billboardSrv.request.description = "This is Billboard";
  billboardSrv.request.pose = markerSrv.request.pose;
  billboardSrv.request.pose.position.x = 5.0;
  billboardSrv.request.scale = markerSrv.request.scale;
  billboardSrv.request.direction.x = 1.0;
  billboardSrv.request.direction.y = 1.0;
  billboardSrv.request.direction.z = 0.0;
  billboardSrv.request.direction.w = 1.0;
  billboardSrv.request.velocity = 3.1;
  billboardClient.call(billboardSrv);

  srs_env_model::AddPlane planeSrv;
  planeSrv.request.name = "Plane";
  planeSrv.request.frame_id = markerSrv.request.frame_id;
  planeSrv.request.description = "This is Plane";
  planeSrv.request.pose = markerSrv.request.pose;
  planeSrv.request.pose.position.y = 5.0;
  planeSrv.request.scale = markerSrv.request.scale;
  planeSrv.request.color.r = 0.7;
  planeSrv.request.color.g = 0.2;
  planeSrv.request.color.b = 0.5;
  planeSrv.request.color.a = 1.0;
  planeClient.call(planeSrv);

  ros::spinOnce(); // Call all the callbacks waiting to be called

  return 0;
}

